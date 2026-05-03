#include "ui.h"

#include <math.h>

#include "cmsis_os.h"
#include "referee.h"
#include "referee_ui.h"

/*
 * 本文件负责 infantry_wheel_legged_sjtu 机器人的自定义裁判系统 UI。
 * 当前绘制的主要内容是一个底盘相对角度指示器：
 * 白色圆环作为参考背景，绿色圆弧表示底盘相对云台/车体参考方向的角度。
 *
 * 裁判系统 UI 的坐标和角度约定：
 * - 屏幕中心点在 ui.h 中定义。
 * - UI 圆弧接口使用 [0, 360] 范围内的角度值。
 * - 机器人侧的角度会先转换到屏幕坐标系，使 0 度显示在圆环正上方。
 */
#define UI_GRAPH_LAYER 7
#define UI_RELATIVE_RADIUS 88
#define UI_RELATIVE_ARC_HALF_ANGLE 16.0f
#define UI_RELATIVE_RING_WIDTH 3
#define UI_RELATIVE_ARC_WIDTH 8
#define UI_TASK_PERIOD_MS 30
#define UI_AUTO_REFRESH_PERIOD_MS 10000 //5秒刷新一次UI
#define UI_AUTO_REFRESH_INTERVAL_TICKS ((UI_AUTO_REFRESH_PERIOD_MS + UI_TASK_PERIOD_MS - 1) / UI_TASK_PERIOD_MS)

float watch_data1,watch_data2;

/* 缓存 RobotInstance 中的裁判系统数据指针，便于本文件各函数直接访问。 */
static referee_info_t *referee_recv_info;

/*
 * 本文件维护的 UI 交互状态。
 * 其中保存了当前需要显示的数据，以及用于判断本轮 UITask 是否需要刷新图形的脏标志。
 */
static Referee_Interactive_info_t interactive_data;

/*
 * UI 通信序号的预留变量。
 * 当前文件暂时没有使用它，但项目内其他 UI 模块通常会保留一个全局 UI 序号计数。
 */
uint8_t UI_Seq;

/*
 * 图形描述符使用 static 保存，保证多次函数调用之间内存一直有效。
 * UICircleDraw() / UIArcDraw() 会先填充这些结构体，
 * 然后再通过 UIGraphRefresh() 发送给客户端。
 */
static Graph_Data_t UI_RelativeRing;
static Graph_Data_t UI_RelativeArc[2];

static void DeterminRobotID(void) {
  /*
   * 客户端 ID 和发送方 ID 依赖裁判系统下发的机器人 ID。
   * 红方机器人 ID 通常为 1..7，蓝方机器人 ID 大于 7。
   * 客户端 ID 按标准规则计算：0x0100 + robot_id。
   */
  referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
  referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
  referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;
  referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static float NormalizeAngle(float angle) {
  /*
   * 将所有 UI 角度归一化到 [0, 360) 半开区间。
   * 这样 -10 度、370 度等越界角度在传入裁判系统绘图接口前都会被修正。
   */
  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  return angle;
}

static uint32_t AngleToUiDegree(float angle) {
  /*
   * 绘图接口使用整数角度。
   * 强制类型转换前加 0.5f，用于四舍五入到最近的整数角度，而不是简单截断小数。
   */
  uint32_t degree = (uint32_t)(NormalizeAngle(angle) + 0.5f);

  /*
   * 359.6 度四舍五入后可能变成 360。
   * 这里再把 360 映射回 0，保证返回值始终位于 [0, 359]。
   */
  return degree >= 360 ? degree - 360 : degree;
}

static float GetRelativeAngle(RobotInstance *robot) {
  /*
   * 默认使用 robot->offset_angle。
   * 在 ONE_BOARD 单板构建下，没有独立底盘板上传的数据包，因此只能使用该来源。
   */
  float angle = robot->offset_angle;
#if !defined(ONE_BOARD)
  /*
   * 在双板构建下，如果底盘板数据可用，则优先使用底盘板上报的 UI 角度。
   * 该字段放大了 10 倍保存，例如 123 表示 12.3 度。
   */
  if (robot->chassis_fetch_data) {
    angle = (float)robot->chassis_fetch_data->ui_chassis_relative_angle_deg_x10 * 0.1f;
  }
#endif
  return angle;
}

static void DrawRelativePosition(float offset_angle, uint32_t operate) {
  /*
   * 绘制一个白色圆环和一个绿色方向圆弧。
   * operate 会直接传给裁判系统 UI 辅助函数，
   * 调用者通过它决定本次是初始化时的 ADD 操作，还是运行中的 CHANGE 操作。
   */
  const int32_t center_x = UI_RELATIVE_CENTER_X;
  const int32_t center_y = UI_RELATIVE_CENTER_Y;

  /*
   * 将机器人侧的相对角度转换成 UI 坐标系角度。
   * 使用 90 度减去 offset_angle，是为了把机器人坐标系映射到屏幕显示习惯：
   * 圆环正上方作为视觉上的前方。
   */
  const float ui_angle = NormalizeAngle(offset_angle);

  /*
   * 绿色指示区域是一个以 ui_angle 为中心的对称圆弧。
   * UI_RELATIVE_ARC_HALF_ANGLE 控制这个方向扇区的一半宽度。
   */
  const float start_angle = NormalizeAngle(ui_angle - UI_RELATIVE_ARC_HALF_ANGLE);
  const float end_angle = NormalizeAngle(ui_angle + UI_RELATIVE_ARC_HALF_ANGLE);

  watch_data1 = start_angle;
  watch_data2 = end_angle;
  /* 绘制背景参考圆环。 */
  UICircleDraw(&UI_RelativeRing, "rg0", operate, UI_GRAPH_LAYER, UI_Color_White, UI_RELATIVE_RING_WIDTH, center_x,
               center_y, UI_RELATIVE_RADIUS);

  if (start_angle <= end_angle) {
    /*
     * 普通情况：圆弧没有跨过 0 度，可以用一段绿色圆弧直接表示。
     * 第二个图形发送一段很小的黑色圆弧，用于清除或隐藏上一帧可能存在的分裂圆弧。
     */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start_angle),
              AngleToUiDegree(end_angle), UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS,
              UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Black, 0, 1, UI_RELATIVE_ARC_WIDTH,
              center_x, center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  } else {
    /*
     * 跨 0 度情况：目标圆弧跨过 0 度，例如 350..10。
     * 裁判系统圆弧接口无法用一个连续区间表达这种情况，
     * 因此拆成两段绘制：start..360 和 0..end。
     */
    UIArcDraw(&UI_RelativeArc[0], "sa0", operate, UI_GRAPH_LAYER, UI_Color_Green, AngleToUiDegree(start_angle), 360,
              UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
    UIArcDraw(&UI_RelativeArc[1], "sa1", operate, UI_GRAPH_LAYER, UI_Color_Green, 0, AngleToUiDegree(end_angle),
              UI_RELATIVE_ARC_WIDTH, center_x, center_y, UI_RELATIVE_RADIUS, UI_RELATIVE_RADIUS);
  }

  /*
   * 将圆环和圆弧发送到客户端。
   * 圆环和圆弧分开发送，是因为 UIGraphRefresh() 的第二个参数表示后面跟随的图形数量。
   */
  UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RelativeRing);
  UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_RelativeArc[0], UI_RelativeArc[1]);
}

static void UIChangeCheck(Referee_Interactive_info_t *data) {
  /*
   * 避免因为角度微小抖动而频繁刷新裁判系统 UI。
   * 只有当前显示的底盘相对角度变化超过 1 度时，才请求重绘。
   */
  if (fabsf(data->chassis_relative_angle - data->last_chassis_relative_angle) > 1.0f) {
    data->UI_Interactive_Flag.relative_flag = 1;
    data->last_chassis_relative_angle = data->chassis_relative_angle;
  }
}

static void MyUIRefresh(Referee_Interactive_info_t *data) {
  /*
   * 基于脏标志刷新 UI：只绘制发生变化的 UI 元素。
   * 这样可以减少裁判系统交互数据链路上的带宽占用。
   */
  if (data->UI_Interactive_Flag.relative_flag) {
    DrawRelativePosition(data->chassis_relative_angle, UI_Graph_Change);
    data->UI_Interactive_Flag.relative_flag = 0;
  }
}

void MyUIInit(RobotInstance *robot) {
  /* 保存最新的裁判系统数据指针，防止 RobotInstance 重建后仍使用旧指针。 */
  referee_recv_info = robot->referee_data;

  /*
   * 启动后机器人 ID 不一定立即有效。
   * 必须等裁判系统上报非 0 ID 后，才能计算客户端 ID 并发送 UI 数据包。
   */
  while (referee_recv_info->GameRobotState.robot_id == 0) {
    osDelay(100);
  }

  /*
   * 配置发送方/客户端 ID，清空客户端上的旧图形，
   * 然后添加初始的相对位置指示器。
   */
  DeterminRobotID();
  UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);

  /*
   * 将当前角度和上一次角度初始化为同一个值。
   * 这样下面执行 ADD 操作后，UIChangeCheck() 不会立刻再触发一次多余的 CHANGE。
   */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);
  interactive_data.last_chassis_relative_angle = interactive_data.chassis_relative_angle;
  DrawRelativePosition(interactive_data.chassis_relative_angle, UI_Graph_ADD);
}

/*
 * 对外暴露 UI 状态，其他模块可以通过该指针请求强制刷新，
 * 或查看当前的交互数据。
 */
Referee_Interactive_info_t *getUI(void) { return &interactive_data; }

void UITask(RobotInstance *robot) {
  /*
   * UITask 预期由 RTOS 任务周期性调用。
   * 每次调用都会采样最新机器人数据，检查 UI 是否需要更新，
   * 并只发送必要的图形变更。
   */
  referee_recv_info = robot->referee_data;

  static uint16_t auto_refresh_counter = 0;
  uint8_t need_full_refresh = interactive_data.force_refresh_ui == 1;

  if (++auto_refresh_counter >= UI_AUTO_REFRESH_INTERVAL_TICKS) {
    auto_refresh_counter = 0;
    need_full_refresh = 1;
  }

  if (need_full_refresh) {
    /*
     * 强制刷新会重新构建全部 UI 图形。
     * 适用于客户端重连、手动删除 UI、Ctrl 手动刷新、2 秒自动刷新，
     * 或者客户端图形状态可能和本地状态不一致的情况。
     */
    MyUIInit(robot);
    interactive_data.force_refresh_ui = 0;
    auto_refresh_counter = 0;
  }

  static uint16_t slow_refresh_counter = 0;
  if (++slow_refresh_counter >= 150) {
    /*
     * 即使角度没有变化，也周期性刷新相对角度指示器。
     * 当裁判系统数据包丢失，或客户端漏掉上一帧更新时，这能让 UI 自动恢复。
     */
    slow_refresh_counter = 0;
    interactive_data.UI_Interactive_Flag.relative_flag = 1;
  }

  /* 处理完可能的重新初始化后，再采样最新角度。 */
  interactive_data.chassis_relative_angle = GetRelativeAngle(robot);

  /*
   * 先根据数据变化设置脏标志，再绘制所有脏 UI 元素。
   * 当前只有底盘相对角度指示器使用这套机制。
   */
  UIChangeCheck(&interactive_data);
  MyUIRefresh(&interactive_data);
}
