# infantry_wheel_legged_sjtu UI 模块

## 概述

`ui` 模块负责在裁判系统客户端绘制步兵轮腿机器人的驾驶员界面。当前实现只绘制一个底盘相对角度指示器：

- 白色圆环：固定参考圆。
- 绿色圆弧：表示底盘相对角度方向。

底层绘图接口来自 `Modules/referee/referee_UI.*`，图形通过学生机器人交互数据下发到裁判系统客户端。

## 文件

| 文件 | 作用 |
| --- | --- |
| `ui.h` | 定义 UI 坐标、状态缓存结构体、刷新标志位和对外接口。 |
| `ui.c` | 实现 UI 初始化、角度采样、变化检测和周期刷新。 |

## 对外接口

| 接口 | 作用 |
| --- | --- |
| `MyUIInit(RobotInstance *robot)` | 等待裁判系统机器人 ID 有效，删除旧 UI，并 ADD 初始相对角度指示器。 |
| `UITask(RobotInstance *robot)` | 周期采样相对角度，按变化标志 CHANGE 指示器。 |
| `getUI(void)` | 返回模块内部 `interactive_data`，供其他模块置位 `force_refresh_ui`。 |

## 运行位置

UI 任务在 `UserApp/os_task.c` 的 `StartUITASK()` 中运行，仅在 `ONE_BOARD` 或 `CHASSIS_BOARD` 编译条件下创建。

启动流程：

1. 等待 `RobotGetInstance()` 非空。
2. 等待 `robot->referee_data` 非空，避免 UI 初始化时访问空指针。
3. 调用 `MyUIInit()` 首次绘制。
4. 每 `30 ms` 调用一次 `UITask()` 增量刷新。

双板模式下，UI 通常由底盘板负责绘制。云台板通过 `Chassis_Fetch_Data_s` 把 UI 需要的 `ui_chassis_relative_angle_deg_x10` 和 `force_refresh_ui` 发给底盘板。

## 显示内容

当前图形位于屏幕中心：

| 图元 | 名称 | 图层 | 说明 |
| --- | --- | --- | --- |
| `UI_RelativeRing` | `rg0` | `UI_GRAPH_LAYER` | 白色参考圆环，圆心 `UI_RELATIVE_CENTER_X/Y`，半径 `UI_RELATIVE_RADIUS`。 |
| `UI_RelativeArc[0]` | `sa0` | `UI_GRAPH_LAYER` | 绿色相对角度主圆弧。 |
| `UI_RelativeArc[1]` | `sa1` | `UI_GRAPH_LAYER` | 跨 0 度时的第二段圆弧；非跨 0 度时用黑色短弧隐藏上一帧残留。 |

右下角显示右腿 `leg[0]` 的并联五连杆位姿：

| 图元 | 名称 | 说明 |
| --- | --- | --- |
| `UI_LegRods[0]` | `lg0` | A-B，主动杆 `l1`。 |
| `UI_LegRods[1]` | `lg1` | B-C，从动杆 `l2`。 |
| `UI_LegRods[2]` | `lg2` | C-D，从动杆 `l3`。 |
| `UI_LegRods[3]` | `lg3` | D-E，主动杆 `l4`。 |
| `UI_LegRods[4]` | `lg4` | A-E，机身侧固定杆 `l5`。 |

相关参数：

| 宏 | 当前值 | 说明 |
| --- | --- | --- |
| `UI_RELATIVE_CENTER_X` | `960` | 指示器圆心 X。 |
| `UI_RELATIVE_CENTER_Y` | `540` | 指示器圆心 Y。 |
| `UI_RELATIVE_RADIUS` | `88` | 圆环和圆弧半径。 |
| `UI_RELATIVE_RING_WIDTH` | `3` | 白色参考圆环线宽。 |
| `UI_RELATIVE_ARC_WIDTH` | `8` | 绿色方向圆弧线宽。 |
| `UI_RELATIVE_ARC_HALF_ANGLE` | `16.0f` | 绿色圆弧半宽，最终显示宽度约 `32 deg`。 |
| `UI_AUTO_REFRESH_PERIOD_MS` | `2000` | 自动全量刷新周期，单位 ms。 |
| `UI_LEG_BASE_LEFT_X` | `1700` | 五连杆左上固定点 A 的 UI X 坐标。 |
| `UI_LEG_BASE_Y` | `200` | 五连杆固定点 A/E 的 UI Y 坐标。 |
| `UI_LEG_SCALE` | `520.0f` | 五连杆物理长度到屏幕像素的缩放比例。 |
| `UI_LEG_REFRESH_PERIOD_MS` | `100` | 五连杆姿态周期刷新时间，单位 ms。 |

## 角度处理

`GetRelativeAngle()` 获取底盘相对角：

- `ONE_BOARD` 下使用 `robot->offset_angle`。
- 双板下如果 `robot->chassis_fetch_data` 可用，优先使用 `ui_chassis_relative_angle_deg_x10 * 0.1f`。

`DrawRelativePosition()` 内部将机器人侧角度转换成 UI 角度：

```c
ui_angle = NormalizeAngle(90.0f - offset_angle);
```

这样屏幕正上方对应视觉上的前方。圆弧以 `ui_angle` 为中心，向两侧各展开 `UI_RELATIVE_ARC_HALF_ANGLE`。

如果圆弧不跨 0 度，直接绘制一段 `start_angle..end_angle`。如果圆弧跨 0 度，例如 `350..10`，则拆成两段：`start_angle..360` 和 `0..end_angle`。

## 状态缓存

`Referee_Interactive_info_t` 当前只保存相对角度 UI 需要的数据：

| 字段 | 说明 |
| --- | --- |
| `UI_Interactive_Flag.relative_flag` | 相对角度指示器刷新标志。 |
| `UI_Interactive_Flag.leg_flag` | 五连杆位姿刷新标志。 |
| `chassis_relative_angle` | 当前相对角度。 |
| `last_chassis_relative_angle` | 上一次已记录的相对角度，用于变化检测。 |
| `leg_phi1..leg_phi4` | 当前右腿五连杆模型角度，用于判断腿部图元是否需要刷新。 |
| `force_refresh_ui` | 外部请求全量重绘的标志。 |

`UIChangeCheck()` 中相对角刷新阈值为 `1.0 deg`。五连杆除了角度变化触发外，还会按 `UI_LEG_REFRESH_PERIOD_MS` 周期重发当前姿态，避免小幅变化不刷新。

## 初始化流程

`MyUIInit()` 的执行顺序：

1. 保存 `robot->referee_data` 到模块内部指针。
2. 等待 `GameRobotState.robot_id != 0`。
3. 调用 `DeterminRobotID()` 设置机器人颜色、机器人 ID 和客户端 ID。
4. 调用 `UIDelete(..., UI_Data_Del_ALL, 0)` 清空客户端旧图形。
5. 采样当前相对角度，并同步 `last_chassis_relative_angle`。
6. 使用 `UI_Graph_ADD` 绘制参考圆环、方向圆弧和五连杆。

注意：初始化会全屏删除并重建图元，不应高频调用。

## 周期刷新流程

`UITask()` 每次执行时：

1. 如果 `force_refresh_ui == 1`，调用 `MyUIInit()` 全量重绘，然后清零该标志。
2. 每 `UI_AUTO_REFRESH_PERIOD_MS` 自动触发一次同样的全量重绘，和 Ctrl 手动刷新共用 `MyUIInit()` 路径。
3. 每 150 次调用强制置位 `relative_flag`，按 `30 ms` 周期约为 `4.5 s` 一次，用于兜底重发 CHANGE 包。
4. 采样最新相对角度。
5. 每 `UI_LEG_REFRESH_PERIOD_MS` 置位一次 `leg_flag`，让五连杆持续跟随当前腿部状态。
6. 调用 `UIChangeCheck()` 判断是否超过变化阈值。
7. 调用 `MyUIRefresh()`，如果 `relative_flag` 或 `leg_flag` 已置位，则用 `UI_Graph_Change` 刷新对应图形。

## 双板 UI 数据

双板通信中与当前 UI 相关的字段位于 `Chassis_Fetch_Data_s`：

| 字段 | 方向 | 用途 |
| --- | --- | --- |
| `force_refresh_ui` | 云台板 -> 底盘板 | 请求底盘板全量重绘 UI。 |
| `ui_chassis_relative_angle_deg_x10` | 云台板 -> 底盘板 | 底盘相对角，单位为 `0.1 deg`。 |

底盘板在 `DoubleBoardComms()` 中接收到 `force_refresh_ui` 后，会通过 `getUI()` 设置 `interactive_data.force_refresh_ui = 1`，实际重绘在下一次 `UITask()` 中完成。

## 维护注意事项

- 静态首次绘制使用 `UI_Graph_ADD`，周期刷新使用 `UI_Graph_Change`。
- 裁判系统 UI 图元名称只有 3 字节，后续 CHANGE 必须使用同名图元。
- `UIGraphRefresh()` 当前只处理 `1`、`2`、`5`、`7` 个图形的批量刷新。
- `MyUIInit()` 会等待机器人 ID 有效。如果裁判系统未连接或 ID 长期为 `0`，UI 初始化任务会停在等待循环。
- 不要在 `referee_data` 为空时调用 `MyUIInit()` 或 `UITask()`。
