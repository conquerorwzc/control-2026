/**
 * @file can_comm_task.c
 * @author yuanluochen
 * @brief can设备通信任务，利用队列实现can数据顺序发送
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "can_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "robot.h"
#include "CAN_receive.h"

// static CAN_TxHeaderTypeDef capid_tx_message;
// static uint8_t capid_can_send_data[8];
//
// static CAN_TxHeaderTypeDef board_can_tx_message;
// static uint8_t board_can_send_data[8];

/**
 * @brief can通信线程初始化, 主要任务为开辟线程队列
 *
 * @param can_comm_init can通信线程初始化结构体
 */
static void can_comm_task_init(can_comm_task_t *can_comm_init);

/**
 * @brief can通信任务发送函数，通信数据队列发送
 *
 * @param can_comm_transmit can通信任务控制结构体
 */
static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit);

/**
 * @brief can通信队列添加函数
 *
 * @param add_comm_queue can通信任务函数
 * @param comm_data can通信数据
 */
static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data);

/**
 * @brief can通信队列数据更新
 *
 * @param feedback_update can通信队列结构体
 */
static void can_comm_feedback_update(can_comm_task_t *feedback_update);


//双板can通信数据
static can_comm_data_t board_can_comm_data = {
    .can_handle = &hcan1, // 初始化双板通信设备can
    .can_comm_target = CAN_COMM_CHASSIS,
};


//裁判系统通信数据
//static can_comm_data_t referee_can_comm_data = {
//    .can_handle = &SHOOT_FLAGS_CAN, // 初始化裁判系统通信设备can
//    .can_comm_target = CAN_COMM_SHOOT_FLAGS,
//};
//发送云台pitch轴的相对角和绝对角


bool init_finish = false;

//实例化can通信线程结构体,全局变量，保证数据一直存在
can_comm_task_t can_comm = { 0 };

void RobotInit() {
  // 空实现
}
void RobotTask()
{
    //要在云台和底盘任务开始之前完成该任务的初始化
    vTaskDelay(CAN_COMM_TASK_INIT_TIME);
    // 初始化CAN接收
    CANReceive_Init();
    //can通信任务初始化
    can_comm_task_init(&can_comm);
    init_finish = true;

    if (DEVICE_ROLE_TX)
    {
      // 测试数据,实际应用中这些数据应该来自其他模块
      uint8_t ui_flag = 1;
      uint8_t fric_flag = 0;
      int16_t chassis_vx = 100;
      int16_t chassis_vy = 50;
      int16_t pitch_abs = 3000;
      uint8_t chassis_behaviour = 1;
      uint8_t cap_flag = 0;

      // 发送计数器,用于测试
      static uint32_t send_count = 0;

      while(1)
      {
          // 每隔100ms发送一次数据
          if(send_count % 100 == 0)
          {
            // 更新测试数据
            chassis_vx = (chassis_vx + 10) % 1000;
            chassis_vy = (chassis_vy + 5) % 500;
            pitch_abs = (pitch_abs + 50) % 6000;
            ui_flag = !ui_flag;

            // 应用层数据 -> 数据打包 -> 加入发送队列
            can_comm_board(ui_flag, fric_flag, chassis_vx, chassis_vy,
                          pitch_abs, chassis_behaviour, cap_flag);
          }
          //can通信参数更新
          can_comm_feedback_update(&can_comm);
          //can通信数据发送
          can_comm_task_transmit(&can_comm);
          //can数据数据发送
          send_count++;
          vTaskDelay(CAN_COMM_TASK_TIME);
      }
    }
    else {
      // 接收板逻辑
      while(1)
      {
        // 检查是否收到新数据
        if (data_received) {
          // 数据已接收，可以在这里处理
          data_received = 0;
        }
        vTaskDelay(50); // 接收板可以延时较长
      }
    }
}

static void can_comm_feedback_update(can_comm_task_t *feedback_update)
{
    feedback_update->can_comm_queue->size = can_comm_queue_size(feedback_update->can_comm_queue);
}

static void can_comm_task_init(can_comm_task_t *can_comm_init)
{
    if (can_comm_init == NULL)
        return;
    //创建并初始化can通信队列
    can_comm_init->can_comm_queue = can_comm_queue_init();
}

static void can_comm_task_transmit(can_comm_task_t * can_comm_transmit)
{
    if (can_comm_transmit == NULL)
        return;
    //队列非空发送
    if (!can_comm_queue_is_empty(can_comm_transmit->can_comm_queue))
    {
        can_comm_data_t *data = can_comm_queue_pop(can_comm_transmit->can_comm_queue);
        if(data)
        {
            can_transmit(data);
        }
    }
}


static void add_can_comm_queue(can_comm_task_t *add_comm_queue, can_comm_data_t *comm_data)
{
    if (add_comm_queue == NULL || comm_data == NULL)
        return;
    //添加数据到发送队列中
    can_comm_queue_push(add_comm_queue->can_comm_queue, comm_data);
}


int16_t pitch;
void can_comm_board(uint8_t ui_flag, uint8_t fric_flag, int16_t chassis_vx, int16_t chassis_vy,int16_t pitch_abs, uint8_t chassis_behaviour, uint8_t cap_flag)
{
    uint8_t vx_tmp =0,vy_tmp=0;

    vx_tmp = chassis_vx * 127/660+127;
    vy_tmp = chassis_vy * 127/660+127;
    pitch = pitch_abs;

    //配置can发送数据
    board_can_comm_data.transmit_message.StdId = CAN_GIMBAL_CONTROL_CHASSIS_ID;
    board_can_comm_data.transmit_message.IDE = CAN_ID_STD;
    board_can_comm_data.transmit_message.RTR = CAN_RTR_DATA;
    board_can_comm_data.transmit_message.DLC = 0x08;
    board_can_comm_data.data[0] = ui_flag;
    board_can_comm_data.data[1] = fric_flag;
    board_can_comm_data.data[2] = vx_tmp;
    board_can_comm_data.data[3] = vy_tmp;
    board_can_comm_data.data[4] = (pitch_abs >> 8);
    board_can_comm_data.data[5] = pitch_abs;
    board_can_comm_data.data[6] = chassis_behaviour;
    board_can_comm_data.data[7] = cap_flag;
    //添加数据到通信队列
    add_can_comm_queue(&can_comm, &board_can_comm_data);
}

bool can_comm_task_init_finish(void)
{
    return init_finish;
}

