//
// Created by suntuwu on 2025/10/15.
//

#include "MotorDriver.hpp"

namespace motor_control
{
    uint8_t testid=0xFF;


    /*-----------------variables----------------*/
    uint8_t CAN_rxBuf[8] = {0};

    uint8_t CAN_3508_1_4_txBuf[8] = {0};
    uint8_t CAN_3508_5_8_txBuf[8] = {0};
    uint8_t CAN_6020_1_4_txBuf[8] = {0};
    uint8_t CAN_6020_5_8_txBuf[8] = {0};

    Motor *globalMotorList[MAX_MOTOR_NUM] = {nullptr};

    /*-----------------PIDController-----------------*/
    PIDController::PIDController() {}

    float PIDController::computePID(float error, float dt)
    {
        // integral_ += error * dt;
        // float derivative = (error - prev_error_) / dt;
        prev_error_ = error;

        return kp_ * error;
        // return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void PIDController::setPID(float p, float i, float d)
    {
        kp_ = p;
        ki_ = i;
        kd_ = d;
    }

    void PIDController::reset()
    {
        integral_ = 0.0f;
        prev_error_ = 0.0f;
    }

    void PIDController::setOutputLimit(float limit)
    {
        output_limit_ = limit;
    }

    float PIDController::getOutputLimit() const
    {
        return output_limit_;
    }

    /*-----------------Motor基类-----------------*/
    Motor::Motor(uint8_t id, MotorType motor_type)
        : id_(id), motor_type_(motor_type), mode_(OPEN_LOOP)
    {

        // 把自己添加到globalMotorList中
        global_id_ = findMotorID(id_, motor_type_);
        if (global_id_ == 0xFF) // 没找到
        {
            for (uint8_t i = 0; i < MAX_MOTOR_NUM; ++i)
            {
                if (globalMotorList[i] == nullptr)
                {
                    globalMotorList[i] = this;
                    global_id_ = i;
                    break;
                }
            }
        }
    }

    Motor::~Motor() {}

    // switch control mode
    void Motor::setOpenLoop() { mode_ = OPEN_LOOP; }
    void Motor::setSpeedControl() { mode_ = SPEED_CONTROL; }
    void Motor::setPositionControl() { mode_ = POSITION_CONTROL; }

    // set targets
    void Motor::setAngle(float angle_deg) { target_angle_ = angle_deg; }
    void Motor::setSpeed(float speed_rpm) { target_speed_ = speed_rpm; }
    void Motor::setOpenLoopCurrent(float current) { open_loop_current_ = current; }

    void Motor::setPositionPID(float p, float i, float d)
    {
        pid_position_.setPID(p, i, d);
    }

    void Motor::setSpeedPID(float p, float i, float d)
    {
        pid_speed_.setPID(p, i, d);
    }

    void Motor::setMaxOutputSpeed(float max_rpm)
    {
        pid_position_.setOutputLimit(max_rpm);
    }

    void Motor::switchToDefaultPID()
    {
        setPositionPID(1.0f, 0.0f, 0.0f);
        setSpeedPID(1.0f, 0.0f, 0.0f);
    }

    void Motor::resetPID()
    {
        pid_position_.reset();
        pid_speed_.reset();
    }

    State Motor::getState() const { return state_; } // getter const

    void Motor::update() {}

    float Motor::computeControl()
    {
        switch (mode_)
        {
        case OPEN_LOOP:
            return open_loop_current_;

        case SPEED_CONTROL:
        {
            float speed_error = target_speed_ - state_.speed;
            float current = pid_speed_.computePID(speed_error);
            
            // 根据电机类型设置不同的电流限制
            float max_current = (motor_type_ == MOTOR_TYPE_3508) ? 20.0f : 3.0f;
            
            if (current > max_current)
                current = max_current;
            else if (current < -max_current)
                current = -max_current;
            return current;
        }

        case POSITION_CONTROL:
        {
            float position_error = target_angle_ - state_.angle;

            state_.error= position_error; // test,更新当前位置误差供外部查询

            float target_speed = pid_position_.computePID(position_error);

            float max_speed = pid_position_.getOutputLimit();
            if (target_speed > max_speed)
                target_speed = max_speed;
            if (target_speed < -max_speed)
                target_speed = -max_speed;

            float speed_error = target_speed - state_.speed;

            target_speed_ = target_speed; // test,更新目标速度供外部查询

            float current = pid_speed_.computePID(speed_error);

            // 根据电机类型设置不同的电流限制
            float max_current = (motor_type_ == MOTOR_TYPE_3508) ? 20.0f : 3.0f;
            if (current > max_current)
                current = max_current;
            else if (current < -max_current)
                current = -max_current;
            return current;
        }

        default:
            return 0.0f;
        }
    }

    /*-----------------CANMotorM3508-----------------*/
    CANMotorM3508::CANMotorM3508(uint8_t CANid)
        : Motor(CANid, MOTOR_TYPE_3508) {}

    void CANMotorM3508::update()
    {
        target_current_ = computeControl();

        int16_t send_current_ = (int16_t)(target_current_ * 16384.0f / 20.0f);

        if (id_ <= 4)
        {
            CAN_3508_1_4_txBuf[(id_ - 1) * 2] = (send_current_ >> 8) & 0xFF;
            CAN_3508_1_4_txBuf[(id_ - 1) * 2 + 1] = send_current_ & 0xFF;
            HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &CanMotorConfig::M3508txHeader1_4, CAN_3508_1_4_txBuf);
        }
        else
        {
            CAN_3508_5_8_txBuf[(id_ - 5) * 2] = (send_current_ >> 8) & 0xFF;
            CAN_3508_5_8_txBuf[(id_ - 5) * 2 + 1] = send_current_ & 0xFF;
            HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &CanMotorConfig::M3508txHeader5_8, CAN_3508_5_8_txBuf);
        }
    }

    /*-----------------CANMotorGM6020-----------------*/
    CANMotorGM6020::CANMotorGM6020(uint8_t id)
        : Motor(id, MOTOR_TYPE_6020) {}

    void CANMotorGM6020::update()
    {
        target_current_ = computeControl();
    
        // int16_t send_current_ = (int16_t)(target_current_ * 16384.0f / 3.0f);
        int16_t send_current_ = (int16_t)(target_current_ * 25000.0f / 3.0f); //电压模式!测试用,需要用上位机改6020

    
        if (id_ <= 4)
        {
            CAN_6020_1_4_txBuf[(id_ - 1) * 2] = (send_current_ >> 8) & 0xFF;
            CAN_6020_1_4_txBuf[(id_ - 1) * 2 + 1] = send_current_ & 0xFF;
            HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &CanMotorConfig::GM6020txHeader1_4, CAN_6020_1_4_txBuf);
        }
        else
        {
            CAN_6020_5_8_txBuf[(id_ - 5) * 2] = (send_current_ >> 8) & 0xFF; //高8位
            CAN_6020_5_8_txBuf[(id_ - 5) * 2 + 1] = send_current_ & 0xFF;
            HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &CanMotorConfig::GM6020txHeader5_8, CAN_6020_5_8_txBuf);
        }
    }

    /*-----------------工具-----------------*/
    uint8_t findMotorID(uint8_t id, uint8_t motor_type)
    {
        for (uint8_t i = 0; i < MAX_MOTOR_NUM && globalMotorList[i] != nullptr; ++i)
        {
            if (globalMotorList[i]->motor_type_ == motor_type && globalMotorList[i]->id_ == id)
            {
                return i;
            }
        }
        return 0xFF;
    }

    /*-----------------callback解码 and init-----------------*/
    extern "C"
    {
        void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
        {
            if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
            {
                FDCAN_RxHeaderTypeDef rxHeader;
                HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, CAN_rxBuf);

                uint16_t id;
                uint16_t angle;
                int16_t speed;
                int16_t current;
                uint8_t temperature;
                if (rxHeader.Identifier >= 0x209 && rxHeader.Identifier <= 0x20B)  //待修复-设置断点!!!!!!!!!!!!!!!!!!!
                {
                    // 6020电机
                    id = rxHeader.Identifier - 0x204;
                    angle = (CAN_rxBuf[0] << 8) | CAN_rxBuf[1];
                    speed = (CAN_rxBuf[2] << 8) | CAN_rxBuf[3];
                    current = (CAN_rxBuf[4] << 8) | CAN_rxBuf[5];
                    temperature = CAN_rxBuf[6];

                    uint8_t motorID = findMotorID(id, MOTOR_TYPE_6020);
                    
                    testid=motorID; //test
                    
                    if (motorID != 0xFF)
                    {
                        globalMotorList[motorID]->state_.angle = angle * 360.0f / 8192.0f;
                        globalMotorList[motorID]->state_.speed = speed;
                        // globalMotorList[motorID]->state_.current = current * 20.0f / 16384.0f;
                        globalMotorList[motorID]->state_.current = current * 3.0f / 25000.0f;
                        globalMotorList[motorID]->state_.temperature = temperature;
                    }
                }

                if (rxHeader.Identifier <= 0x208)
                {
                    // 3508电机
                    id = rxHeader.Identifier - 0x200;
                    angle = (CAN_rxBuf[0] << 8) | CAN_rxBuf[1];
                    speed = (CAN_rxBuf[2] << 8) | CAN_rxBuf[3];
                    current = (CAN_rxBuf[4] << 8) | CAN_rxBuf[5];
                    temperature = CAN_rxBuf[6];

                    uint8_t motorID = findMotorID(id, MOTOR_TYPE_3508);
                    if (motorID != 0xFF)
                    {
                        globalMotorList[motorID]->state_.angle = angle * 360.0f / 8192.0f;
                        globalMotorList[motorID]->state_.speed = speed/19.0f;
                        globalMotorList[motorID]->state_.current = current * 20.0f / 16384.0f;
                        globalMotorList[motorID]->state_.temperature = temperature;
                    }
                }
            }
        }

        void CANMotorInit(FDCAN_HandleTypeDef *hfdcan)
    {
        HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, CAN_RxFifo0Callback);
        HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        HAL_FDCAN_ConfigFilter(hfdcan, &CanMotorConfig::M3508GM6020rxFilter);//0x200 ~ 0x20F  (16个ID)
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0xFFFFFFFF);
        HAL_FDCAN_Start(hfdcan);
    }

    }


    void updateAllCAN(Motor *motorList[], size_t size, FDCAN_HandleTypeDef *hfdcan)
    {
        for (size_t i = 0; i < size && motorList[i] != nullptr; ++i)
        {
            if (motorList[i]->motor_type_ == MOTOR_TYPE_3508 || motorList[i]->motor_type_ == MOTOR_TYPE_6020)
            {
                motorList[i]->update();
            }
        }
    }

    /*-----------------CAN配置-----------------*/
    namespace CanMotorConfig
    {
        FDCAN_TxHeaderTypeDef M3508txHeader1_4 = {
            .Identifier = 0x200,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0};

        FDCAN_TxHeaderTypeDef M3508txHeader5_8 = {
            .Identifier = 0x1FF,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0};

        FDCAN_TxHeaderTypeDef GM6020txHeader1_4 = {
            .Identifier = 0x1FE,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0};

        FDCAN_TxHeaderTypeDef GM6020txHeader5_8 = {
            .Identifier = 0x2FF,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0};

        FDCAN_FilterTypeDef M3508GM6020rxFilter = {
            .IdType = FDCAN_STANDARD_ID,
            .FilterIndex = 0,
            .FilterType = FDCAN_FILTER_MASK,
            .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
            .FilterID1 = 0x200,//10 0000 0000
            .FilterID2 = 0x7F0};//111 1111 0000 //0x200 ~ 0x20F  (16个ID)
    } // namespace CanMotorConfig

} // namespace motor_control
