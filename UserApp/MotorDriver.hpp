#ifndef MOTOR_SIMPLE_HPP
#define MOTOR_SIMPLE_HPP

#include <cstdint>
#include "fdcan.h"

#define MAX_MOTOR_NUM 254

extern "C" {
namespace motor_control
{
    class Motor;

    extern Motor *globalMotorList[MAX_MOTOR_NUM];

    struct State
    {
        float angle = 0.0f;
        float speed = 0.0f;
        float current = 0.0f;
        float temperature = 0.0f;
        float error = 0;
    };

    enum ControlMode
    {
        OPEN_LOOP,
        SPEED_CONTROL,
        POSITION_CONTROL
    };

    enum MotorType
    {
        MOTOR_TYPE_3508,
        MOTOR_TYPE_6020
    };

    class PIDController
    {
    public:
        PIDController();
        float computePID(float error, float dt = 0.001f);
        void setPID(float p, float i, float d);
        void reset();
        void setOutputLimit(float limit);
        float getOutputLimit() const;

    private:
        float kp_ = 1.0f;
        float ki_ = 0.0f;
        float kd_ = 0.0f;
        float integral_ = 0.0f;
        float prev_error_ = 0.0f;
        float output_limit_ = 10000.0f;
    };

    class Motor
    {
    public:
        Motor(uint8_t id, MotorType motor_type);
        virtual ~Motor();

        void setOpenLoop();
        void setSpeedControl();
        void setPositionControl();

        void setAngle(float angle_deg);
        void setSpeed(float speed_rpm);
        void setOpenLoopCurrent(float current);

        void setPositionPID(float p, float i, float d);
        void setSpeedPID(float p, float i, float d);
        void setMaxOutputSpeed(float max_rpm);
        void switchToDefaultPID();
        void resetPID();

        State getState() const;

        virtual void update();
        float computeControl();

        uint8_t id_;
        MotorType motor_type_;
        uint8_t global_id_;
        State state_;

        float target_angle_ = 0.0f;
        float target_speed_ = 0.0f;
        float target_current_ = 0.0f;

    protected:
        ControlMode mode_;
        PIDController pid_position_;
        PIDController pid_speed_;
        float open_loop_current_ = 0.0f;
    };

    class CANMotorM3508 : public Motor
    {
    public:
        CANMotorM3508(uint8_t id);
        void update() override;

    float real_current = 0.0f;


        FDCAN_HandleTypeDef *hfdcan_ = &hfdcan1;
    };

    class CANMotorGM6020 : public Motor
    {
    public:
        CANMotorGM6020(uint8_t id);
        void update() override;

    float real_current = 0.0f;

        FDCAN_HandleTypeDef *hfdcan_ = &hfdcan1;
    };

    uint8_t findMotorID(uint8_t id, uint8_t motor_type);
    void CANMotorInit(FDCAN_HandleTypeDef *hfdcan);
    void updateAllCAN(Motor *motorList[], size_t size, FDCAN_HandleTypeDef *hfdcan);

    namespace CanMotorConfig
    {
        extern FDCAN_TxHeaderTypeDef M3508txHeader1_4;
        extern FDCAN_TxHeaderTypeDef M3508txHeader5_8;
        extern FDCAN_TxHeaderTypeDef GM6020txHeader1_4;
        extern FDCAN_TxHeaderTypeDef GM6020txHeader5_8;
        extern FDCAN_FilterTypeDef M3508GM6020rxFilter;
    }
} // namespace motor_control
}
#endif // MOTOR_SIMPLE_HPP
