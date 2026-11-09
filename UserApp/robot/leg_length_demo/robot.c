#include "robot.h"
#include "FreeRTOS.h"
#include "message_center.h"
#include "remote_control.h"
#include "robot_config.h"
#include "task.h"

typedef enum
{
    LENGTH_STOPPED,
    LENGTH_ENABLING,
    LENGTH_ACTIVE,
    LENGTH_FAULT
} LengthMode;
typedef struct
{
    LengthMode mode;
    bool disable_seen;
    float requested_m, target_m[2];
    LegLengthState leg[2];
    uint32_t enable_start_ms;
    uint32_t control_cycles;
} LengthDemoState;

// 调试入口；PID 可通过 length_demo.leg[i].pid 查看/调参。
LengthDemoState length_demo;
static DMMotorInstance *joint[4];
static RC_ctrl_t *rc;
static Publisher_t *status_pub;

static void StopJoints(void)
{
    for (unsigned i = 0; i < 4; ++i)
        DMMotorSetHardwareEnabled(joint[i], 0u);
    for (unsigned side = 0; side < 2; ++side)
    {
        // 保留在线调整的增益，仅清控制器历史。
        PIDInstance *pid = &length_demo.leg[side].pid;
        pid->Iout = pid->ITerm = pid->Last_ITerm = pid->Last_Err = 0.0f;
        pid->Output = pid->Last_Output = pid->Dout = pid->Last_Dout = 0.0f;
        length_demo.leg[side].force_n = 0.0f;
        length_demo.leg[side].torque_nm[0] = length_demo.leg[side].torque_nm[1] = 0.0f;
    }
}

void RobotInit(void)
{
    for (unsigned i = 0; i < 4; ++i)
        joint[i] = DMMotorInitSynchronous(&joint_config[i]);
    for (unsigned i = 0; i < 2; ++i)
        PIDInit(&length_demo.leg[i].pid, &length_config.pid);
    rc = RemoteControlInit(&huart5);
    status_pub = PubRegister("leg_length_status", 5u * sizeof(float));
    StopJoints();
}

void RobotTask(void)
{
    DM_Motor_Measure_s measure[4];
    uint32_t updated_ms[4];
    bool received[4];
    RC_ctrl_t remote;
    bool online;
    taskENTER_CRITICAL();
    remote = rc[TEMP];
    online = RemoteControlIsOnline() != 0u;
    for (unsigned i = 0; i < 4; ++i)
    {
        measure[i] = joint[i]->measure;
        updated_ms[i] = joint[i]->last_feedback_ms;
        received[i] = joint[i]->feedback_received != 0u;
    }
    taskEXIT_CRITICAL();
    const uint32_t now_ms = HAL_GetTick();
    const bool requested = LegLengthRequested(online, remote.rc.switch_left, remote.rc.switch_right);
    length_demo.requested_m = LegLengthTarget((float)remote.rc.dial, kRemoteFullScale, &length_config);
    bool fresh = true, normal = true, error = false;
    for (unsigned i = 0; i < 4; ++i)
    {
        fresh &= received[i] && (uint32_t)(now_ms - updated_ms[i]) <= LEG_LENGTH_FEEDBACK_TIMEOUT_MS;
        normal &= measure[i].state == 1u;
        error |= measure[i].state > 1u;
    }
    // 失能时也计算腿长，便于先核对反馈和零点；有效性只在进入输出链时使用。
    bool geometry_valid = fresh;
    for (unsigned side = 0; side < 2 && geometry_valid; ++side)
    {
        const unsigned n = side * 2u;
        const float q[2] = {measure[n].total_angle - kJointZeroRad[n],
                            measure[n + 1].total_angle - kJointZeroRad[n + 1]};
        const float qd[2] = {measure[n].velocity, measure[n + 1].velocity};
        geometry_valid = LegLengthGeometry(&length_config, q, qd, &length_demo.leg[side]);
    }
    if (!requested)
    {
        StopJoints();
        length_demo.mode = LENGTH_STOPPED;
        // 离线不能当成人为重新使能；必须在线拨下后再切回。
        length_demo.disable_seen = online && remote.rc.switch_left >= 1u && remote.rc.switch_left <= 3u &&
                                   remote.rc.switch_right >= 1u && remote.rc.switch_right <= 3u &&
                                   (remote.rc.switch_left == 2u || remote.rc.switch_right == 2u);
    }
    else if (length_demo.mode == LENGTH_STOPPED && length_demo.disable_seen && LEG_LENGTH_CALIBRATED)
    {
        length_demo.disable_seen = false;
        if (!geometry_valid || error)
            length_demo.mode = LENGTH_FAULT;
        else
        {
            bool sent = true;
            for (unsigned i = 0; i < 4; ++i)
                sent &= DMMotorSetHardwareEnabled(joint[i], 1u) != 0u;
            length_demo.enable_start_ms = now_ms;
            length_demo.mode = sent ? LENGTH_ENABLING : LENGTH_FAULT;
        }
    }
    if (length_demo.mode == LENGTH_ENABLING)
    {
        if (!fresh || error || (uint32_t)(now_ms - length_demo.enable_start_ms) > LEG_LENGTH_ENABLE_TIMEOUT_MS)
            length_demo.mode = LENGTH_FAULT;
        else if (normal)
        {
            for (unsigned side = 0; side < 2; ++side)
            {
                length_demo.target_m[side] =
                    fminf(fmaxf(length_demo.leg[side].length_m, length_config.minimum_m), length_config.maximum_m);
                DWT_GetDeltaT(&length_demo.leg[side].pid.DWT_CNT);
            }
            length_demo.mode = geometry_valid ? LENGTH_ACTIVE : LENGTH_FAULT;
        }
    }
    if (length_demo.mode == LENGTH_ACTIVE)
    {
        bool valid = geometry_valid && normal;
        for (unsigned side = 0; side < 2 && valid; ++side)
        {
            const float step = kHandoverSpeedMps / (float)LEG_LENGTH_CONTROL_HZ;
            length_demo.target_m[side] +=
                fminf(fmaxf(length_demo.requested_m - length_demo.target_m[side], -step), step);
            valid = LegLengthStep(&length_config, length_demo.target_m[side], &length_demo.leg[side]);
        }
        if (!valid)
            length_demo.mode = LENGTH_FAULT;
    }
    if (length_demo.mode == LENGTH_FAULT)
        StopJoints();
    bool sent = true;
    for (unsigned i = 0; i < 4; ++i)
    {
        const float torque = length_demo.mode == LENGTH_ACTIVE ? length_demo.leg[i / 2u].torque_nm[i % 2u] : 0.0f;
        DMMotorSetRef(joint[i], torque);
        sent &= DMMotorControlStep(joint[i]) != 0u;
    }
    if (!sent)
    {
        length_demo.mode = LENGTH_FAULT;
        StopJoints();
    }
    if (++length_demo.control_cycles % 5u == 0u)
    {
        float status[5] = {(float)length_demo.mode, length_demo.requested_m, length_demo.leg[0].length_m,
                           length_demo.leg[1].length_m, (float)LEG_LENGTH_CALIBRATED};
        PubPushMessage(status_pub, status);
    }
}
