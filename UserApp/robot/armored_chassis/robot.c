#include <math.h>
#include <stdbool.h>

#include "robot.h"
#include "robot_config.h"
#include "remote_control.h"


static DJIMotorInstance *wheel_lf;
static DJIMotorInstance *wheel_rf;
static DJIMotorInstance *wheel_lr;
static DJIMotorInstance *wheel_rr;
static DJIMotorInstance *guide_left;
static DJIMotorInstance *guide_right;
static DJIMotorInstance *side_left_3508;
static DJIMotorInstance *side_right_3508;
static DJIMotorInstance *side_left_2006;
static DJIMotorInstance *side_right_2006;

static DMMotorInstance *lift_lf;
static DMMotorInstance *lift_rf;
static DMMotorInstance *lift_lr;
static DMMotorInstance *lift_rr;

static RC_ctrl_t *remote_data;
static bool motors_enabled;
static float rotate_command;




void RobotInit(void)
{
    wheel_lf = DJIMotorInit(&wheel_lf_config);
    DJIMotorStop(wheel_lf);
    wheel_rf = DJIMotorInit(&wheel_rf_config);
    DJIMotorStop(wheel_rf);
    wheel_lr = DJIMotorInit(&wheel_lr_config);
    DJIMotorStop(wheel_lr);
    wheel_rr = DJIMotorInit(&wheel_rr_config);
    DJIMotorStop(wheel_rr);

    guide_left = DJIMotorInit(&guide_left_config);
    DJIMotorStop(guide_left);
    guide_right = DJIMotorInit(&guide_right_config);
    DJIMotorStop(guide_right);

    side_left_3508 = DJIMotorInit(&side_left_3508_config);
    DJIMotorStop(side_left_3508);
    side_right_3508 = DJIMotorInit(&side_right_3508_config);
    DJIMotorStop(side_right_3508);
    side_left_2006 = DJIMotorInit(&side_left_2006_config);
    DJIMotorStop(side_left_2006);
    side_right_2006 = DJIMotorInit(&side_right_2006_config);
    DJIMotorStop(side_right_2006);

    lift_lf = DMMotorInit(&lift_lf_config);
    DMMotorStop(lift_lf);
    lift_rf = DMMotorInit(&lift_rf_config);
    DMMotorStop(lift_rf);
    lift_lr = DMMotorInit(&lift_lr_config);
    DMMotorStop(lift_lr);
    lift_rr = DMMotorInit(&lift_rr_config);
    DMMotorStop(lift_rr);

    remote_data = RemoteControlInit(&huart5);
}


static float RemoteAxis(int16_t input)
{
    if (input >= -10 && input <= 10) {
        return 0.0f;
    }
    return (float)input / 660.0f;
}

static void SetAllMotorsEnabled(bool enabled)
{
    DJIMotorInstance *const dji_motors[] = {
        wheel_lf, wheel_rf, wheel_lr, wheel_rr,
        guide_left, guide_right,
        side_left_3508, side_right_3508,
        side_left_2006, side_right_2006,
    };
    DMMotorInstance *const lift_motors[] = {
        lift_lf, lift_rf, lift_lr, lift_rr,
    };

    for (unsigned int i = 0; i < sizeof(dji_motors) / sizeof(dji_motors[0]); ++i) {
        if (enabled) {
            DJIMotorEnable(dji_motors[i]);
        } else {
            DJIMotorStop(dji_motors[i]);
        }
    }

    for (unsigned int i = 0; i < sizeof(lift_motors) / sizeof(lift_motors[0]); ++i) {
        if (enabled) {
            DMMotorEnable(lift_motors[i]);
        } else {
            DMMotorStop(lift_motors[i]);
        }
    }
}



void RobotTask(void)
{
    const RC_ctrl_t *remote = &remote_data[TEMP];

    if (!RemoteControlIsOnline() || !switch_is_mid(remote->rc.switch_right)) {
        if (motors_enabled) {
            SetAllMotorsEnabled(false);
            motors_enabled = false;
        }

        rotate_command = 0.0f;
        return;
    }

    const float max_wheel_speed_deg_s = 55536.0f;
    const float forward = RemoteAxis(remote->rc.rocker_l1);
    const float left = RemoteAxis(remote->rc.rocker_l_);
    const float dial = RemoteAxis(remote->rc.dial);
    const float rotate_target = -0.50f * dial * fabsf(dial);

    rotate_command += 0.10f * (rotate_target - rotate_command);
    const float rotate = rotate_command;

    const float demand = fabsf(forward) + fabsf(left) + fabsf(rotate);
    const float speed_scale = max_wheel_speed_deg_s / fmaxf(1.0f, demand);

    DJIMotorSetPIDRef(wheel_lf, (forward - left - rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_rf, (forward + left + rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_lr, (forward + left - rotate) * speed_scale);
    DJIMotorSetPIDRef(wheel_rr, (forward - left + rotate) * speed_scale);


    if (!motors_enabled) {
        SetAllMotorsEnabled(true);
        motors_enabled = true;
    }
}
