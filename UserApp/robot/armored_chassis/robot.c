#include "robot.h"
#include "robot_config.h"

static DJIMotorInstance *wheel_lf;
static DJIMotorInstance *wheel_rf;
static DJIMotorInstance *wheel_lr;
static DJIMotorInstance *wheel_rr;
static DJIMotorInstance *guide_left;
static DJIMotorInstance *guide_right;

static DMMotorInstance *lift_lf;
static DMMotorInstance *lift_rf;
static DMMotorInstance *lift_lr;
static DMMotorInstance *lift_rr;

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

    lift_lf = DMMotorInit(&lift_lf_config);
    DMMotorStop(lift_lf);
    lift_rf = DMMotorInit(&lift_rf_config);
    DMMotorStop(lift_rf);
    lift_lr = DMMotorInit(&lift_lr_config);
    DMMotorStop(lift_lr);
    lift_rr = DMMotorInit(&lift_rr_config);
    DMMotorStop(lift_rr);
}

void RobotTask(void)
{

};