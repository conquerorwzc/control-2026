/**
 ******************************************************************************
 * @file    hip_odometry.c
 * @brief   轮腿髋点纵向里程计的纯数学计算
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "hip_odometry.h"

#include <math.h>
#include <stddef.h>

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 由单条腿的轮端和虚拟腿状态计算髋点纵向位置、速度。
 *
 * 坐标关系为 x_O = x_P + l * sin(theta)。轮端滚动位移给出 x_P；
 * 对位置求导后得到轮速、腿长变化和腿摆动三项速度。
 *
 * @param input 单条腿同一帧的轮端和虚拟腿输入。
 * @param output 返回未减零点的位置和髋点速度。
 * @return 输入和结果均为有限数且轮半径有效时返回 1，否则返回 0。
 */
uint8_t WheelLeggedHipOdometryCalculate(const WheelLeggedHipOdometryLegInput_t *input,
                                        WheelLeggedHipOdometryLegOutput_t *output)
{
    if (input == NULL || output == NULL || !isfinite(input->wheel_radius) || input->wheel_radius <= 0.0f ||
        !isfinite(input->wheel_angle) || !isfinite(input->wheel_speed) || !isfinite(input->leg_length) ||
        input->leg_length < 0.0f || !isfinite(input->leg_length_dot) || !isfinite(input->leg_theta) ||
        !isfinite(input->leg_theta_dot))
    {
        return 0u;
    }

    output->raw_position = input->wheel_radius * input->wheel_angle + input->leg_length * sinf(input->leg_theta);
    output->velocity = input->wheel_radius * input->wheel_speed + input->leg_length_dot * sinf(input->leg_theta) +
                       input->leg_length * cosf(input->leg_theta) * input->leg_theta_dot;
    return isfinite(output->raw_position) && isfinite(output->velocity);
}
