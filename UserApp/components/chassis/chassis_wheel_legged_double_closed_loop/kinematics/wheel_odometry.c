/**
 ******************************************************************************
 * @file    wheel_odometry.c
 * @brief   轮腿底盘纯轮式纵向里程计的纯数学实现
 ******************************************************************************
 */
/* Private includes ----------------------------------------------------------*/
#include "wheel_odometry.h"

#include <math.h>
#include <stddef.h>

/* Private define ------------------------------------------------------------*/

/* Intermediate variables calculated by private functions -------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
 * @brief 根据左右轮端角度和角速度计算平均纯轮式纵向里程计。
 *
 * @param input 左右轮端同一帧输入。
 * @param output 返回未减零点的位置和速度。
 * @return 输入和结果均有限且轮半径有效时返回 1，否则返回 0。
 */
uint8_t WheelLeggedWheelOdometryCalculate(const WheelLeggedWheelOdometryInput_t *input,
                                          WheelLeggedWheelOdometryOutput_t *output)
{
    if (input == NULL || output == NULL || !isfinite(input->left_wheel_radius) ||
        !isfinite(input->right_wheel_radius) || input->left_wheel_radius <= 0.0f ||
        input->right_wheel_radius <= 0.0f || !isfinite(input->left_wheel_angle) ||
        !isfinite(input->right_wheel_angle) || !isfinite(input->left_wheel_speed) ||
        !isfinite(input->right_wheel_speed))
    {
        return 0u;
    }

    output->raw_position = 0.5f * (input->left_wheel_radius * input->left_wheel_angle +
                                   input->right_wheel_radius * input->right_wheel_angle);
    output->velocity = 0.5f * (input->left_wheel_radius * input->left_wheel_speed +
                               input->right_wheel_radius * input->right_wheel_speed);
    return isfinite(output->raw_position) && isfinite(output->velocity);
}
