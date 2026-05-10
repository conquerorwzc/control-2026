#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 */

#define MEC_ARM_GRAVITY_J2_TERM_NUM 17
static const float kMecArmGravityJ2Coeff[17] = {
    -2.072771115e-02f,
    -1.213411634e-02f,
    4.029226944e-02f,
    3.198269182e-02f,
    1.150616331e-02f,
    2.072094053e-02f,
    -2.995612878e-02f,
    1.154304500e-03f,
    1.015862523e-03f,
    4.959271441e-03f,
    -1.234565273e-02f,
    2.623247603e-04f,
    -1.815642742e-02f,
    -1.768130885e-02f,
    -1.949272888e-02f,
    -1.315665236e-02f,
    8.183661943e-03f,
};

/* Joint2 term order:
 * 0: 1
 * 1: sin(q2)
 * 2: cos(q2)
 * 3: sin(q3)
 * 4: cos(q3)
 * 5: sin(q4)
 * 6: cos(q4)
 * 7: sin(q3+q4)
 * 8: cos(q3+q4)
 * 9: sin(q3+q4+q5)
 * 10: cos(q3+q4+q5)
 * 11: sin(q2+q3)
 * 12: cos(q2+q3)
 * 13: sin(q2+q3+q4)
 * 14: cos(q2+q3+q4)
 * 15: sin(q2+q3+q4+q5)
 * 16: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J2(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ2Coeff[0] * (1.0f) +
        kMecArmGravityJ2Coeff[1] * (arm_sin_f32(q2)) +
        kMecArmGravityJ2Coeff[2] * (arm_cos_f32(q2)) +
        kMecArmGravityJ2Coeff[3] * (arm_sin_f32(q3)) +
        kMecArmGravityJ2Coeff[4] * (arm_cos_f32(q3)) +
        kMecArmGravityJ2Coeff[5] * (arm_sin_f32(q4)) +
        kMecArmGravityJ2Coeff[6] * (arm_cos_f32(q4)) +
        kMecArmGravityJ2Coeff[7] * (arm_sin_f32(q3 + q4)) +
        kMecArmGravityJ2Coeff[8] * (arm_cos_f32(q3 + q4)) +
        kMecArmGravityJ2Coeff[9] * (arm_sin_f32(q3 + q4 + q5)) +
        kMecArmGravityJ2Coeff[10] * (arm_cos_f32(q3 + q4 + q5)) +
        kMecArmGravityJ2Coeff[11] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[12] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[13] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[14] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[15] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ2Coeff[16] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J3_TERM_NUM 13
static const float kMecArmGravityJ3Coeff[13] = {
    4.931759521e-04f,
    2.084578622e-03f,
    5.817215790e-02f,
    1.370543326e-01f,
    -1.141645040e-01f,
    1.047905913e-02f,
    -8.264182848e-02f,
    1.020712941e-01f,
    -4.007339043e-01f,
    -7.220939634e-02f,
    -2.026568489e-01f,
    -4.799591734e-02f,
    -2.768531535e-02f,
};

/* Joint3 term order:
 * 0: 1
 * 1: sin(q3)
 * 2: cos(q3)
 * 3: sin(q3+q4)
 * 4: cos(q3+q4)
 * 5: sin(q3+q4+q5)
 * 6: cos(q3+q4+q5)
 * 7: sin(q2+q3)
 * 8: cos(q2+q3)
 * 9: sin(q2+q3+q4)
 * 10: cos(q2+q3+q4)
 * 11: sin(q2+q3+q4+q5)
 * 12: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J3(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ3Coeff[0] * (1.0f) +
        kMecArmGravityJ3Coeff[1] * (arm_sin_f32(q3)) +
        kMecArmGravityJ3Coeff[2] * (arm_cos_f32(q3)) +
        kMecArmGravityJ3Coeff[3] * (arm_sin_f32(q3 + q4)) +
        kMecArmGravityJ3Coeff[4] * (arm_cos_f32(q3 + q4)) +
        kMecArmGravityJ3Coeff[5] * (arm_sin_f32(q3 + q4 + q5)) +
        kMecArmGravityJ3Coeff[6] * (arm_cos_f32(q3 + q4 + q5)) +
        kMecArmGravityJ3Coeff[7] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[8] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[9] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[10] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[11] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ3Coeff[12] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J4_TERM_NUM 13
static const float kMecArmGravityJ4Coeff[13] = {
    2.110995255e-03f,
    -2.924105453e-03f,
    2.540651252e-03f,
    -2.514324677e-02f,
    -3.782003248e-04f,
    2.991288341e-02f,
    -5.915848537e-03f,
    1.220963440e-02f,
    -2.544442916e-02f,
    5.797620693e-03f,
    -1.039553349e-01f,
    7.624348632e-03f,
    7.396837166e-03f,
};

/* Joint4 term order:
 * 0: 1
 * 1: sin(q4)
 * 2: cos(q4)
 * 3: sin(q4+q5)
 * 4: cos(q4+q5)
 * 5: sin(q3+q4)
 * 6: cos(q3+q4)
 * 7: sin(q3+q4+q5)
 * 8: cos(q3+q4+q5)
 * 9: sin(q2+q3+q4)
 * 10: cos(q2+q3+q4)
 * 11: sin(q2+q3+q4+q5)
 * 12: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J4(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ4Coeff[0] * (1.0f) +
        kMecArmGravityJ4Coeff[1] * (arm_sin_f32(q4)) +
        kMecArmGravityJ4Coeff[2] * (arm_cos_f32(q4)) +
        kMecArmGravityJ4Coeff[3] * (arm_sin_f32(q4 + q5)) +
        kMecArmGravityJ4Coeff[4] * (arm_cos_f32(q4 + q5)) +
        kMecArmGravityJ4Coeff[5] * (arm_sin_f32(q3 + q4)) +
        kMecArmGravityJ4Coeff[6] * (arm_cos_f32(q3 + q4)) +
        kMecArmGravityJ4Coeff[7] * (arm_sin_f32(q3 + q4 + q5)) +
        kMecArmGravityJ4Coeff[8] * (arm_cos_f32(q3 + q4 + q5)) +
        kMecArmGravityJ4Coeff[9] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ4Coeff[10] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ4Coeff[11] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ4Coeff[12] * (arm_cos_f32(q2 + q3 + q4 + q5));
}
