#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 */

#define MEC_ARM_GRAVITY_J2_TERM_NUM 17
static const float kMecArmGravityJ2Coeff[17] = {
    -4.218924927e-02f,
    -3.551706700e-02f,
    1.830154735e-02f,
    -8.847123618e-03f,
    3.139548738e-02f,
    6.316246823e-03f,
    -6.691416749e-03f,
    -1.998192895e-02f,
    1.167964064e-02f,
    4.680356567e-03f,
    3.099533508e-03f,
    3.057615111e-02f,
    -1.665396475e-02f,
    1.815198281e-02f,
    -8.196215110e-03f,
    2.196908853e-04f,
    -7.000205721e-03f,
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
    -2.544385340e-02f,
    -1.602438330e-01f,
    -6.048620002e-02f,
    -9.139596331e-02f,
    -2.658865855e-02f,
    -1.799442223e-02f,
    -1.029903321e-02f,
    1.161283435e-01f,
    -3.894499041e-01f,
    1.958202303e-02f,
    -2.134893544e-01f,
    -5.931772735e-02f,
    1.135781460e-02f,
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
    1.623555464e-02f,
    5.796708731e-02f,
    -3.009826630e-02f,
    1.687137633e-02f,
    -2.796084612e-02f,
    -7.816340200e-02f,
    -2.765792540e-02f,
    -1.486740462e-02f,
    1.602439230e-02f,
    2.530569398e-02f,
    -7.880140426e-02f,
    -1.139384452e-02f,
    -2.566355186e-02f,
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
