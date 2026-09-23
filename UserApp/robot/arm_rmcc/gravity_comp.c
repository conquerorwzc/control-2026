/**
 ******************************************************************************
 * @file    gravity_comp.c
 * @brief   Gravity + friction feed-forward for the yaw-pitch-pitch field arm.
 *          See gravity_comp.h and docs/design-contract.md.
 ******************************************************************************
 */

#include "gravity_comp.h"

#include <math.h>

/* -------------------------------------------------------------------------
 * Identified coefficients.
 *
 * scripts/export_c_header.py --patch rewrites the block between the GENERATED
 * markers below directly from config/controller.json, so the MCU and the
 * simulator cannot drift apart. The values here are the safe
 * "compensation disabled" defaults.
 *
 * These are macros, not struct objects, so this file stays self-contained: the
 * generated arm_gravity_params.h is then a human-readable record of what was
 * flashed rather than a second definition that could clash at link time.
 * ---------------------------------------------------------------------- */
/* BEGIN GENERATED PARAMS */
/* Generated from controller.json by scripts/export_c_header.py - do not edit. */
#define GC_C1 0.341841543f
#define GC_C2 0.0811663373f
#define GC_C3 0.0810007813f
#define GC_S1 6.90456546e-05f
#define GC_S2 3.4992185e-05f
#define GC_C4 7.1072566e-06f
#define GC_FR_B1 0.0f
#define GC_FR_TC1 0.0f
#define GC_FR_O1 0.0f
#define GC_FR_B2 0.0f
#define GC_FR_TC2 0.0f
#define GC_FR_O2 0.0f
#define GC_FR_EPS 0.02f
/* END GENERATED PARAMS */

/* Standard gravity, matching sim/plant.py G_STANDARD. */
#define GRAVITY_COMP_G 9.80665f

static GravityCompConfig s_cfg = {
    0.0f, 0.0f,          /* q1_zero, q2_zero */
    1.0f, 1.0f,          /* q1_sign, q2_sign */
    1.0f, 0.0f,          /* k_grav, k_fric (friction off by default) */
    10.0f                /* tau_max: DM-J4310 */
};

static float clampf_sym(float x, float limit) {
  if (x > limit) return limit;
  if (x < -limit) return -limit;
  return x;
}

void GravityComp_Init(float q1_zero, float q2_zero, float q1_sign, float q2_sign) {
  s_cfg.q1_zero = q1_zero;
  s_cfg.q2_zero = q2_zero;
  s_cfg.q1_sign = (q1_sign < 0.0f) ? -1.0f : 1.0f;
  s_cfg.q2_sign = (q2_sign < 0.0f) ? -1.0f : 1.0f;
}

GravityCompConfig *GravityComp_GetConfig(void) { return &s_cfg; }

void GravityComp_Calc(float q1, float q2, float *tau1, float *tau2) {
  /* raw encoder -> contract convention (0 = link horizontal, +q raises it) */
  q1 = (q1 - s_cfg.q1_zero) * s_cfg.q1_sign;
  q2 = (q2 - s_cfg.q2_zero) * s_cfg.q2_sign;

  const float c1 = cosf(q1);
  const float c12 = cosf(q1 + q2);
  const float s1 = sinf(q1);
  const float s12 = sinf(q1 + q2);

  /* tau_comp = +g*(C1 cos q1 + C2 cos(q1+q2)) + phase-error terms.
   * Convention: q = 0 is HORIZONTAL and +q raises the link. So at q = 0 the
   * compensation is at its maximum, +g*(C1+C2) = +4.148 N*m for this arm -
   * the torque needed to hold it straight out. A cos/sin swap here used to
   * return ZERO at the horizontal pose, i.e. wrong by the whole static load;
   * tests/test_c_parity.py compares against the Python model pointwise. */
  float t1 = GC_C1 * (GRAVITY_COMP_G * c1) + GC_C2 * (GRAVITY_COMP_G * c12) +
             GC_S1 * (GRAVITY_COMP_G * s1) + GC_S2 * (GRAVITY_COMP_G * s12);
  float t2 = GC_C3 * (GRAVITY_COMP_G * c12) + GC_C4 * (GRAVITY_COMP_G * s12);

  t1 *= s_cfg.k_grav;
  t2 *= s_cfg.k_grav;

  /* the offsets/signs above are applied to the INPUT; the torque follows the
   * same sign convention so that it is applied in the motor's own frame */
  if (s_cfg.q1_sign < 0.0f) t1 = -t1;
  if (s_cfg.q2_sign < 0.0f) t2 = -t2;

  if (tau1) *tau1 = clampf_sym(t1, s_cfg.tau_max);
  if (tau2) *tau2 = clampf_sym(t2, s_cfg.tau_max);
}

void GravityComp_FrictionComp(float dq1, float dq2, float *tau1, float *tau2) {
  const float eps = (GC_FR_EPS > 1e-6f) ? GC_FR_EPS : 1e-6f;

  dq1 = dq1 * s_cfg.q1_sign;
  dq2 = dq2 * s_cfg.q2_sign;

  float t1 = GC_FR_B1 * dq1 + GC_FR_TC1 * tanhf(dq1 / eps) + GC_FR_O1;
  float t2 = GC_FR_B2 * dq2 + GC_FR_TC2 * tanhf(dq2 / eps) + GC_FR_O2;

  t1 *= s_cfg.k_fric;
  t2 *= s_cfg.k_fric;

  if (s_cfg.q1_sign < 0.0f) t1 = -t1;
  if (s_cfg.q2_sign < 0.0f) t2 = -t2;

  if (tau1) *tau1 = clampf_sym(t1, s_cfg.tau_max);
  if (tau2) *tau2 = clampf_sym(t2, s_cfg.tau_max);
}

void GravityComp_CalcFull(float q1, float q2, float dq1, float dq2,
                          float *tau1, float *tau2) {
  float g1, g2, f1, f2;
  GravityComp_Calc(q1, q2, &g1, &g2);
  GravityComp_FrictionComp(dq1, dq2, &f1, &f2);
  if (tau1) *tau1 = clampf_sym(g1 + f1, s_cfg.tau_max);
  if (tau2) *tau2 = clampf_sym(g2 + f2, s_cfg.tau_max);
}

float GravityComp_MaxAbsTorque(void) {
  /* The bound of a1*sin(x) + a2*sin(y) is the amplitude sum, reached when both
   * links are horizontal. Includes the cosine (zero-offset) columns so the
   * safety check cannot be fooled by a mis-calibrated encoder zero. */
  const float a1 = GRAVITY_COMP_G * (fabsf(GC_C1) + fabsf(GC_C2) + fabsf(GC_S1) + fabsf(GC_S2));
  const float a2 = GRAVITY_COMP_G * (fabsf(GC_C3) + fabsf(GC_C4));
  float m = (a1 > a2) ? a1 : a2;
  m *= s_cfg.k_grav;
  return (m < 0.0f) ? -m : m;
}
