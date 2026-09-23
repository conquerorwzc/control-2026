/**
 ******************************************************************************
 * @file    arm_gravity_params.h
 * @brief   GENERATED FILE - do not edit by hand.
 *
 * Source     : /Users/dumenghuanxiang/Documents/RoboMaster/RMCC/config/controller.json
 * Generated  : 2026-09-21T05:17:36+00:00
 * g          : 9.80665f
 *
 * Provenance of the fit:
 *   fitted_at     : 2026-09-21T05:17:14+00:00
 *   samples       : 17
 *   train RMS     : 0.02020 N*m
 *   held-out RMS  : 0.00105 N*m
 *   cond(scaled)  : 0.00
 *
 * Compensation law (strictly 2 inputs -> 2 outputs, no yaw term):
 *   tau_c1 = g*(C1*cos q1 + C2*cos(q1+q2)) + g*(S1*sin q1 + S2*sin(q1+q2))
 *   tau_c2 = g*(C3*cos(q1+q2))             + g*(C4*sin(q1+q2))
 *
 * q = 0 is HORIZONTAL and +q RAISES the link, so compensation peaks at q = 0.
 * Physically C1 = m1*c1 + m2*l1 and C2 = C3 = m2*c2 (positive static moments).
 * S1/S2/C4 absorb encoder zero-offset error and should be near zero.
 ******************************************************************************
 */

#ifndef ARM_GRAVITY_PARAMS_H
#define ARM_GRAVITY_PARAMS_H

/* gravity_comp.h provides the types and the API prototypes; this header adds
 * the identified coefficient values. Including it keeps a translation unit that
 * only does `#include "arm_gravity_params.h"` compiling. */
#include "gravity_comp.h"

/* BEGIN GENERATED PARAMS */
/* The values that must be compiled into c/gravity_comp.c. Apply with
 *   python scripts/export_c_header.py --patch
 * They are emitted as MACROS so this header is a record of what was flashed,
 * not a second definition that would clash at link time. */
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

#endif /* ARM_GRAVITY_PARAMS_H */
