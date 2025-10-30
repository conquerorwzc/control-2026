#ifndef P560_KIN_H
#define P560_KIN_H

#include "arm_math_c_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// Build links for a 6-DOF p560-style arm using standard DH parameters.
// Units: d/a in meters, alpha/theta in radians.
// offsets: optional per-joint theta offsets (rad). If NULL, all zeros.
void p560_build_links(am_link_t links[6], const float offsets[6]);

// Convenience FK wrapper: builds links and calls am_fkine.
// q: 6 joint angles (rad)
// offsets: optional per-joint offsets (rad), may be NULL
// Returns 0 on success, <0 on error.
int p560_fkine(const float q[6], float T_out[16], const float offsets[6]);

// Convenience IK wrapper: builds links and calls am_ikine.
// Td: desired 4x4 pose (row-major)
// q_init: initial guess (rad), may be NULL => zeros
// tol: tolerance (<=0 uses default 1e-4)
// max_iter: max iterations
// offsets: optional per-joint offsets (rad), may be NULL
// Returns 0 if ||err||<=tol, 1 if converged but >tol, <0 on error.
int p560_ikine(const float Td[16], const float q_init[6],
               float tol, int max_iter,
               float q_out[6],
               const float offsets[6]);

#ifdef __cplusplus
}
#endif

#endif // P560_KIN_H

