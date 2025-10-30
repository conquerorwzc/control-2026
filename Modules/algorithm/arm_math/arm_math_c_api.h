// C-callable wrapper API for C++ matrix/robotics library
// This header is safe to include from C files.
#ifndef ARM_MATH_C_API_H
#define ARM_MATH_C_API_H

#ifdef __cplusplus
extern "C" {
#endif

// Conventions:
// - All matrices/vectors are row-major float arrays.
// - rpy = [yaw, pitch, roll] in radians.
// - q = [q0, q1, q2, q3] with q0 = cos(theta/2).
// - R is 3x3 (length 9), p is 3 (length 3), T is 4x4 (length 16).

// Basic vector algebra
void am_cross3(const float a[3], const float b[3], float out[3]);

// Rotation conversions
void am_rpy2r(const float rpy[3], float R_out[9]);
void am_r2rpy(const float R_in[9], float rpy_out[3]);
void am_r2quat(const float R_in[9], float q_out[4]);
void am_quat2r(const float q[4], float R_out[9]);

// Homogeneous transform helpers
void am_rp2t(const float R_in[9], const float p_in[3], float T_out[16]);
void am_t2r(const float T_in[16], float R_out[9]);
void am_t2p(const float T_in[16], float p_out[3]);

// ---------------- Robot kinematics (C API) ----------------
// Joint type: 0 = revolute (R), 1 = prismatic (P)
#define AM_JOINT_R 0
#define AM_JOINT_P 1

typedef struct am_link_s {
  // DH parameters (standard DH): theta, d, a, alpha
  float theta;
  float d;
  float a;
  float alpha;
  // joint config
  int   type;     // AM_JOINT_R or AM_JOINT_P
  float offset;   // added to q before applying to DH (theta for R, d for P)
  float qmin;     // limit min (if qmin <= qmax, limit is active)
  float qmax;     // limit max
} am_link_t;

// Forward kinematics: compute end-effector T (4x4) from links and q.
// - links: array of length n
// - n: DOF, supported 1..8; returns -2 if n out of range
// - q: joint vector of length n (rad for R, meters for P)
// - T_out: 4x4 row-major
// Return 0 on success; <0 on error (e.g., -1 bad args, -2 unsupported n)
int am_fkine(const am_link_t* links, int n, const float q[], float T_out[16]);

// Inverse kinematics: numerical Newton solver.
// - links, n like above (1..8)
// - Td: desired 4x4 target pose
// - q_init: initial guess vector length n (optional, can be NULL => zeros)
// - tol: error norm tolerance (e.g., 1e-4)
// - max_iter: iteration cap (e.g., 50)
// - q_out: result vector length n
// Return 0 on success (achieved <= tol), 1 on converged but > tol, <0 on error
int am_ikine(const am_link_t* links, int n,
             const float Td[16], const float q_init[],
             float tol, int max_iter,
             float q_out[]);

#ifdef __cplusplus
}
#endif

#endif // ARM_MATH_C_API_H
