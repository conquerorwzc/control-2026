#include "arm_math_c_api.h"

// Wrap original C++ headers
#include "matrix.h"
#include "robotics.h"

static inline void copy_mat_3x3_to_array(const Matrixf<3,3>& M, float out[9]) {
  const float* p = M.arm_mat_.pData;
  for (int i = 0; i < 9; ++i) {
    out[i] = p[i];
  }
}

static inline void copy_mat_4x4_to_array(const Matrixf<4,4>& M, float out[16]) {
  const float* p = M.arm_mat_.pData;
  for (int i = 0; i < 16; ++i) {
    out[i] = p[i];
  }
}

static inline void copy_vec3_to_array(const Matrixf<3,1>& V, float out[3]) {
  const float* p = V.arm_mat_.pData;
  out[0] = p[0];
  out[1] = p[1];
  out[2] = p[2];
}

// ---- Kinematics helpers (C++ linkage) ----

static inline robotics::Joint_Type_e to_joint_type(int t) {
  return (t == AM_JOINT_P) ? robotics::Joint_Type_e::P : robotics::Joint_Type_e::R;
}

template<int N>
static inline void build_links(const am_link_t* in, robotics::Link (&out)[N]) {
  for (int i = 0; i < N; ++i) {
    out[i] = robotics::Link(
      in[i].theta,
      in[i].d,
      in[i].a,
      in[i].alpha,
      to_joint_type(in[i].type),
      in[i].offset,
      in[i].qmin,
      in[i].qmax
    );
  }
}

template<int N>
static inline void fill_q(const float q_in[], Matrixf<N,1>& q) {
  for (int i = 0; i < N; ++i) {
    q[i][0] = q_in[i];
  }
}

template<int N>
static inline void copy_q_to_array(const Matrixf<N,1>& q, float q_out[]) {
  const float* p = q.arm_mat_.pData;
  for (int i = 0; i < N; ++i) {
    q_out[i] = p[i];
  }
}

template<int N>
static int am_fkine_N(const am_link_t* links, const float q_in[], float T_out[16]) {
  robotics::Link ls[N];
  build_links<N>(links, ls);
  robotics::Serial_Link<N> chain(ls);
  Matrixf<N,1> q;
  fill_q<N>(q_in, q);
  Matrixf<4,4> T = chain.fkine(q);
  copy_mat_4x4_to_array(T, T_out);
  return 0;
}

template<int N>
static int am_ikine_N(const am_link_t* links,
                      const float Td_in[16],
                      const float q_init[],
                      float tol, int max_iter,
                      float q_out[]) {
  robotics::Link ls[N];
  build_links<N>(links, ls);
  robotics::Serial_Link<N> chain(ls);
  Matrixf<4,4> Td((float*)Td_in);
  Matrixf<N,1> q0 = matrixf::zeros<N,1>();
  if (q_init) {
    fill_q<N>(q_init, q0);
  }
  Matrixf<N,1> q = chain.ikine(Td, q0, tol, (uint16_t)(max_iter > 0 ? max_iter : 50));
  // Report convergence by checking residual twist norm
  Matrixf<4,4> Tcur = chain.fkine(q);
  float err_norm = (robotics::t2twist(Td * robotics::invT(Tcur))).norm();
  copy_q_to_array<N>(q, q_out);
  return (err_norm <= tol) ? 0 : 1;
}

// ---- C-callable API ----
extern "C" {

void am_cross3(const float a[3], const float b[3], float out[3]) {
  float da[3] = {a[0], a[1], a[2]};
  float db[3] = {b[0], b[1], b[2]};
  Matrixf<3,1> A(da);
  Matrixf<3,1> B(db);
  Matrixf<3,1> C = vector3f::cross(A, B);
  copy_vec3_to_array(C, out);
}

void am_rpy2r(const float rpy[3], float R_out[9]) {
  float d[3] = {rpy[0], rpy[1], rpy[2]};
  Matrixf<3,1> RPY(d);
  Matrixf<3,3> R = robotics::rpy2r(RPY);
  copy_mat_3x3_to_array(R, R_out);
}

void am_r2rpy(const float R_in[9], float rpy_out[3]) {
  float dR[9];
  for (int i = 0; i < 9; ++i) dR[i] = R_in[i];
  Matrixf<3,3> R(dR);
  Matrixf<3,1> rpy = robotics::r2rpy(R);
  copy_vec3_to_array(rpy, rpy_out);
}

void am_r2quat(const float R_in[9], float q_out[4]) {
  float dR[9];
  for (int i = 0; i < 9; ++i) dR[i] = R_in[i];
  Matrixf<3,3> R(dR);
  Matrixf<4,1> q = robotics::r2quat(R);
  q_out[0] = q[0][0];
  q_out[1] = q[1][0];
  q_out[2] = q[2][0];
  q_out[3] = q[3][0];
}

void am_quat2r(const float q[4], float R_out[9]) {
  float dq[4] = {q[0], q[1], q[2], q[3]};
  Matrixf<4,1> Q(dq);
  Matrixf<3,3> R = robotics::quat2r(Q);
  copy_mat_3x3_to_array(R, R_out);
}

void am_rp2t(const float R_in[9], const float p_in[3], float T_out[16]) {
  float dR[9];
  for (int i = 0; i < 9; ++i) dR[i] = R_in[i];
  float dp[3] = {p_in[0], p_in[1], p_in[2]};
  Matrixf<3,3> R(dR);
  Matrixf<3,1> p(dp);
  Matrixf<4,4> T = robotics::rp2t(R, p);
  copy_mat_4x4_to_array(T, T_out);
}

void am_t2r(const float T_in[16], float R_out[9]) {
  float dT[16];
  for (int i = 0; i < 16; ++i) dT[i] = T_in[i];
  Matrixf<4,4> T(dT);
  Matrixf<3,3> R = robotics::t2r(T);
  copy_mat_3x3_to_array(R, R_out);
}

void am_t2p(const float T_in[16], float p_out[3]) {
  float dT[16];
  for (int i = 0; i < 16; ++i) dT[i] = T_in[i];
  Matrixf<4,4> T(dT);
  Matrixf<3,1> p = robotics::t2p(T);
  copy_vec3_to_array(p, p_out);
}

int am_fkine(const am_link_t* links, int n, const float q[], float T_out[16]) {
  if (!links || !q || !T_out) return -1;
  switch (n) {
    case 1: return am_fkine_N<1>(links, q, T_out);
    case 2: return am_fkine_N<2>(links, q, T_out);
    case 3: return am_fkine_N<3>(links, q, T_out);
    case 4: return am_fkine_N<4>(links, q, T_out);
    case 5: return am_fkine_N<5>(links, q, T_out);
    case 6: return am_fkine_N<6>(links, q, T_out);
    case 7: return am_fkine_N<7>(links, q, T_out);
    case 8: return am_fkine_N<8>(links, q, T_out);
    default: return -2;
  }
}

int am_ikine(const am_link_t* links, int n,
             const float Td[16], const float q_init[],
             float tol, int max_iter,
             float q_out[]) {
  if (!links || !Td || !q_out) return -1;
  if (tol <= 0) tol = 1e-4f;
  switch (n) {
    case 1: return am_ikine_N<1>(links, Td, q_init, tol, max_iter, q_out);
    case 2: return am_ikine_N<2>(links, Td, q_init, tol, max_iter, q_out);
    case 3: return am_ikine_N<3>(links, Td, q_init, tol, max_iter, q_out);
    case 4: return am_ikine_N<4>(links, Td, q_init, tol, max_iter, q_out);
    case 5: return am_ikine_N<5>(links, Td, q_init, tol, max_iter, q_out);
    case 6: return am_ikine_N<6>(links, Td, q_init, tol, max_iter, q_out);
    case 7: return am_ikine_N<7>(links, Td, q_init, tol, max_iter, q_out);
    case 8: return am_ikine_N<8>(links, Td, q_init, tol, max_iter, q_out);
    default: return -2;
  }
}

} // extern "C"
