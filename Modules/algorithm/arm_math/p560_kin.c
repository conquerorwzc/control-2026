#include "p560_kin.h"
#include "general_def.h"  // for PI

static void set_link(am_link_t* L, float theta, float d, float a, float alpha,
                     int type, float offset, float qmin, float qmax) {
  L->theta = theta;
  L->d = d;
  L->a = a;
  L->alpha = alpha;
  L->type = type;
  L->offset = offset;
  L->qmin = qmin;
  L->qmax = qmax;
}

void p560_build_links(am_link_t links[6], const float offsets[6]) {
  float off[6] = {0,0,0,0,0,0};
  if (offsets) {
    for (int i = 0; i < 6; ++i) off[i] = offsets[i];
  }
  // Standard DH per your spec (meters / radians)
  set_link(&links[0], 0.0f, 0.2645f, 0.0f,   -PI * 0.5f, AM_JOINT_R, off[0], -PI, PI);
  set_link(&links[1], 0.0f, 0.0550f, 0.1700f, 0.0f,     AM_JOINT_R, off[1], -PI, PI);
  set_link(&links[2], 0.0f, 0.0f,    0.0f,   -PI * 0.5f, AM_JOINT_R, off[2], -PI, PI);
  set_link(&links[3], 0.0f, 0.1705f, 0.0f,    PI * 0.5f, AM_JOINT_R, off[3], -PI, PI);
  set_link(&links[4], 0.0f, 0.0f,    0.0f,   -PI * 0.5f, AM_JOINT_R, off[4], -PI, PI);
  set_link(&links[5], 0.0f, 0.0f,    0.0f,    0.0f,      AM_JOINT_R, off[5], -PI, PI);
}

int p560_fkine(const float q[6], float T_out[16], const float offsets[6]) {
  if (!q || !T_out) return -1;
  am_link_t links[6];
  p560_build_links(links, offsets);
  return am_fkine(links, 6, q, T_out);
}

int p560_ikine(const float Td[16], const float q_init[6],
               float tol, int max_iter,
               float q_out[6],
               const float offsets[6]) {
  if (!Td || !q_out) return -1;
  am_link_t links[6];
  p560_build_links(links, offsets);
  return am_ikine(links, 6, Td, q_init, tol, max_iter, q_out);
}

