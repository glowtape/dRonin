#ifndef LQG_H
#define LQG_H

#include <pios.h>
#include <misc_math.h>

typedef struct rtkf_state* rtkf_t;
typedef struct lqr_state* lqr_t;

typedef struct lqg_state* lqg_t;

extern rtkf_t rtkf_create(float beta, float tau, float Ts, float R, float Q1, float Q2, float Q3, float biaslim);
extern void rtkf_stabilize_covariance(rtkf_t rtkf, int iterations);
extern void rtkf_predict_axis(rtkf_t rtkf, float signal, float input, float Xout[3]);
extern bool rtkf_is_solved(rtkf_t rtkf);

extern lqr_t lqr_create(float beta, float tau, float Ts, float q1, float q2);
extern void lqr_update(lqr_t lqr, float q1, float q2);
extern void lqr_stabilize_covariance(lqr_t lqr, int iterations);
extern bool lqr_is_solved(lqr_t lqr);
extern void lqr_get_gains(lqr_t lqg, float K[2]);

extern lqg_t lqg_create(int axis, rtkf_t rtkf, lqr_t lqr);
extern void lqg_set_x0(lqg_t lqq, float x0);
extern void lqg_run_solver(lqg_t lqg);

extern void lqg_get_axes(lqg_t *axes);
//extern void lqg_stabilize_lqr_covariances(lqg_t lqg);
extern float lqg_controller(lqg_t lqg, float signal, float setpoint);
extern bool lqg_is_solved(lqg_t lqg);
extern void lqg_get_estimate(lqg_t lqg, float *rate, float *torque, float *bias);
extern void lqg_override_u(lqg_t lqg, float u);

extern lqr_t lqg_get_lqr(lqg_t lqg);
extern rtkf_t lqg_get_rtkf(lqg_t lqg);


#endif // LQG_H
