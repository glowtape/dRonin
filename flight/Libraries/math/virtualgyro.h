#ifndef KALMANGYRO_H
#define KALMANGYRO_H

#include "lpfilter.h"

/* How large to scale the a posteriori history for the MSE, in relation to tau. */
#define APOSTERIORI_SCALER 			2.0f

#define MEASURE_ACTUAL_MSE
// #define AUTO_PROCESS_NOISE

struct virtualgyro {

	/* Kalman filter state. */

	float p_prio;			/* A priori covariance. Not needed, only for diagnostics. */
#if defined(MEASURE_ACTUAL_MSE)
	struct mse *p_post;		/* A posteriori covariance based on moving window MSE. */
#else
	float p_post;			/* Simple posteriori covariance. */
#endif

	float xhat;				/* Corrected prediction. */
	float xhat_minus;		/* Initial prediction. Not needed, only for diagnostics. */

	float Q;				/* Process noise covariance. */
	float R;				/* Sensor noise covariance. */

	float residual;			/* Residual prediction error. Needed for adaptive R. */

	float kj;				/* Kalman gain. Not needed, only for diagnostics. */

	/* Axis actuator modeling state. */

	float dT;
	float tau;
	lpfilter_state_t lpf;
	float torque;
	float actuator;

};


void virtualgyro_initialize(float r_min, float r_max, float r_alpha, float q_min, float q_max, float q_alpha);
struct virtualgyro *virtualgyro_create(float dT, float tau, float R, float Q);
void virtualgyro_create_model(struct virtualgyro *g, float beta);
float virtualgyro_update(struct virtualgyro *g, float xj, float actuator);
float virtualgyro_get_value(struct virtualgyro *g);
float virtualgyro_get_gain(struct virtualgyro *g);
float virtualgyro_get_autoR(struct virtualgyro *g);
float virtualgyro_get_autoQ(struct virtualgyro *g);
float virtualgyro_get_residual(struct virtualgyro *g);
float virtualgyro_get_ppost(struct virtualgyro *g);

#endif // KALMANGYRO_H