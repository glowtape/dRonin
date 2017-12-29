#ifndef VIRTUALGYRO_H
#define VIRTUALGYRO_H

#include "lpfilter.h"

/* Mean squared error window length in milliseconds. Ought to be long enough to capture at
   least one full oscillation of the frame vibration. 100hz is 10ms. */
#define MSE_WINDOW_MS		25

/* Uses mean squared error to measure covariance, instead of IIR with time constant. */
#define MEASURE_ACTUAL_MSE

/* Automatic sensor noise as per Arvix paper 1702.00884, instead of Kalman gain based ramping. */
#define AUTO_SENSOR_NOISE

/* Automatic process noise as per Arvix paper 1702.00884 */
#define AUTO_PROCESS_NOISE

/* Uses tau for automatic process/sensor noise. */
#define AUTO_NOISE_USE_TAU

/* Noise alpha value */
#define AUTO_NOISE_ALPHA 0.9f

struct virtualgyro {

	/* Kalman filter state. */

	float p_prio;			/* A priori covariance. Not needed, only for diagnostics. */
#if defined(MEASURE_ACTUAL_MSE)
	struct mse *p_post;		/* A posteriori covariance based on moving window MSE. */
#else
	float p_tau;			/* Forgetting factor. */
	float p_post;			/* Simple posteriori covariance. */
#endif

	float P[2][2];			/* Covariance matrix */
	float Q[2][2];			/* Process noise */

	float xhat;				/* Corrected prediction. */
	float xhat_minus;		/* Initial prediction. Not needed, only for diagnostics. */
	float bias;				/* Torque bias */

	float _Q;				/* Process noise covariance. */
	float R;				/* Sensor noise covariance. */

	float residual;			/* Residual prediction error. Needed for adaptive R. */

	float kj;				/* Kalman gain. Not needed, only for diagnostics. */

	float auto_noise_alpha; /* */

	/* Axis actuator modeling state. */

	float dT;
	float tau;
	lpfilter_state_t lpf;
	float torque;
	float actuator;

	float buj;

	float actuator_last[10];
	float step_response[10];

};


void virtualgyro_initialize(float r_min, float r_max, float r_alpha, float q_min, float q_max, float q_alpha);
struct virtualgyro *virtualgyro_create();
void virtualgyro_configure(struct virtualgyro *g, float dT, float tau, float R, float Q);
void virtualgyro_set_model(struct virtualgyro *g, float beta);
// float virtualgyro_update(struct virtualgyro *g, float xj, float a, float actuators[10]);
float virtualgyro_update_biased(struct virtualgyro *g, float xj, float a, float armed, bool yaw);
float virtualgyro_get_value(struct virtualgyro *g);
float virtualgyro_get_gain(struct virtualgyro *g);
float virtualgyro_get_autoR(struct virtualgyro *g);
float virtualgyro_get_residual(struct virtualgyro *g);
float virtualgyro_get_cov(struct virtualgyro *g);
float virtualgyro_get_buj(struct virtualgyro *g);
float virtualgyro_get_bias(struct virtualgyro *g);

#endif // KALMANGYRO_H