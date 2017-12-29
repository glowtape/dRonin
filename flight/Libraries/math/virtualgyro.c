#include "pios.h"
#include "coordinate_conversions.h"

#include "virtualgyro.h"

/*

	Based on:

	Scalar KF
		The Scalar Kalman Filter
		http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/ScalarKalman.html

	Two state KF
		A practical approach to Kalman filter and how to implement it
		http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/

	Auto process noise (exp.)
		Adaptive Adjustment of Noise Covariance in Kalman Filter for Dynamic State Estimation
		https://arxiv.org/abs/1702.00884

	A = [  1  dT  ]
	    [  0   1  ]

	B = [  dT  ]
	    [   0  ]

	t = Desired torque (e^Beta * ActuatorDesired.Axis)

	u = u + (dT/(dT+tau)) * (t - u)  -- or -- Biquad lowpass with cutoff 1/(1.4142*PI*tau)

*/

/* Fixed-point windowed mean squared error */

struct mse {

	int size;
	int pos;

	uint *samples;
	uint xval;

	float val;

	float div;
};

struct mse *mse_create(int samples)
{
	if (samples <= 0)
		return NULL;

	struct mse *m;

	m = PIOS_malloc_no_dma(sizeof(*m));
	if (!m)
		return NULL;

	memset(m, 0, sizeof(*m));

	m->samples = PIOS_malloc_no_dma(sizeof(uint) * samples);
	if (!m->samples)
		return NULL;

	memset(m->samples, 0, sizeof(int)*samples);

	m->size = samples;
	m->div = 1.0f / (float)samples;
	m->val = 0;

	return m;
}

float mse_update(struct mse *m, float val)
{
	if (!m)
		PIOS_Assert(0);
		//return val;

	uint v = (uint)(val * val * 10000.0f * m->div);

	m->xval -= m->samples[m->pos];
	m->xval += v;
	m->samples[m->pos] = v;
	m->val = (float)m->xval * 0.0001f;

	m->pos++;
	m->pos %= m->size;

/*
	float x = 0;
	for (int i = 0; i < m->size; i++)
		x += m->samples[i];

	m->val = x;
*/
	return m->val;
}

inline float mse_get(struct mse *m)
{
	if (!m)
		PIOS_Assert(0);

	return m->val;
}

/* Virtual gyro */

struct virtualgyro_global *vgs;				/* Global runtime stuff. */

struct virtualgyro *virtualgyro_create()
{
	struct virtualgyro *g;

	g = PIOS_malloc_no_dma(sizeof(*g));
	if (!g)
		return NULL;
	memset(g, 0, sizeof(*g));

	return g;
}

void virtualgyro_configure(struct virtualgyro *g, float dT, float tau, float R, float Q)
{
	g->_Q = Q;
	g->R = R;
	g->dT = dT;
	g->tau = powf(M_E, tau);

#if defined(MEASURE_ACTUAL_MSE)
	g->p_post = mse_create((int)((float)MSE_WINDOW_MS / (1000.0f * dT) + 0.5f));
	g->i_cov = mse_create((int)((float)MSE_WINDOW_MS / (1000.0f * dT) + 0.5f));
#else
	g->p_tau = g->tau * SIMPLE_COV_TAU_SCALE;
	g->p_post = 0.25f;
#endif
	g->Q[0][0] = Q;
	g->Q[1][1] = Q*3;
	g->P[0][0] = 0;
	g->P[1][1] = 0;

#if defined(AUTO_NOISE_USE_TAU)
	g->auto_noise_alpha = exp(-dT/tau);
#else
	g->auto_noise_alpha = AUTO_NOISE_ALPHA;
#endif
}

void virtualgyro_set_model(struct virtualgyro *g, float beta)
{
	if (!g)
		return;

	float tau = g->tau;
	/* Drop tau a tiny bit to compensate for digital 2nd order delay. */
	if (tau > 0.009f) tau -= 0.001f;

	lpfilter_create(&g->lpf, 1.0f / (tau * (float)M_PI * 2.0f), g->dT, 1, 1);
	g->torque = powf(M_E, beta);
}

void virtualgyro_deconvolve(struct virtualgyro *g, float rate)
{
	for (int i = 0; i < 10; i++) {
		if (g->actuator_last[i] > 5) {
			g->step_response[i] = rate / g->actuator_last[i];
		}
		else
			g->step_response[i] = 0;
	}
}

float virtualgyro_convolve(struct virtualgyro *g, float actuators[10])
{
	float buj = 0;
	for (int i = 0; i < 10; i++) {
		if (g->actuator_last[i] > 5 && actuators[i] > 5)
			buj += g->step_response[i] * (actuators[i] - g->actuator_last[i]);
		g->actuator_last[i] = actuators[i];
	}
	return buj;
}
/*
float virtualgyro_convolve_predict(struct virtualgyro *g, float actuators[10])
{
	float pred = 0;
	for (int i = 0; i < 10; i++) {
		if (g->actuator_last[i] > 5 && actuators[i] > 5)
			pred += g->step_response[i] * actuators[i];
		g->actuator_last[i] = actuators[i];
	}
	return pred;
}
*/

bool virtualgyro_enable_auto_r(struct virtualgyro *restrict g, float throttle)
{
	if (!g->auto_r) {
		if (throttle > 0.25f) {
			g->auto_r_armcnt++;
			if (g->auto_r_armcnt > (int)(1.0f / g->dT))
				g->auto_r = true;
		}
	}
	return g->auto_r;
}

float virtualgyro_update_biased(struct virtualgyro *restrict g, float rate, float a, float throttle, float armed, bool yaw)
{
	if (!g)
		return rate;

	float torque = g->torque * a;
	//float torque = g->torque * a;

	float bu = 0;

	if (g->lpf) {
		torque = lpfilter_run_single(g->lpf, 0, torque);
		torque += (armed && !yaw ? g->bias : 0);
		g->actuator = a;
		torque *= g->dT;
	} else {
		torque = (rate - g->xhat) * 0.5f;
	}

	g->buj = bu;
	g->xhat_minus = g->xhat + torque;

	float prio[2][2];

#if defined(MEASURE_ACTUAL_MSE)
	g->P[0][0] = mse_get(g->p_post);
#else
	g->P[0][0] = g->p_post;
#endif

	prio[0][0] = g->P[0][0] - g->P[1][0] * g->dT - g->dT*(g->P[0][1] - g->P[1][1]*g->dT) + g->Q[0][0] * g->dT;
	prio[0][1] = g->P[0][1] - g->P[1][1] * g->dT;
	prio[1][0] = g->P[1][0] - g->P[1][1] * g->dT;
	prio[1][1] = g->P[1][1] + g->Q[1][1] * g->dT;

	float innov = rate - g->xhat_minus;

	float innov_cov = prio[0][0] + g->R;

	float K[2];

	K[0] = g->P[0][0] / innov_cov;
	K[1] = g->P[1][0] / innov_cov;
	g->kj = K[0];

	g->xhat = g->xhat_minus + K[0] * innov;
	g->bias += (armed ? K[1] * innov : 0);

	/* Cap torque bias to a third. Change to I-term limit percentage. */
	if (fabsf(g->bias*3.f) > g->torque) {
		g->bias = g->torque * 0.333333f * (g->bias < 0 ? -1 : 1);
	}

	g->P[0][0] = (1 - K[0]) * prio[0][0];
	g->P[0][1] = (1 - K[0]) * prio[0][1];
	g->P[1][0] = (1 - K[1]) * prio[0][0];
	g->P[1][1] = (1 - K[1]) * prio[0][1];

	/* Measure actual covariance and overwrite P[0][0].
	   Currently filtering by tau. Should probably be 10hz or something. */
	g->residual = rate - g->xhat;

#if defined(MEASURE_ACTUAL_MSE)
	mse_update(g->p_post, g->residual);
#else
	g->p_post = g->p_post + (g->dT / (g->p_tau + g->dT)) * (g->residual * g->residual - g->p_post);
#endif

	/* Try to maintain Kalman gain below 0.25 by adapting R. */
	if (virtualgyro_enable_auto_r(g, throttle) && torque < 5000.0f*g->dT && g->xhat < 120.0f) {
		// Only do this during "small" rotational speeds .
		if (g->kj > 0.25f) {
			g->R += 0.07f;
		} else if (g->kj < 0.09f) {
			g->R -= 0.05f;
		}
		if (g->R < 10.0f) g->R = 10.0f;
		else if (g->R > 250.0f) g->R = 250.0f;
	}

#if defined(AUTO_PROCESS_NOISE)
	/* Do state covariance update.
	   Calculate adaptive Q.
	   As with R, paper suggests MSE, but IIR considered sufficient. */
#if defined(AUTO_NOISE_USE_TAU)
	g->Q[0][0] = g->Q[0][0] + (g->dT/(g->dT+g->tau)) * (K[0] * K[0] * innov * innov - g->Q[0][0]);
	g->Q[1][1] = g->Q[1][1] + (g->dT/(g->dT+g->tau)) * (K[1] * K[1] * innov * innov - g->Q[1][1]);
#else
	g->Q[0][0] = g->AUTO_NOISE_ALPHA * g->Q[0][0] + (1 - g->auto_noise_alpha) * (K[0] * K[0] * innov * innov);
	g->Q[1][1] = g->AUTO_NOISE_ALPHA * g->Q[1][1] + (1 - g->auto_noise_alpha) * (K[1] * K[1] * innov * innov);
#endif

	/* Clamp things with generous values. Crap gets sent during initialization,
	   which upsets everything. */
	if (g->Q[0][0] > 100.0f) g->Q[0][0] = 100.0f;
	if (g->Q[1][1] > 100.0f) g->Q[1][1] = 100.0f;
#endif

	return g->xhat;
}

float virtualgyro_get_value(struct virtualgyro *g)
{
	if (!g)
		return 0;

	return g->xhat;
}

float virtualgyro_get_gain(struct virtualgyro *g)
{
	if (!g)
		return 0;

	return g->kj;
}

float virtualgyro_get_autoR(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->R;
}

float virtualgyro_get_residual(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->residual;
}

float virtualgyro_get_cov(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

#if defined(MEASURE_ACTUAL_MSE)
	return mse_get(g->p_post);
#else
	return g->p_post;
#endif
}

float virtualgyro_get_buj(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->buj;
}

float virtualgyro_get_bias(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->bias;
}