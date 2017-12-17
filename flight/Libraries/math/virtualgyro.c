#include "pios.h"
#include "virtualgyro.h"

/*

	Based on:

	The Scalar Kalman Filter
	http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/ScalarKalman.html

	Adaptive Adjustment of Noise Covariance in Kalman Filter for Dynamic State Estimation
	https://arxiv.org/abs/1702.00884

*/

struct mse {

	int size;
	int pos;

	int *samples;
	int xval;

	float val;

	float div;
};

struct mse *mse_create(float tau, float dt)
{
	int sz = (int)((tau * 1000.0f * dt * 1000.0f) + 0.5f);
	if (sz <= 0)
		return NULL;

	struct mse *m;

	m = PIOS_malloc_no_dma(sizeof(*m));
	if (!m)
		return NULL;

	memset(m, 0, sizeof(*m));

	m->samples = PIOS_malloc_no_dma(sizeof(int) * sz);
	if (!m->samples)
		return NULL;

	memset(m->samples, 0, sizeof(int)*sz);

	m->size = sz;
	m->div = 1.0f / (float)sz;
	m->val = 0;

	return m;
}

float mse_update(struct mse *m, float val)
{
	if (!m)
		PIOS_Assert(0);
		//return val;

	int v = (int)(val * val * m->div * 10000.0f);

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

float mse_get(struct mse *m)
{
	if (!m)
		PIOS_Assert(0);

	return m->val;
}

/* ---------------------------------------------------------------- */

struct virtualgyro_global *vgs;				/* Global runtime stuff. */

struct virtualgyro *virtualgyro_create(float dT, float tau, float R, float Q)
{
	struct virtualgyro *g;

	g = PIOS_malloc_no_dma(sizeof(*g));
	if (!g)
		return NULL;
	memset(g, 0, sizeof(*g));

#if defined(MEASURE_ACTUAL_MSE)
	g->p_post = mse_create(tau * APOSTERIORI_SCALER, dT);
	if (!g->p_post)
		return NULL;
#else
	g->p_post = 0.25f;
#endif

	g->Q = Q;
	g->R = R;
	g->dT = dT;
	g->tau = tau;

#if defined(AUTO_RQ_MSE)
	g->r_mse = mse_create(tau * APOSTERIORI_SCALER, dT);
	g->q_mse = mse_create(tau * APOSTERIORI_SCALER, dT);
#endif

	return g;
}

void virtualgyro_create_model(struct virtualgyro *g, float beta)
{
	if (!g)
		return;

	lpfilter_create(&g->lpf, 1, g->dT, 1, 1);
	g->torque = powf(M_E, beta);
}

float virtualgyro_update(struct virtualgyro *g, float xj, float actuator)
{
	if (!g)
		return xj;

	float buj = 0;

	if (g->lpf) {
		float x = (actuator - g->actuator) * g->torque * g->dT;
		buj = lpfilter_run_single(g->lpf, 0, x);
		g->actuator = actuator;
	} else {
		buj = (xj - g->xhat) * 0.5f;
	}

	/* Predict. */
	g->xhat_minus = g->xhat + buj;

	/* Calculate a priori covariance. */
#if defined(MEASURE_ACTUAL_MSE)
	g->p_prio = mse_get(g->p_post) + g->Q;
#else
	g->p_prio = g->p_post + g->Q;
#endif

	/* Calculate innovation. */
	float innovation = xj - g->xhat_minus;

	/* Calculate Kalman gain. */
	g->kj = g->p_prio / (g->p_prio + g->R);

	/* Apply correction. */
	g->xhat = g->xhat_minus + g->kj * innovation;

	/* Get a posteriori residual error for adaptive R. */
	g->residual = xj - g->xhat;

	/* Calculate/measure a posteriori covariance. */
#if defined(MEASURE_ACTUAL_MSE)
	mse_update(g->p_post, g->residual);
#else
	g->p_post = g->p_prio * (1 - g->kj);
#endif

#if defined(AUTO_PROCESS_NOISE)
	/* Do state covariance update.
	   Calculate adaptive Q.
	   As with R, paper suggests MSE, but IIR considered sufficient. */
	float alpha = 0.3f;
	g->Q = alpha * g->Q + (1 - alpha) * (g->kj * g->kj * innovation * innovation);

	/* Clamp things with generous values. Crap gets sent during initialization,
	   which upsets everything. */
	if (g->Q > 100.0f) g->Q = 100.0f;
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

float virtualgyro_get_autoQ(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->Q;
}

float virtualgyro_get_residual(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

	return g->residual;
}

float virtualgyro_get_ppost(struct virtualgyro *g)
{
	if (!g)
		PIOS_Assert(0);

#if defined(MEASURE_ACTUAL_MSE)
	return mse_get(g->p_post);
#else
	return g->p_post;
#endif
}