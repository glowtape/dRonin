#include "pios.h"
#include "virtualgyro.h"

/*

	Based on:

	The Scalar Kalman Filter
	http://www.swarthmore.edu/NatSci/echeeve1/Ref/Kalman/ScalarKalman.html

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
	g->Q = Q;
	g->R = R;
	g->dT = dT;
	g->tau = powf(M_E, tau);

#if defined(MEASURE_ACTUAL_MSE)
	g->p_post = mse_create(g->tau * FULL_MSE_COV_TAU_SCALE, dT);
#else
	g->p_tau = g->tau * SIMPLE_COV_TAU_SCALE;
	g->p_post = 0.25f;
#endif
}

void virtualgyro_set_model(struct virtualgyro *g, float beta)
{
	if (!g)
		return;

	float tau = g->tau;
	/* Drop tau a tiny bit to compensate for digital 2nd order delay. */
	if (tau > 0.009f) tau -= 0.001f;

	lpfilter_create(&g->lpf, 1.0f / (tau * (float)M_PI * 1.414f), g->dT, 2, 1);
	g->torque = powf(M_E, beta);
}

float virtualgyro_update(struct virtualgyro *g, float xj, float actuator)
{
#if defined(MEASURE_ACTUAL_MSE)
	if (!g || !g->p_post)
#else
	if (!g)
#endif
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

	/* Try to maintain Kalman gain below 0.25 by adapting R. */
	if (xj < 180.0f) {
		/* Only do this during "small" rotational speeds . */
		if (g->kj > 0.25f) {
			g->R += 0.07f;
		} else if (g->kj < 0.09f) {
			g->R -= 0.05f;
		}
		if (g->R < 0.5f) g->R = 0.5f;
		else if (g->R > 250.0f) g->R = 250.0f;
	}

	/* Calculate/measure a posteriori covariance. */
#if defined(MEASURE_ACTUAL_MSE)
	mse_update(g->p_post, g->residual);
#else
	// g->p_post = g->p_prio * (1 - g->kj);
	g->p_post = g->p_post + (g->dT / (g->p_tau + g->dT)) * (g->residual * g->residual - g->p_post);
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