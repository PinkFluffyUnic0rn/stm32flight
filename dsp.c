#include <math.h>

#include "dsp.h"

int dsp_setunity(struct dsp_lpf *ir, int init)
{
	if (init)
		ir->s1 = 0.0;

	ir->alpha = 0.0;
	ir->order = DSP_LPFORDER_0;

	return 0;
}

int dsp_setlpf1t(struct dsp_lpf *ir, float tcoef, int freq, int init)
{
	if (init)
		ir->s1 = 0.0;

	ir->alpha = exp(-1.0 / (float) freq / tcoef);
	ir->order = DSP_LPFORDER_1;

	return 0;
}

int dsp_setlpf1f(struct dsp_lpf *ir, float cutoff, int freq, int init)
{
	float v;

	if (init)
		ir->s1 = 0.0;

	v = 2.0f * M_PI * cutoff / (float) freq;

	ir->alpha = 1.0 - v / (v + 1.0f);
	ir->order = DSP_LPFORDER_1;

	return 0;
}

float dsp_getlpf(struct dsp_lpf *ir)
{
	return ir->s1;
}

float dsp_updatelpf(struct dsp_lpf *ir, float v)
{
	ir->s1 = ir->alpha * ir->s1 + (1 - ir->alpha) * v;

	return ir->s1;
}

int dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd,
	float dcutoff, int freq, int init)
{
	if (init) {
		pv->pe = 0.0;
		pv->s = 0.0;
	}

	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;
	
	dsp_setlpf1f(&(pv->dlpf), dcutoff, freq, init);

	return 0;
}

int dsp_setpidbl(struct dsp_pidblval *pv, float kp, float ki,
	float kd, float imax, float dcutoff, int freq, int init)
{
	float ts, tf, tt, c;

	pv->step = 0;
	pv->depth = 2;

	pv->imax = imax;

	ts = 1.0 / (float) freq;
	tf = 1.0 / (dcutoff * 2.0 * M_PI);
	tt = ts * ts;

	c = 1.0 / (4.0 * tf + 2.0 * ts);

	pv->a[0] = (4.0 * kd + 4.0 * tf * kp + 2.0 * kp * ts) * c;
	pv->a[1] = (-8.0 * kd - 8.0 * tf * kp) * c;
	pv->a[2] = (4.0 * kd + 4.0 * tf * kp - 2.0 * kp * ts) * c;
	pv->ia[0] = ki * (tt + 2.0 * tf * ts) * c;
	pv->ia[1] = ki * (2.0 * tt) * c;
	pv->ia[2] = ki * (tt - 2.0 * tf * ts) * c;

	pv->b[0] = 1.0;
	pv->b[1] = 8.0 * tf * c;
	pv->b[2] = -(4.0 * tf - 2.0 * ts) * c;

	return 0;
}

float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt)
{
	float e, v;

	e = target - val;

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd
		* dsp_updatelpf(&(pv->dlpf), (e - pv->pe) / dt);

	pv->pe = e;

	return v;
}

float dsp_pidbl(struct dsp_pidblval *pv, float target, float val)
{
	float e, v, vs;

	e = target - val;

	switch (pv->step) {
	case 0:
		pv->i = pv->ia[0] * e;
		v = pv->a[0] * e;
		vs = v + pv->i;
		break;

	case 1:
		pv->i = pv->ia[0] * e
			+ pv->ia[1] * pv->e[0] + pv->b[1] * pv->iv[0];
		
		v = pv->a[0] * e
			+ pv->a[1] * pv->e[0] + pv->b[1] * pv->v[0];
		
		vs = v + pv->i;
	
		break;

	default:
		pv->i = pv->ia[0] * e
			+ pv->ia[1] * pv->e[0] + pv->b[1] * pv->iv[0]
			+ pv->ia[2] * pv->e[1] + pv->b[2] * pv->iv[1];

		v = pv->a[0] * e
			+ pv->a[1] * pv->e[0] + pv->b[1] * pv->v[0]
			+ pv->a[2] * pv->e[1] + pv->b[2] * pv->v[1];
		
		vs = v + pv->i;

		break;
	}

	if (pv->i < -pv->imax)
		pv->iv[0] = pv->i = -pv->imax;
	else if (pv->i > pv->imax)
		pv->iv[0] = pv->i = pv->imax;

	pv->e[1] = pv->e[0];
	pv->v[1] = pv->v[0];
	pv->iv[1] = pv->iv[0];
	pv->e[0] = e;
	pv->v[0] = v;
	pv->iv[0] = pv->i;

	if (pv->step < pv->depth)
		++pv->step;

	return vs;
}

int dsp_resetpidbls(struct dsp_pidblval *pv)
{
	pv->iv[0] = pv->iv[1] = pv->iv[2] = pv->i = 0.0;

	return 0;
}

float dsp_circpid(struct dsp_pidval *pv, float target,
	float val, float dt)
{
	float e, v;

	e = circf(target - val);

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd * (e - pv->pe) / dt;

	pv->pe = e;

	return v;
}

int dsp_setcompl(struct dsp_compl *comp, float tc, int freq, int init)
{
	if (init)
		comp->s = 0;

	comp->coef = tc / (tc + 1.0 / (float) freq);

	return 0;
}

float dsp_getcompl(struct dsp_compl *comp)
{
	return comp->s;
}

float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1)
{
	comp->s = comp->coef * (comp->s + v0) + (1.0 - comp->coef) * v1;

	return comp->s;
}

float dsp_updatecirccompl(struct dsp_compl *comp, float v0, float v1)
{
	float s;

	s = circf(comp->s + v0);

	if (fabsf(s - v1) > M_PI) {
		if (s < 0)	s += 2.0 * M_PI;
		if (v1 < 0)	v1 += 2.0 * M_PI;
	}

	comp->s = circf(comp->coef * s + (1.0 - comp->coef) * v1);

	return comp->s;
}
