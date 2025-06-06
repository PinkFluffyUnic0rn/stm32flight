#include <math.h>

#include "dsp.h"

int dsp_initlpf1t(struct dsp_lpf *ir, float tcoef, int freq)
{
	ir->s1 = 0.0;
	dsp_setlpf1t(ir, tcoef,  freq);

	return 0;
}

int dsp_initlpf1f(struct dsp_lpf *ir, float cutoff, int freq)
{
	ir->s1 = 0.0;
	dsp_setlpf1f(ir, cutoff,  freq);

	return 0;
}

int dsp_setlpf1t(struct dsp_lpf *ir, float tcoef, int freq)
{
	ir->alpha = exp(-1.0 / (float) freq / tcoef);
	ir->order = DSP_LPFORDER_1;

	return 0;
}

int dsp_setlpf1f(struct dsp_lpf *ir, float cutoff, int freq)
{
	float v;

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

int dsp_initpid(struct dsp_pidval *pv, float kp, float ki, float kd,
	float dcutoff, int freq)
{
	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;

	dsp_initlpf1f(&(pv->dlpf), dcutoff, freq);

	pv->pe = 0.0;
	pv->s = 0.0;

	return 0;
}

float dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd,
	float dcutoff, int freq)
{
	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;
	
	dsp_setlpf1f(&(pv->dlpf), dcutoff, freq);

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

int dsp_initcompl(struct dsp_compl *comp, float tc, int freq)
{
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
