#include <math.h>

#include "dsp.h"

int dsp_initlpf(struct dsp_lpf *ir, float tcoef, int freq)
{
	ir->avg = 0.0;
	ir->alpha = exp(-1.0 / (float) freq / tcoef);

	return 0;
}

float dsp_getlpf(struct dsp_lpf *ir)
{
	return ir->avg;
}

float dsp_updatelpf(struct dsp_lpf *ir, float v)
{
	ir->avg = ir->alpha * ir->avg + (1 - ir->alpha) * v;

	return ir->avg;
}

int dsp_initpidval(struct dsp_pidval *pv, float kp, float ki, float kd,
	float target)
{
	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;
	pv->pe = 0.0;
	pv->s = 0.0;

	return 0;
}

float dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd)
{
	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;

	return 0;
}

float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt)
{
	float e, v;

	e = target - val;

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd * (e - pv->pe) / dt;

	pv->pe = e;

	return v;
}

float dsp_circpid(struct dsp_pidval *pv, float target, float val, float dt)
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
