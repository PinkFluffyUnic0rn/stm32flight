#include "dsp.h"

int dsp_initlpf(struct dsp_lpf *ir, float coef)
{
	ir->avg = 0.0;
	ir->alpha = coef;

	return 0;
}

float dsp_getlpf(struct dsp_lpf *ir)
{
	return ir->avg;
}

float dsp_updatelpf(struct dsp_lpf *ir, float v)
{
	ir->avg = ir->alpha * v + (1 - ir->alpha) * ir->avg;

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

int dsp_initcompl(struct dsp_compl *comp, float coef)
{
	comp->s = 0;
	comp->coef = coef;

	return 0;
}

float dsp_getcompl(struct dsp_compl *comp)
{
	return comp->s;
}

float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1)
{
	comp->s = comp->coef * (comp->s + v0) + (1 - comp->coef) * v1;

	return comp->s;
}
