#include <math.h>

#include "dsp.h"

// initilize low-pass filter.
//
// ir -- low-pass filter context.
// tcoef -- time low-pass filter constant used to calculate
// 	it's alpha coefficient.
// freq -- discretisation frequency used to calculate alpha. In flight
// 	controller application it's the stabilization loop frequency
// 	(see main.c).
int dsp_initlpf(struct dsp_lpf *ir, float tcoef, int freq)
{
	ir->avg = 0.0;
	ir->alpha = exp(-1.0 / (float) freq / tcoef);

	return 0;
}

// get last calculated low-pass filtering result (from last 
// dsp_updatelpf call).
//
// ir -- low-pass filter's context.
float dsp_getlpf(struct dsp_lpf *ir)
{
	return ir->avg;
}

// calculate next low-pass filter's value and get the result.
//
// ir -- low-pass filter context.
// v -- new value of a signal being filtered.
float dsp_updatelpf(struct dsp_lpf *ir, float v)
{
	ir->avg = ir->alpha * ir->avg + (1 - ir->alpha) * v;

	return ir->avg;
}

// initilize PID controller.
//
// pv -- PID controller context.
// kp -- P coefficient.
// ki -- I coefficient.
// kd -- D coefficient.
// target -- initial target value (unsed, should be removed).
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

// set new P, I and D coefficient for a PID controller.
//
// pv -- PID controller context.
// kp -- P coefficient.
// ki -- I coefficient.
// kd -- D coefficient.
float dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd)
{
	pv->kp = kp;
	pv->ki = ki;
	pv->kd = kd;

	return 0;
}

// calculate next PID controller's correction value.
//
// pv -- PID controller context.
// target -- desired value (setpoint).
// val -- current value.
// dt -- time delta, i.e. time passed from previous correction
// 	value calculation.
float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt)
{
	float e, v;

	e = target - val;

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd * (e - pv->pe) / dt;

	pv->pe = e;

	return v;
}

// calculate next PID controller's correction value
// 	in circular way: [-2Pi:2Pi] range is used, correction's
// 	direction is determined by shortest arc from current value to
// 	target. It's used only for yaw axis where full 360 degrees
// 	rotaion is needed.
//
// pv -- PID controller context.
// target -- desired value (setpoint).
// val -- current value.
// dt -- time delta, i.e. time passed from previous correction
// 	value calculation.
float dsp_circpid(struct dsp_pidval *pv, float target, float val, float dt)
{
	float e, v;

	e = circf(target - val);

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd * (e - pv->pe) / dt;

	pv->pe = e;

	return v;
}

// initilize complimentary filter.
//
// comp -- complimentary filter's context.
// tc -- complimentary filter's time constant in seconds.
// freq -- discretisation frequency used to calculate filter's.
// coefficient. In flight controller application it's the stabilization
// 	loop frequency (see main.c).
int dsp_initcompl(struct dsp_compl *comp, float tc, int freq)
{
	comp->s = 0;
	comp->coef = tc / (tc + 1.0 / (float) freq);

	return 0;
}

// get last calculated complimentary filtering result.
// (from last dsp_updatecompl call).
//
// comp -- complimentary filter's context.
float dsp_getcompl(struct dsp_compl *comp)
{
	return comp->s;
}

// calculate next complimentary filter's value and get the result.
//
// comp -- complimentary filter's context.
// v0 -- new value of first signal to be filtered and merged.
// v1 -- new value of second signal to be filtered and merged.
float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1)
{
	comp->s = comp->coef * (comp->s + v0) + (1.0 - comp->coef) * v1;

	return comp->s;
}

// calculate next complimentary filter's value in circular way
// ([-2Pi:2Pi] range is used) and get the result.
//
// comp -- complimentary filter's context.
// v0 -- new value of first signal to be filtered and merged.
// v1 -- new value of second signal to be filtered and merged.
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
