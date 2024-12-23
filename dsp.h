#ifndef DSP_H
#define DSP_H

// trim value by range, if it falls out of range, it will be
// set to nearest border value.
#define trimf(v, l, u) (((v) < (l)) ? (l) : (((v) > (u)) ? (u) : (v)))

// trim between 0.0 and 1.0. It it falls out of range, it
// will be set to nearest border value.
#define trimuf(v) (((v) < 0.0) ? 0.0 : (((v) > 1.0) ? 1.0 : (v)))

// check that rotaions value is in [-2Pi:2Pi] range, if it falls out
// of this range, exceess cirles (whole 2Pi's) will be removed.
#define circf(v) (((v) < -M_PI) ? ((v) + 2.0 * M_PI)		\
	: (((v) > M_PI) ? ((v) - 2.0 * M_PI) : (v)))

// convert degrees to radians
#define deg2rad(v) ((v) / 180.0 * M_PI)

// low pas filter context
// it holds lpf's alpha and
// accumulated filter data between calls
struct dsp_lpf {
	float avg;
	float alpha;
};

// PID context
// it holds PID controller P, I and D coefficients and
// accumelated data between calls.
struct dsp_pidval {
	float kp, ki, kd;
	float pe;
	float s;
};

// complimentary filter context
// it holds filter's time constant and
// accumulated filter data between calls.
struct dsp_compl {
	float s;
	float coef;
};

// initilize low-pass filter.
//
// ir -- low-pass filter context.
// tcoef -- time low-pass filter constant used to calculate
// 	it's alpha coefficient.
// freq -- discretisation frequency used to calculate alpha. In flight
// 	controller application it's the stabilization loop frequency
// 	(see main.c).
int dsp_initlpf(struct dsp_lpf *ir, float tcoef, int freq);

// get last calculated low-pass filtering result (from last 
// dsp_updatelpf call).
//
// ir -- low-pass filter's context.
float dsp_getlpf(struct dsp_lpf *ir);

// calculate next low-pass filter's value and get the result.
//
// ir -- low-pass filter context.
// v -- new value of a signal being filtered.
float dsp_updatelpf(struct dsp_lpf *ir, float v);

// initilize PID controller.
//
// pv -- PID controller context.
// kp -- P coefficient.
// ki -- I coefficient.
// kd -- D coefficient.
// target -- initial target value (unsed, should be removed).
int dsp_initpidval(struct dsp_pidval *pv, float kp, float ki, float kd,
	float target);

// set new P, I and D coefficient for a PID controller.
//
// pv -- PID controller context.
// kp -- P coefficient.
// ki -- I coefficient.
// kd -- D coefficient.
float dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd);

// calculate next PID controller's correction value.
//
// pv -- PID controller context.
// target -- desired value (setpoint).
// val -- current value.
// dt -- time delta, i.e. time passed from previous correction
// 	value calculation.
float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt);

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
float dsp_circpid(struct dsp_pidval *pv, float target, float val, float dt);

// initilize complimentary filter.
//
// comp -- complimentary filter's context.
// tc -- complimentary filter's time constant in seconds.
// freq -- discretisation frequency used to calculate filter's.
// coefficient. In flight controller application it's the stabilization
// 	loop frequency (see main.c).
int dsp_initcompl(struct dsp_compl *comp, float tc, int freq);

// get last calculated complimentary filtering result.
// (from last dsp_updatecompl call).
//
// comp -- complimentary filter's context.
float dsp_getcompl(struct dsp_compl *comp);

// unused, should be removed.
float dsp_updatecirccompl(struct dsp_compl *comp, float v0, float v1);

// calculate next complimentary filter's value and get the result.
//
// comp -- complimentary filter's context.
// v0 -- new value of first signal to be filtered and merged.
// v1 -- new value of second signal to be filtered and merged.
float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1);

#endif
