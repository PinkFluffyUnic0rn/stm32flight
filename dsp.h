#ifndef DSP_H
#define DSP_H

#define trimf(v, l, u) (((v) < (l)) ? (l) : (((v) > (u)) ? (u) : (v)))
#define trimuf(v) (((v) < 0.0) ? 0.0 : (((v) > 1.0) ? 1.0 : (v)))
#define deg2rad(v) ((v) / 180.0 * M_PI)

struct dsp_lpf {
	float avg;
	float alpha;
};

struct dsp_pidval {
	float kp, ki, kd;
	float pe;
	float s;
};

struct dsp_compl {
	float s;
	float coef;
};

int dsp_initlpf(struct dsp_lpf *ir, float coef);

float dsp_getlpf(struct dsp_lpf *ir);

float dsp_updatelpf(struct dsp_lpf *ir, float v);

int dsp_initpidval(struct dsp_pidval *pv, float kp, float ki, float kd,
	float target);

float dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd);

float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt);

int dsp_initcompl(struct dsp_compl *comp, float tc, int freq);//float coef);

float dsp_getcompl(struct dsp_compl *comp);

float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1);

#endif
