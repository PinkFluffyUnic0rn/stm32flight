#include <math.h>

#include "mcudef.h"

#include "dsp.h"

ITCMRAM double trimf(double v, double l, double u)
{
	return ((v < l) ? l : ((v > u) ? u : v));
}

ITCMRAM double trimuf(double v)
{
	return ((v < 0.0) ? 0.0 : ((v > 1.0) ? 1.0 : v));
}

ITCMRAM float trimuff(float v)
{
	return ((v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v));
}

ITCMRAM double deg2rad(double v)
{
	return (v / 180.0 * M_PI);
}

ITCMRAM double circf(double v)
{
	return ((v < -M_PI) ? (v + 2.0 * M_PI)
		: ((v > M_PI) ? (v - 2.0 * M_PI) : v));
}

ITCMRAM int dsp_setunity(struct dsp_lpf *ir, int init)
{
	if (init)
		ir->s1 = 0.0;

	ir->alpha = 0.0;
	ir->order = DSP_LPFORDER_0;

	return 0;
}

ITCMRAM int dsp_setlpf1t(struct dsp_lpf *ir, double tcoef,
	int freq, int init)
{
	if (init)
		ir->s1 = 0.0;

	ir->alpha = exp(-1.0 / (double) freq / tcoef);
	ir->order = DSP_LPFORDER_1;

	return 0;
}

ITCMRAM int dsp_setlpf1f(struct dsp_lpf *ir, double cutoff,
	int freq, int init)
{
	double v;

	if (init)
		ir->s1 = 0.0;

	v = 2.0 * M_PI * cutoff / (double) freq;

	ir->alpha = 1.0 - v / (v + 1.0);
	ir->order = DSP_LPFORDER_1;

	return 0;
}

ITCMRAM double dsp_getlpf(struct dsp_lpf *ir)
{
	return ir->s1;
}

ITCMRAM double dsp_updatelpf(struct dsp_lpf *ir, double v)
{
	if (ir->order == DSP_LPFORDER_0)
		ir->s1 = v;
	else
		ir->s1 = ir->alpha * ir->s1 + (1 - ir->alpha) * v;

	return ir->s1;
}

ITCMRAM int dsp_setnotch(struct dsp_biquad *bqd, double rejfreq,
	double bw, int freq, int init)
{
	double w, a, a0inv;
	double cw;

	if (init)
		bqd->step = 0;

	w = 2.0 * M_PI * rejfreq / (double) freq;
	a = sin(w) / (2.0 * bw);
	cw = cos(w);
	a0inv = 1 / (1 + a);

	bqd->b[0] = 1.0 * a0inv;
	bqd->b[1] = -2.0 * cw * a0inv;
	bqd->b[2] = 1 * a0inv;

	bqd->a[0] = 1.0;
	bqd->a[1] = bqd->b[1];
	bqd->a[2] = (1 - a) * a0inv;

	bqd->depth = 2;

	return 0;
}

ITCMRAM int dsp_updatebiquadv(struct dsp_biquad *bqd,
	double x, double y, double z,
	double *xo, double *yo, double *zo)
{
	double sx, sy, sz;

	sx = bqd->b[0] * x;
	sy = bqd->b[0] * y;
	sz = bqd->b[0] * z;

	if (bqd->step > 0) {
		sx += bqd->b[1] * bqd->vx[0] - bqd->a[1] * bqd->sx[0];
		sy += bqd->b[1] * bqd->vy[0] - bqd->a[1] * bqd->sy[0];
		sz += bqd->b[1] * bqd->vz[0] - bqd->a[1] * bqd->sz[0];
	}
	
	if (bqd->step > 1) {
		sx += bqd->b[2] * bqd->vx[1] - bqd->a[2] * bqd->sx[1];
		sy += bqd->b[2] * bqd->vy[1] - bqd->a[2] * bqd->sy[1];
		sz += bqd->b[2] * bqd->vz[1] - bqd->a[2] * bqd->sz[1];
	}

	bqd->vx[2] = bqd->vx[1];
	bqd->vx[1] = bqd->vx[0];
	bqd->vx[0] = x;
	bqd->sx[2] = bqd->sx[1];
	bqd->sx[1] = bqd->sx[0];
	bqd->sx[0] = sx;

	bqd->vy[2] = bqd->vy[1];
	bqd->vy[1] = bqd->vy[0];
	bqd->vy[0] = y;
	bqd->sy[2] = bqd->sy[1];
	bqd->sy[1] = bqd->sy[0];
	bqd->sy[0] = sy;

	bqd->vz[2] = bqd->vz[1];
	bqd->vz[1] = bqd->vz[0];
	bqd->vz[0] = z;
	bqd->sz[2] = bqd->sz[1];
	bqd->sz[1] = bqd->sz[0];
	bqd->sz[0] = sz;

	if (bqd->step < bqd->depth)
		++bqd->step;

	*xo = sx;
	*yo = sy;
	*zo = sz;

	return 0;
}

ITCMRAM int dsp_setpid(struct dsp_pidval *pv, double kp, double ki,
	double kd, double dcutoff, int freq, int init)
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

ITCMRAM int dsp_setpidbl(struct dsp_pidblval *pv, double kp, double ki,
	double kd, double imax, double dcutoff, int circ,
	int freq, int init)
{
	double ts, tf, tt, c;

	pv->step = 0;
	pv->depth = 2;

	pv->imax = imax;
	pv->circular = circ;

	ts = 1.0 / (double) freq;
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

ITCMRAM double dsp_pid(struct dsp_pidval *pv, double target,
	double val, double dt)
{
	double e, v;

	e = target - val;

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd
		* dsp_updatelpf(&(pv->dlpf), (e - pv->pe) / dt);

	pv->pe = e;

	return v;
}

ITCMRAM double dsp_pdbl(struct dsp_pidblval *pv,
	double target, double val)
{
	double e, v;

	e = target - val;

	if (pv->circular)
		e = circf(e);

	switch (pv->step) {
	case 0:
		v = pv->a[0] * e;
		break;

	case 1:
		v = pv->a[0] * e
			+ pv->a[1] * pv->e[0] + pv->b[1] * pv->v[0];
		break;

	default:
		v = pv->a[0] * e
			+ pv->a[1] * pv->e[0] + pv->b[1] * pv->v[0]
			+ pv->a[2] * pv->e[1] + pv->b[2] * pv->v[1];
		break;
	}

	pv->e[1] = pv->e[0];
	pv->v[1] = pv->v[0];
	pv->e[0] = e;
	pv->v[0] = v;

	if (pv->step < pv->depth)
		++pv->step;

	return v;
}

ITCMRAM double dsp_pidbl(struct dsp_pidblval *pv,
	double target, double val)
{
	double e, v, vs;

	e = target - val;

	if (pv->circular)
		e = circf(e);

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

ITCMRAM int dsp_resetpidbls(struct dsp_pidblval *pv)
{
	pv->iv[0] = pv->iv[1] = pv->iv[2] = pv->i = 0.0;

	return 0;
}

ITCMRAM double dsp_circpid(struct dsp_pidval *pv, double target,
	double val, double dt)
{
	double e, v;

	e = circf(target - val);

	pv->s += e * dt;

	v = pv->kp * e + pv->ki * pv->s + pv->kd * (e - pv->pe) / dt;

	pv->pe = e;

	return v;
}

int dsp_setcompl(struct dsp_compl *comp, double tc, int freq, int init)
{
	if (init)
		comp->s = 0;

	comp->coef = tc / (tc + 1.0 / (double) freq);

	return 0;
}

ITCMRAM double dsp_getcompl(struct dsp_compl *comp)
{
	return comp->s;
}

ITCMRAM double dsp_updatecompl(struct dsp_compl *comp, double v0,
	double v1)
{
	comp->s = comp->coef * (comp->s + v0) + (1.0 - comp->coef) * v1;

	return comp->s;
}

ITCMRAM double dsp_updatecirccompl(struct dsp_compl *comp, double v0,
	double v1)
{
	double s;

	s = circf(comp->s + v0);

	if (fabs(s - v1) > M_PI) {
		if (s < 0)	s += 2.0 * M_PI;
		if (v1 < 0)	v1 += 2.0 * M_PI;
	}

	comp->s = circf(comp->coef * s + (1.0 - comp->coef) * v1);

	return comp->s;
}

ITCMRAM int dsp_setcomplv2(struct dsp_complv *comp, double tc,
	int freq, int init)
{
	if (init)
		comp->r = comp->p = comp->y = 0;

	comp->coef = tc / (tc + 1.0 / (double) freq);

	return 0;
}

ITCMRAM int dsp_getcomplv2(struct dsp_complv *comp, double *r,
	double *p)
{
	*r = comp->r;
	*p = comp->p;

	return 0;
}

ITCMRAM int dsp_updatecomplv2(struct dsp_complv *comp,
	double r0, double p0, double y0,
	double r1, double p1, double y1,
	double *ro, double *po, double *yo)
{
	double ra[3][3];
	double rg[3][3];
	double r[3][3];
	double cr, sr, cp, sp, cy, sy;
	double rollg, pitchg, yawg;

	cr = cos(comp->r);	sr = sin(comp->r);
	cp = cos(comp->p);	sp = sin(comp->p);
	cy = cos(comp->y);	sy = sin(comp->y);

	ra[0][0] = cp * cy;
	ra[0][1] = sp * sr * sy - sy * cr;
	ra[0][2] = sp * cr * cy + sr * sy;

	ra[1][0] = sy * cp;
	ra[1][1] = sp * sr * sy + cr * cy;
	ra[1][2] = sp * sy * cr - sr * cy;

	ra[2][0] = -sp;
	ra[2][1] = sr * cp;
	ra[2][2] = cp * cr;

	sr = sin(r0);	cr = cos(r0);
	sp = sin(p0);	cp = cos(p0);
	sy = sin(y0);	cy = cos(y0);

	rg[0][0] = cp * cy;
	rg[0][1] = sp * sr * sy - sy * cr;
	rg[0][2] = sp * cr * cy + sr * sy;

	rg[1][0] = sy * cp;
	rg[1][1] = sp * sr * sy + cr * cy;
	rg[1][2] = sp * sy * cr - sr * cy;

	rg[2][0] = -sp;
	rg[2][1] = sr * cp;
	rg[2][2] = cp * cr;

	r[0][0] = ra[0][0] * rg[0][0] + ra[0][1] * rg[1][0] + ra[0][2] * rg[2][0];
	r[0][1] = ra[0][0] * rg[0][1] + ra[0][1] * rg[1][1] + ra[0][2] * rg[2][1];
	r[0][2] = ra[0][0] * rg[0][2] + ra[0][1] * rg[1][2] + ra[0][2] * rg[2][2];

	r[1][0] = ra[1][0] * rg[0][0] + ra[1][1] * rg[1][0] + ra[1][2] * rg[2][0];
	r[1][1] = ra[1][0] * rg[0][1] + ra[1][1] * rg[1][1] + ra[1][2] * rg[2][1];
	r[1][2] = ra[1][0] * rg[0][2] + ra[1][1] * rg[1][2] + ra[1][2] * rg[2][2];

	r[2][0] = ra[2][0] * rg[0][0] + ra[2][1] * rg[1][0] + ra[2][2] * rg[2][0];
	r[2][1] = ra[2][0] * rg[0][1] + ra[2][1] * rg[1][1] + ra[2][2] * rg[2][1];
	r[2][2] = ra[2][0] * rg[0][2] + ra[2][1] * rg[1][2] + ra[2][2] * rg[2][2];

	pitchg = atan2(-r[2][0],
		sqrt(r[2][1] * r[2][1] + r[2][2] * r[2][2]));
	rollg = atan2(r[2][1], r[2][2]);
	yawg = circf(atan2(r[1][0], r[0][0]));

	if (fabs(yawg - y1) > M_PI) {
		if (yawg < 0)	yawg += 2.0 * M_PI;
		if (y1 < 0)	y1 += 2.0 * M_PI;
	}

	comp->r = comp->coef * rollg + (1.0 - comp->coef) * r1;
	comp->p = comp->coef * pitchg + (1.0 - comp->coef) * p1;
	comp->y = circf(comp->coef * yawg + (1.0 - comp->coef) * y1);

	*ro = comp->r;
	*po = comp->p;
	*yo = comp->y;

	return 0;
}
