/**
* @file dsp.h
* @brief All DSP related functions and structures
*/

#ifndef DSP_H
#define DSP_H

/**
* @brief Trim value by range, if it falls out of range, it will be
* set to nearest border value.
* @param v value to trim
* @param l low border of range
* @param u high border of range
* @return Common trimmed value
*/
#define trimf(v, l, u) (((v) < (l)) ? (l) : (((v) > (u)) ? (u) : (v)))

/**
* @brief Trim between 0.0 and 1.0. It it falls out of range, it
* will be set to nearest border value.
* @param v value to trim
* @return Common trimmed value
*/
#define trimuf(v) (((v) < 0.0) ? 0.0 : (((v) > 1.0) ? 1.0 : (v)))

/**
* @brief Check that rotaions value is in [-2Pi;2Pi] range,
* if it falls out of this range, exceess circles (whole 2Pi's)
* will be removed.
* @param v value to check and remove circles
* @return value with removed circles
*/
#define circf(v) (((v) < -M_PI) ? ((v) + 2.0 * M_PI)		\
	: (((v) > M_PI) ? ((v) - 2.0 * M_PI) : (v)))

/**
* @brief Convert degrees to radians
* @param value to convert
* @return converted value
*/
#define deg2rad(v) ((v) / 180.0 * M_PI)

/**
* @brief Reset PID controller's I-term
* @param pid PID controller's context
*/
#define dsp_resetpids(pid) ((pid)->s = 0)

/**
* @brief Reset bilinear PID controller
* @param pid bilinear PID controller's context
*/
#define dsp_resetpidbl(pid) ((pid)->step = 0)

/**
* @brief Crop PID controller's I-term by value
* @param pid PID controller's context
* @param v maximum I-term value
*/
#define dsp_croppids(pid, v) \
	((pid)->s = ((pid)->s > (v)) ? (v) : (pid)->s)

/**
* @brief Low-pass filter's order.
*/
enum DSP_LPFORDER {
	DSP_LPFORDER_0 = 0,	/*!< unity filter */
	DSP_LPFORDER_1 = 1,	/*!< first order */
	DSP_LPFORDER_2 = 2,	/*!< secord order */
	DSP_LPFORDER_3 = 3	/*!< third order */
};

/**
* @brief Low pass filter context it holds lpf's
* alpha and accumulated filter data between calls.
*/
struct dsp_lpf {
	float s1;			/*!< accumulated value */
	float alpha;			/*!< filtering coefficient */
	enum DSP_LPFORDER order;	/*!< filter's order */
};

/**
* @brief PID context it holds PID controller P, I and 
* coefficients and accumelated data between calls.
*/
struct dsp_pidval {
	float kp,		/*!< P term */
	      ki,		/*!< I term */
	      kd;		/*!< D term */
	struct dsp_lpf dlpf;	/*!< D term low-pass filter's context */
	float pe;		/*!< previous error value */
	float s;		/*!< I-term accumulated value */
};

/**
* @brief Bilinear PID context it holds PID controller P, I and D
* coefficients and accumelated data between calls.
*/
struct dsp_pidblval {
	float imax;	/*!< I-term maximum value */
	int circular;	/*!< is controlled value circular */

	float a[3];	/*!< 'a' Z-transform derived coefficients
			without I-term related terms */
	float b[3];	/*!< 'b' Z-transform derived coefficients */
	float ia[3];	/*!< 'a' Z-transform derived coefficients
			for I-term related terms */	
	float e[2];	/*!< previous error values */
	float v[2];	/*!< previous correction values
			  without I-term */
	float iv[2];	/*!< previous I-term correction values */
	float i;	/*!< last I-term value */

	int step;	/*!< calculation step, stops at value of 2 */
	int depth;	/*!< number of 'a' and 'b' Z-transform
			derived coefficient */
};

/**
* @brief Complimentary filter context it holds filter's time
* constant and accumulated filter data between calls.
*/
struct dsp_compl {
	float s;
	float coef;
};

/**
* @brief Set new unity filter using time constant value. Just
* conventional function to use lpf structure for signals, that
* doesn't need low-pass filtering.
*
* @param ir unity filter context.
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setunity(struct dsp_lpf *ir, int init);

/**
* @brief Set new first order low-pass filter using time constant value.
*
* @param ir low-pass filter context.
* @param tcoef low-pass filter's time constant used to calculate
* 	it's alpha coefficient.
* @param freq discretisation frequency used to calculate alpha. In flight
* 	controller application it's the stabilization loop frequency
* 	(see main.c).
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setlpf1t(struct dsp_lpf *ir, float tcoef, int freq, int init);

/**
* @brief Set new first order low-pass filter
* using cut-off frequency value.
*
* @param ir low-pass filter context.
* @param cutoff low-pass filter's cut-off frequency used to calculate
* 	it's alpha coefficient.
* @param freq discretisation frequency used to calculate alpha. In flight
* 	controller application it's the stabilization loop frequency
* 	(see main.c).
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setlpf1f(struct dsp_lpf *ir, float cutoff, int freq, int init);

/**
* @brief Get last calculated low-pass filtering result (from last
* dsp_updatelpf call).
*
* @param ir low-pass filter's context.
* @return last calculated low-pass filtered value
*/
float dsp_getlpf(struct dsp_lpf *ir);

/**
* @brief Calculate next low-pass filter's value and get the result.
*
* @param ir low-pass filter context.
* @param v new value of a signal being filtered.
* @return low-pass filtered value
*/
float dsp_updatelpf(struct dsp_lpf *ir, float v);

/**
* @brief Set new P, I and D coefficient for a PID controller.
*
* @param pv PID controller context.
* @param kp P coefficient.
* @param ki I coefficient.
* @param kd D coefficient.
* @param dcoutoff D low-pass filter cut-off frequency.
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setpid(struct dsp_pidval *pv, float kp, float ki, float kd,
	float dcutoff, int freq, int init);

/**
* @brief Set new P, I and D coefficient for a bilinear PID controller.
*
* @param pv PID controller context.
* @param kp P coefficient.
* @param ki I coefficient.
* @param kd D coefficient.
* @param dcoutoff D low-pass filter cut-off frequency.
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setpidbl(struct dsp_pidblval *pv, float kp, float ki,
	float kd, float imax, float dcutoff, int circular,
	int freq, int init);

/**
* @brief Calculate next PID controller's correction value.
*
* @param pv PID controller context.
* @param target desired value (setpoint).
* @param val current value.
* @param dt time delta, i.e. time passed from previous correction
* 	value calculation.
* @return correction value
*/
float dsp_pid(struct dsp_pidval *pv, float target, float val, float dt);

/**
* @brief Calculate next bilinear PID controller's correction value.
*
* @param pv bilinear PID controller context.
* @param target desired value (setpoint).
* @param val current value.
* @param dt time delta, i.e. time passed from previous correction
* 	value calculation.
* @return correction value
*/
float dsp_pidbl(struct dsp_pidblval *pv, float target, float val);

/**
* @brief Reset bilinear PID controller's I-term
*
* @param pv bilinear PID controller context.
* @return always 0
*/
int dsp_resetpidbls(struct dsp_pidblval *pv);

/**
* @brief Calculate next PID controller's correction value
* 	in circular way: [-Pi:Pi] range is used, correction's
* 	direction is determined by shortest arc from current value to
* 	target. It's used only for yaw axis where full 360 degrees
* 	rotaion is needed.
*
* @param pv PID controller context.
* @param target desired value (setpoint).
* @param val current value.
* @param dt time delta, i.e. time passed from previous correction
* 	value calculation.
* @return correction value
*/
float dsp_circpid(struct dsp_pidval *pv, float target,
	float val, float dt);

/**
* @brief Set new complimentary filter using time constant value.
*
* @param comp complimentary filter's context.
* @param tc complimentary filter's time constant in seconds.
* @param freq discretisation frequency used to calculate filter's.
* @param coefficient. In flight controller application it's the stabilization
* 	loop frequency (see main.c).
* @param init 1, if internal values initializaion required, 0 otherwise.
* @return always 0
*/
int dsp_setcompl(struct dsp_compl *comp, float tc, int freq, int init);

/**
* @brief Get last calculated complimentary filtering result.
* (from last dsp_updatecompl call).
*
* @param comp complimentary filter's context.
* @return last complimentary filtered result
*/
float dsp_getcompl(struct dsp_compl *comp);

/**
* @brief Calculate next complimentary filter's value in circular way
* ([-Pi;Pi] range is used) and get the result.
*
* @param comp complimentary filter's context.
* @param v0 new value of first signal to be filtered and merged.
* @param v1 new value of second signal to be filtered and merged.
* @return complimentary filtered result
*/
float dsp_updatecirccompl(struct dsp_compl *comp, float v0, float v1);

/**
* @brief Calculate next complimentary filter's value and get the result.
*
* @param comp complimentary filter's context.
* @param v0 new value of first signal to be filtered and merged.
* @param v1 new value of second signal to be filtered and merged.
* @return complimentary filtered result
*/
float dsp_updatecompl(struct dsp_compl *comp, float v0, float v1);

#endif
