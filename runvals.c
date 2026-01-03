#include "runvals.h"

struct cdevice dev[DEV_COUNT];

struct bdevice flashdev;

struct dsp_lpf batlpf;
struct dsp_lpf currlpf;

struct dsp_lpf avgthrustlpf;

struct dsp_lpf valpf;
struct dsp_lpf tlpf;
struct dsp_lpf vtlpf;
struct dsp_lpf volpf;
struct dsp_lpf flpf;

struct dsp_lpf altlpf;
struct dsp_lpf templpf;

struct dsp_lpf atemppt1;

struct dsp_lpf accxpt1;
struct dsp_lpf accypt1;
struct dsp_lpf acczpt1;

struct dsp_lpf gyroxpt1;
struct dsp_lpf gyroypt1;
struct dsp_lpf gyrozpt1;

struct dsp_lpf magxpt1;
struct dsp_lpf magypt1;
struct dsp_lpf magzpt1;

struct dsp_compl pitchcompl;
struct dsp_compl rollcompl;
struct dsp_compl yawcompl;

struct dsp_compl climbratecompl;
struct dsp_compl altcompl;

struct dsp_pidblval pitchpv;
struct dsp_pidblval rollpv;
struct dsp_pidblval pitchspv;
struct dsp_pidblval rollspv;
struct dsp_pidval yawpv;
struct dsp_pidblval yawspv;
struct dsp_pidblval tpv;
struct dsp_pidblval cpv;
struct dsp_pidblval apv;

struct qmc_data qmcdata;
struct gnss_data gnss;
struct crsf_tele tele;

float thrust = 0.0;
float rolltarget = 0.0;
float pitchtarget = 0.0;
float yawtarget = 0.0;
float ltm = 1.0;
float lbm = 1.0;
float rtm = 1.0;
float rbm = 1.0;
float en = 0.0;
enum ALTMODE altmode = 0;
int speedpid = 0;
int yawspeedpid = 0;
int hovermode = 0;
int elrs = 0;
int autopilot = 0;

float alt0 = 0.0;
float goffset = 0.0;
float forwardspeed = 0.0;
float forwardpath = 0.0;
float faoffset = 0.0;

struct appoint points[MAX_POINT_COUNT];
int pointscount;
int curpoint = 0;
float autopilottimer = 0.0;

int curslot = 0;

struct timev evs[TEV_COUNT];

int loops = 0;

int loopscount = 0;

int elrstimeout = ELRS_TIMEOUT;

int emergencydisarm = 0;
