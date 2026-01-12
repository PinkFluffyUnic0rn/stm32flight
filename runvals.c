#include "runvals.h"

struct cdevice dev[DEV_COUNT];
struct bdevice flashdev;
struct dsp_lpf lpf[LPF_COUNT];
struct dsp_compl cmpl[CMPL_COUNT];
struct dsp_pidblval pid[PID_COUNT];

struct dsp_pidval yawpv;

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

struct trackpoint points[MAX_POINT_COUNT];
int pointscount;
int curpoint = 0;
float autopilottimer = 0.0;

int curslot = 0;

struct timev evs[TEV_COUNT];

int loops = 0;

int loopscount = 0;

int elrstimeout = ELRS_TIMEOUT;

int emergencydisarm = 0;
