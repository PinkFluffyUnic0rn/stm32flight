#include "runvals.h"

struct timev Evs[TEV_COUNT];
struct cdevice Dev[DEV_COUNT];
struct bdevice Flashdev;
struct dsp_lpf Lpf[LPF_COUNT];
struct dsp_compl Cmpl[CMPL_COUNT];
struct dsp_pidblval Pid[PID_COUNT];

struct qmc_data Qmcdata;
struct icm_data Imudata;
struct gnss_data Gnss;
struct crsf_tele Tele;

float Thrust = 0.0;
float Rolltarget = 0.0;
float Pitchtarget = 0.0;
float Yawtarget = 0.0;
float En = 0.0;
enum ALTMODE Altmode = 0;
int Speedpid = 0;
int Yawspeedpid = 0;
int Hovermode = 0;
int Elrs = 0;
int Autopilot = 0;

float Alt0 = 0.0;
float Goffset = 0.0;

struct trackpoint Points[MAX_POINT_COUNT];
int Pointscount = 0;
int Curpoint = 0;
float Autopilottimer = 0.0;

int Curslot = 0;

int Loops = 0;

int Loopscount = 0;

int Elrstimeout = ELRS_TIMEOUT;

int Emergencydisarm = 0;
