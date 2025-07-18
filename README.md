STM32 Flight controller
=========================

Implementing a flight controller for a small quadcopter. UAV is
controlled through ELRS transmitter.

What's done:
* single roll/pitch PID control loop, stabilization using gyroscope
(accro mode) readings.
* double roll/pitch PID control loop, stabilization using
accelerometer/gyroscope readings.
* yaw stabilization using gyroscope readings.
* yaw stabilization using magnetometer/gyroscope readings.
* acceleration stabilization using accelerometer readings.
* configuration through wi-fi.
* altitude hold.
* dshot-300 ESC control protocol.
* IRC Tramp VTX control.
* telemetry through eLRS.
* terminal configuration tool.
* Qt5 based GUI configuration tool.

Devices
=======
 * MCU: STM32F405RGT6
 * Crystall oscillator: 20 Mhz
 * DC-DC converter for control board and camera: TPS5430
 * Control board voltage regulator: AMS1117-3.3
 * Accelerometer/Gyroscope: ICM-42688-P
 * Magnetometer: QMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug/config: ESP8285
 * Barometer: HP206C

Project structure
=========
* `pcb` -- the directory containing schematics for the flight
controller and BEC:
    * `bottom.png` -- bottom side of flight controller board.
    * `top.png` -- top side of flight controller board.
    * `ground.png` -- ground layer of flight controller board.
    * `power.png` -- power layer of flight controller board.
    * `tps5430.JPG` -- DC-DC conveter used to convert battery voltage to
5 volts used by main board's input LDO and FPV camera.
    * `conponentvalues.txt` -- values and types for components on
schematics.
* `additional` -- additional files
    * `frame.dxf` -- quadcopter frame outline.
* `devices` -- drivers for devices used by the flight controller:
    * `device.h` -- main interface for a character device. Used by almost
all devices in devices directory.
    * `uartconf.c` and `uartconf.h -- driver that receives and sends
data through one of UART's used for configuration and debugging.
    * `bmp280.c` and `bmp280.h` -- driver for a bmp280 barometer
(currenty unused).
    * `crsf.c` and `crsf.h` -- driver for the CRSF protocol used by a
ERLS receiver to interract with the main MCU through UART.
    * `esp8266.c` and `esp8266.h` -- driver used to exchange data with
esp8285 through SPI.
    * `hmc5883l.c` and `hmc5883l.h` -- driver for an HMC5883L
magnetometer (currently unused).
    * `hp206c.c` -- driver for a HP206C barometer.
    * `mpu6500.c` and `mpu6500.h` -- I2C driver for mpu6050 and mpu6500
IMUs (accelerometer + gyroscope).
    * `qmc5883l.c` and `qmc5883l.h` -- driver for an QMC5883L
magnetometer.
    * `icm42688.c` and `icm42688.h` -- driver for an icm42688
IMU (accelerometer + gyroscope).
    * `m10.c` and `m10.h` -- driver for a M10 GNSS module.
    * `w25.c` and `w25.h` -- driver for a w25q SPI flash.
    * `irc.c` and `irc.h` -- driver for an IRC Tramp controlled
VTX device.

* `main.c` -- all routines related to flight control.
* `global.h` -- global values used in many places in source code.
* `crc.h` -- CRC-8 implementation.
* `stm32periph.c` and `stm32periph.h` -- stm32 periphery initialization
and error handling.
* `util.c` and `util.h` -- common utility functions that can be used in
more than one source file.
* `dsp.c` and `dsp.h` -- functions for PID control and data filtering
like low-pass filtering and complimentary filtering.
* `command.c` and `command.h` -- configuration and informational
commands processing API.
* `timev.c` and `timev.h` -- periodic event processing API.
* `uavconf/uavconf.c` -- configuration tool that works through wi-fi AP
created by flight controller using UDP protocol.
* `uavconf_gui` -- gui configuration tool:
    * `main.cpp` -- entry point for GUI configuration tool.
    * `mainwidget.cpp` -- implementation for Qt5 related functions.
    * `mainwidget.h` -- headers for Qt5 related functions.
    * `uavconf.c` and `uavconf.h` -- API for UDP interaction with UAV.
* all other `.c`, `*.h` and `*.s` files are generated by software.

UAV commands
=============

Debug and configuration is performed through AP named `copter` created
by this UAV. Listed commands should be sent using UDP/IP to address
`192.168.3.1`.

 * `r` -- turn off motors
 * `c [altitude]` -- recalibrate and set altitude reference value.
 * `calib mag (on|off)` -- enter/escape magnetometer calibration mode
 * `flash write` -- write settings into MCU's internal flash.
 * `info (mpu | qmc | hp | | dev | values | pid | gnss | ctrl | filter)`
-- get various values and infomation:
    * `mpu` -- ICM-42688-P IMU data.
    * `qmc` -- QMC5883L magnetometer data.
    * `dev` -- all devices status.
    * `values` -- configuration values.
    * `pid` -- PID values.
    * `gnss` -- M10 GNSS data.
    * `ctrl` -- eLRS remote configuration values.
    * `filter` -- complimentary and LPF filters configuration values.
 * `log set {l}` -- set log length to `{l}` records and if `{l}` greater
than `0` start logging. If `{l}` is `0`, stop logging.
 * `log rget {b} {e}` -- get log records whose number starts from `{b}`
and end with `{e} - 1`.
 * `log bget {n1}...{nN}` -- get batch of log records with numbers
`{n1}...{nN}`.
 * `pid (tilt|yaw) mode (single/double)` -- switch to single/double PID
loop mode for tilt/yaw
 * `pid (tilt|stilt|yaw|syaw|throttle/climbrate/altitude) (p|i|d) {val}`
-- set PID controller values:
    * `tilt` -- tilt PID controler P/I/D value.
    * `stilt` -- pitch/roll rotation speed PID controller P/I/D value.
    * `yaw` -- yaw PID controller P/I/D value.
    * `syaw` -- yaw rotation speed PID controller P/I/D value.
    * `throttle` -- acceleration PID controller P/I/D value.
    * `climbrate` -- climb speed PID controller P/I/D value.
    * `altitude` -- altitude PID controller P/I/D value.
 * `compl (attitude/yaw/climbrate/altitude) {val}` -- set complimentary
filters time constants:
    * `attitude` -- filter that mixing accelerometer and gyroscope data
to get pitch/roll value.
    * `yaw` -- filter that mixing gyroscope and magnetometer data to get
yaw value.
    * `climb rate` -- filter that mixing acceleromter interated data and
barometer data to get climb speed value.
    * `altitude` -- filter that mixing climb rate value and barometer
data to get altitude value.
 * `lpf (climb|vaccel/altitude) {val}` -- set low-pass filter's time
constant for climb speed/vertical acceleration/altitude.
 * `adj (rollthrust/pitchthrust) {val}` -- adjust motors thrust.
 * `adj (roll|pitch|yaw) {val}` -- set offset for roll/pitch/yaw
(only for dual PID loop mode).
 * `adj acc (x|y|z) {val}` -- set x/y/z offset for acceleromter
 * `adj gyro (x|y|z) {val}` -- set x/y/z offset for gyroscope
 * `adj mag (x0|y0|z0|xscale|yscale|zscale|decl) {val}` -- set x/y/z
offset, x/y/z scale or magnetic declination for magnetometer 
 * `ctrl (roll|pitch) {val}` -- set maximum roll/pitch tilt value
in radians.
 * `ctrl (syaw|yaw) {val}` -- set yaw rotation speed for single/double
loop mode in radians.
 * `ctrl accel {val}` -- set maximum acceleration for throttle loop.
 * `ctrl climbrate {val}` -- set maximum climb rate in m/s.
 * `ctrl altmax {val}` -- set maximum altitude in meters.

Log format
==========

Log records got by commands `log rget` and `log bget` are just lines
with set of space separated numeric strings and have such format:

```
    {CRC} {number} {v0} ... {v31}

```
, where `{CRC}` is CRC-8 sum of record line, `{number}` -- record
number and `{v0} ... {v31}` -- 31 record values.

Log records sampled 64 times per second, so it is possible to get
exact time from log start for each record, by it's number. For example,
record with number `1234` was sampled at `45763 / 64 = 715.04` seconds
from logging start, or, at 11 minutes and 55.04 seconds.

Record values store various values that have meaning during flight. They
are listed below:

| number |value                           |
|--------|--------------------------------|
|0       |accelerometer scaled raw X value|
|1       |accelerometer scaled raw Y value|
|2       |accelerometer scaled raw Z value|
|3       |gyroscope scaled raw X value    |
|4       |gyroscope scaled raw Y value    |
|5       |gyroscope scaled raw Z value    |
|6       |magnetometer raw X value        |
|7       |magnetometer raw Y value        |
|8       |magnetometer raw Z value        |
|9       |barometer temperature           |
|10      |barometer altitude              |
|11      |filtered accelerometer X value  |
|12      |filtered accelerometer Y value  |
|13      |filtered accelerometer Z value  |
|14      |calculated roll value           |
|15      |calculated pitch value          |
|16      |calculated yaw value            |
|17      |calculated climb rate           |
|18      |calculated altitude             |
|19      |left-forward motor's thrust     |
|20      |left-backward motor's thrust    |
|21      |right-forward motor's thrust    |
|22      |right-backward motor's thrust   |
|23      |eLRS channel 0 value            |
|24      |eLRS channel 1 value            |
|25      |eLRS channel 2 value            |
|26      |eLRS channel 3 value            |
|27      |eLRS channel 4 value            |
|28      |eLRS channel 5 value            |
|29      |eLRS channel 6 value            |
|30      |eLRS channel 7 value            |
|31      |battery voltage                 |

Quadcopter parameters
==========
 * Motors: 1404, 3800 kV
 * Props: 3030
 * Battery: 14.8v (4s), 1300 mAh, 120c
 * Frame: carbon fiber

Quadcopter Weight
======
 * PCB and camera: 20g
 * ESC: 30g
 * Motors + props: 48g
 * Battery: 148g
 * BEC: ~6g
 * Antenna: 10g
 * Frame: ~114g
 * Total weight: 376g
