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
* altitude hold (kind of, yet tested only within 2.5 meters range)

Devices
=======
 * MCU: STM32F405GRT6
 * Crystall oscillator: 20 Mhz
 * DC-DC converter for control board and camera: TPS5430
 * Control board voltage regulator: AMS1117-3.3
 * Accelerometer/Gyroscope: ICM42688P
 * Magnetometer: QMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug: ESP8266 (ESP-07)
 * Barometer: HP206C

Project structure
=========
* `additional` -- the directory containing schematics for the flight
controller and BEC:
    * `side1.png` -- first side of flight controller board that is
faced down.
    * `side2.png` -- second side of flight controller board that is
faced up.
    * `tps5430.JPG` -- DC-DC conveter used to convert battery voltage to
5 volts used by main board's input LDO and FPV camera.
* `devices` -- drivers for devices used by the flight controller:
    * `device.h` -- main interface for a character device. Used by almost
all devices in devices directory.
    * `bmp280.c` and `bmp280.h` -- driver for a bmp280 barometer
(currenty unused).
    * `crsf.c` and `crsf.h` -- driver for the CRSF protocol used by a
ERLS receiver to interract with the main MCU through UART.
    * `esp8266.c` and `esp8266.h` -- driver that process AT command used
to interact with an esp8266 (esp-07 board to be precise) throught UART.
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

* `main.c` -- all routines related to flight control.
* `dsp.c` and `dsp.h` -- functions for PID control and data filtering
like low-pass filtering and complimentary filtering.
* `uartdebug.c` and `uartdebug.h` -- uartprintf function for debugging.
* all other files are generated by software.
* `rc/rc.c` -- configuration tool that works through wi-fi connection
created by flight controller.

UAV commands
=============

Debug and configuration is performed through AP named `copter` created
by this UAV. Listed commands should be sent using UDP/IP to address
`192.168.3.1`.

 * `info (mpu | qmc | hp | values | pid)` -- mpu6050/qmc5883L/hp206c/control
values/PID data
 * `r` -- turn off motors
 * `c [altitude]` -- recalibrate
 * `calib mag (on|off)` -- enter/escape magnetometer calibration mode
 * `pid (tilt|stilt|yaw|syaw|sclimb) (p|i|d) {val}` -- set tilt/tilt
speed/yaw/yaw speed/climb speed PID P/I/D value
 * `pid (tilt|yaw) (single/double)` -- switch to single/double PID loop
mode for tilt/yaw
 * `compl {val}` -- set complimentary filter's time constant for tilt
 * `lpf (climb|pressure) {val}` -- set low-pass filter's time constant
for climb speed/pressure
 * `adj (roll|pitch|yaw) {val}` -- set offset for roll/pitch/yaw
(only for dual PID loop mode)
 * `adj mag (x0|y0|z0|xscale|yscale|zscale|decl) {val}` -- set x/y/z
offset, x/y/z scale or magnetic declination for magnetometer 

Quadcopter parameters
==========
 * Motors: 1204, 5000 kV
 * Props: 3016
 * Battery: 15.4v (4s), 1100 mAh, 60c
 * Frame: plywood

Quadcopter Weight
======
 * PCB and ESCs: ~57g
 * Motors + props: 27g
 * Battery: 88g
 * Frame + wires + antenna: ~105g
 * Total weight: 277g 
