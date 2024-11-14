STM32 Flight controller
=========================

Implementing flight controller for a small quadcopter. UAV is controlled
through ELRS transmitter.

Parameters
==========
 * Motors: 1204
 * Props: 3016
 * Battery: 7.4v (2s), 1300 mAh
 * Frame: plywood

Weight
======
 * PCB and ESCs: ~57g
 * Motors + props: 27g
 * Battery: 64g
 * Frame + wires: ~59g
 * Total weight: 207g 

Devices
=======
 * MCU: STM32F303CBT6
 * Crystall oscillator: 16 Mhz
 * DC-DC converter for control board and camera: TPS5430
 * Control board voltage regulator: AMS1117-3.3
 * Accelerometer/Gyroscope: MPU6050
 * Magnetometer: QMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug: ESP8266 (ESP-01s)
 * Barometer: HP206C

UAV commands:
=============

Debug and configuration is performed through AP named `copter` created
by this UAV. Listed commands should be sent using UDP/IP to address
`192.168.3.1`.

 * `info (mpu | qmc | hp | values | pid)` -- mpu6050/qmc5883L/hp206c/control values/PID data
 * `r` -- turn off motors
 * `e` -- turn on motors
 * `c [altitude]` -- recalibrate
 * `calib mag (on|off)` -- enter/escape magnetometer calibration mode
 * `pid (tilt|stilt|yaw|syaw|sclimb) (p|i|d) {val}` -- set tilt/tilt speed/yaw/yaw speed/climb speed PID P/I/D value
 * `pid (tilt|yaw) (single/double)` -- switch to single/double PID loop mode for tilt/yaw
 * `compl {val}` -- set complimentary filter's time constant for tilt
 * `lpf (climb|pressure) {val}` -- set low-pass filter's time constant for climb speed/pressure
 * `adj (roll|pitch|yaw) {val}` -- set offset for roll/pitch/yaw (only for dual PID loop mode)
 * `adj mag (x0|y0|z0|xscale|yscale|zscale|decl) {val}` -- set x/y/z offset, x/y/z scale or magnetic declination for magnetometer 
