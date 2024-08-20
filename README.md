STM32 Flight controller
=========================

Implementing flight controller for a small quadcopter.

Parameters
==========
 * Motors: 1106
 * Props: 4045, ~100mm
 * Battery: 7.4v (2s), 850 mAh
 * Frame: plywood

Weight
======
 * PCB and ESCs: ~32g
 * Motors: 32g
 * Battery: 36g
 * Frame: ~30g
 * Wires: ~30g
 * Total weight: 162g 

Devices
=======
 * MCU: STM32F411CEU6
 * Accelerometer/Gyroscope: MPU6050
 * Magnetometer: HMC5883L
 * Remote control: ESP8266 (ESP-01s)
 * Barometer: BMP280

UAV commands:
=============
info commands:
--------------
 * md -- mpu6050 data
 * bd -- bmp280 data
 * hd -- hmc5883L data
 * vd -- control values
 * pd -- PID values

control commands:
--------------
 * r -- turn off motors
 * e -- turn on motors
 * c [altitude] -- recalibrate

value commands:
--------------
 * `pid (tilt|stilt|yaw|syaw|sclimb) (p|i|d) {val}` -- set tilt/tilt speed/yaw/yaw speed/climb speed PID P/I/D value
 * `pid (tilt|yaw) (single/double)` -- switch to single/double PID loop mode for tilt/yaw
 * `compl {val}` -- set complimentary filter's time constant for tilt
 * `lpf (climb|pressure) {val}` -- set low-pass filter's time constant for climb speed/pressure
 * `adj (roll|pitch|yaw) {val}` -- set offset for roll/pitch/yaw (only for dual PID loop mode)
 * `t (p | r | y | c) {val}` -- set pitch/roll/yaw/climb target
