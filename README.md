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
 * vd -- control values
 * pd -- PID values
 * hd -- hmc5883L values

control commands:
--------------
 * r -- turn off motors
 * e -- turn on motors
 * sl -- switch to single PID loop mode (speed controlled) for roll/picth
 * dl -- switch to dobule PID loop (with stabilization) for roll/picth
 * ysl -- switch to single PID loop mode (speed controlled) for yaw 
 * ydl -- switch to dobule PID loop (with stabilization) for yaw
 * c [altitude] -- recalibrate

value commands:
--------------
 * `tt {val})` -- set thrust target
 * `(p | r | y)t {val}` -- set pitch/roll/yaw target
 * `(x | y | z)c {val}` -- set X/Y/Z orienation correction
 * `(p | i | d) {val}` -- set position pid P/I/D value
 * `s(p | i | d) {val}` -- set speed pid P/I/D value
 * `y(p | i | d) {val}` -- set yaw pid P/I/D value
 * `ys(p | i | d) {val}` -- set yaw speed pid P/I/D value
 * `t(p | i | d) {val}` -- set thrust P/I/D value
 * `tc {val}` -- set roll/pitch complementary filter's time constant
 * `ptc {val}` -- set pressure low-pass filter's time constant
 * `ztc {val}` -- set Z-axis acceleration low-pass filter's time constant
