STM32 Flight controller
=========================

Implementing flight controller for a small quadcopter with coreless 8520
dc motors.

Parameters
==========
 * Motors: 8520
 * Props: 65mm

Weight
======
 * PCB: ~33g
 * Motors: 20g
 * Battery: 12g (400mAh), 27g (1800 mAh)
 * Frame: ~15 (plywood)
 * Total weight: ~80g (400 mAh battery), ~100g (1800 mAh battery)

Devices
=======
 * MCU: STM32F411CEU6
 * Accelerometer/Gyroscope: MPU6050
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

control commands:
--------------
 * r -- turn off motors
 * e -- turn on motors
 * sl -- switch to single PID loop mode (speed controlled)
 * dl -- switch to dobule PID loop (with stabilization)
 * c [altitude] -- recalibrate

value commands:
--------------
 * `t (i | d | s {val})` -- increase/decrease/set thrust
 * `(p | r | y)t (i | d| s {val})` -- increase/decrease/set pitch/roll/yaw target
 * `(x | y | z)c (i | d | s {val})` -- increase/decrease X/Y/Z accelerometer correction
 * `(p | i | d) (i | d | s {val})` -- increase/decrease/set position pid P/I/D value
 * `s(p | i | d) (i | d | s {val})` -- increase/decrease/set speed pid P/I/D value
 * `y(p | i | d) (i | d | s {val})` -- increase/decrease/set yaw pid P/I/D value
