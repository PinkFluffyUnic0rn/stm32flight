STM32 Flight controller
=========================

Implementing flight controller for a small quadcopter.

Parameters
==========
 * Motors: 1106
 * Props: 4045, ~100mm

Weight
======
 * PCB and ESCs: ~30g
 * Motors: 32g
 * Battery: 28g (1800 mAh)
 * Frame: ~30g (plywood)
 * Wires: ~30g
 * Total weight: 150g 

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
 * `tc (i | d | s {val})` -- increate/decrease/set complementary filter's time constant
