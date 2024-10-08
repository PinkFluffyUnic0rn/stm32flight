STM32 Flight controller
=========================

Implementing flight controller for a small quadcopter. UAV is controlled
from desktop PC using Xbox (or any else supported by SLD2) gamepad.

Parameters
==========
 * Motors: 1106
 * Props: 4045, ~100mm
 * Battery: 7.4v (2s), 1100 mAh
 * Frame: plywood

Weight
======
 * PCB and ESCs: ~33g
 * Motors: 32g
 * Battery: 46g
 * Frame + Connector: ~56g
 * Total weight: 167g 

Devices
=======
 * MCU: STM32F303CBT6
 * Crystall oscillator: 16 Mhz
 * Control board voltage regulator: AMS1117-3.3
 * Accelerometer/Gyroscope: MPU6050
 * Magnetometer: HMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug: ESP8266 (ESP-01s)
 * Barometer: BMP280

UAV commands:
=============
 
 * `info (mpu | hmc | bmp | values | pid)` -- mpu6050/hmc5883L/bmp280/control values/PID data
 * `r` -- turn off motors
 * `e` -- turn on motors
 * `c [altitude]` -- recalibrate
 * `calib mag (on|off)` -- enter/escape magnetometer calibration mode
 * `pid (tilt|stilt|yaw|syaw|sclimb) (p|i|d) {val}` -- set tilt/tilt speed/yaw/yaw speed/climb speed PID P/I/D value
 * `pid (tilt|yaw) (single/double)` -- switch to single/double PID loop mode for tilt/yaw
 * `compl {val}` -- set complimentary filter's time constant for tilt
 * `lpf (climb|pressure) {val}` -- set low-pass filter's time constant for climb speed/pressure
 * `adj (roll|pitch|yaw) {val}` -- set offset for roll/pitch/yaw (only for dual PID loop mode)
 * `t (p | r | y | c) {val}` -- set pitch/roll/yaw/climb target
