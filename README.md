STM32 Flight controller
=========================

DIY quadcopter flight controller from scratch, two boards and software.

[Project structure](https://github.com/PinkFluffyUnic0rn/stm32flight/wiki/Project-structure) 
is described on wiki page.

Software
=======

Software part implements all basic quadcopter flight control features.
Configuration commands and settings are on
[wiki page](https://github.com/PinkFluffyUnic0rn/stm32flight/wiki/FC-configuration-commands-and-settings).

* Rotation speed stabilization using gyroscope readings.
* Attitude stabilization using accelerometer/gyroscope readings.
* Yaw stabilization using gyroscope readings.
* Yaw stabilization using magnetometer/gyroscope readings.
* Acceleration stabilization using accelerometer readings.
* Altitude stabilization using barometer/accelerometer readings.
* Configuration through Wi-Fi.
* Dshot-300 ESC control protocol.
* IRC Tramp VTX control.
* Telemetry and control through eLRS.
* Terminal configuration tool.
* Qt5 based GUI configuration tool for desktop and android.

STM32F405 based board
=======
 * MCU: STM32F405RGT6
 * Crystall oscillator: 20 Mhz
 * DC-DC converter for control board and camera: TPS5430 (external)
 * Control board voltage regulator: LDL1117S33R-3.3
 * Accelerometer/Gyroscope: ICM-42688-P
 * Magnetometer: QMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug/config: ESP8285
 * Barometer: DPS368

STM32H723 based board
=======
[Schematic](https://github.com/PinkFluffyUnic0rn/stm32flight/blob/main/pcb/H7/schematic.pdf)

 * MCU: STM32F723VGT6
 * Crystall oscillator: 20 Mhz
 * DC-DC converter for control board and camera: TPS5430
 * Control board voltage regulator: LDL1117S33R
 * Accelerometer/Gyroscope: ICM-42688-P
 * Magnetometer: QMC5883L
 * Remote control: ERLS CRSF receiver
 * Telemetry/debug/config: ESP8285
 * Barometer: DPS368

The quadcopter used for testing
==========
 * Motors: 1404, 3800 kV
 * Props: 3030
 * Battery: 14.8v (4s), 1300 mAh, 120c
 * Frame: carbon fiber
