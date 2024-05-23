STM32 VFS
===========

Simple VFS for SMT32 MCU and FS for spi flash devices


Structure
---------

 * `main.c` &mdash; a simple terminal program to demonstrate and test
vfs functions.
 * `uartterm.c` and `uartterm.h` &mdash; API and functions that
implement unix-like ternimal working through UART.
 * `driver.h` &mdash; common interface for drivers in this project.
 * `filesystem.c` and `filesystem.h` &mdash; common interface for
filesystems in this project.
 * `w25.c` and `w25.h` &mdash; driver that implement most basic function
of W25Q SPI flash memory.
 * `sfs.c` and `sfs.h` &mdash; A simple filesystem.
 * `rfs.c` and `rfs.h` &mdash; Filesystem that resides in RAM.
 * `call.c` and `call.h` &mdash; Implementation for system call not
related to VFS.


What's done
-----------
 * Filesystem mounting/unmounting.
 * open/close and read/write calls in VFS.
 * Simple filesystem, bad version of ext with built-in checksums.
 * Naive implementations of malloc/free/realloc.
 * RAM filesystem and that malloc/free/realloc calls.
 * Common interfaces for filesystems and drivers.


What's not done
---------------
 * Device files
 * Page cache
 * Separtion of character and block devices
 * Unit tests
# stm32flight
