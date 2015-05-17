#EvvGC-PLUS
**EvvGC-PLUS** is open source software and firmware hobby project for EvvGC brushless gimbal controller v1.x.

**FEATURES of EvvGC-PLUS:**
* EvvGC-PLUS firmware is based on [ChibiOS/RT](http://chibios.org "ChibiOS Homepage") real time operating system (**RTOS**) for easy multitasking.
* ChibiOS/RT hardware abstraction layer (**HAL**) is used for efficient, mostly DMA based control of the STM32 peripherals.
* Quaternion based attitude estimation loop runs at 667 Hz.
* Motor driving efficiency is increased by ~14% using third harmonic injection technique.
* Any orientation of the sensor is possible.
* PID controller is based on motor speed.
* EvvGC-PLUS configurator is written using [Qt 5.4.1](http://qt-io.org "Qt Homepage") technology for cross-platform development.
* [QCustomPlot](http://www.qcustomplot.com "QCustomPlot Homepage") is used for easy plotting and data visualization.
* Dedicated thread is used for serial communications.

**DO NOT DOWNLOAD, BUILD, FLASH OR INSTALL IT IF YOU DON'T KNOW WHAT YOU ARE DOING!!!**
