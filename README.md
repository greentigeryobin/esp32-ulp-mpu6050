# ULP

Reading MPU6050 using ULP Coprocessor in ESP32. This uses HULP liberary from [here](https://github.com/boarchuz/HULP "here").

## Features

* Using espidf framework it reads MPU6050 using ULP.
* Wakes the device if any of the accelerometer value crosses a defined threshold, the device is will wake and display the value.

## Getting started

In menuconfig change `RTC slow memory reserved for coprocessor` to `8192`.
