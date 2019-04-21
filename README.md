# K64F I2C Accelerometer

This project offers a test implementation of an I2C master communication with an I2C Slave (Accelerometer).

## The MCU
This project uses an NXP Kinetis K64F. However, it can easily be adapted to any other NXP micro-controller using CMSIS drivers 

## The Accelerometer
The project uses NXP FXOS8700 as I2C slave. However, it can be adapted to recognize any kind of I2C accelerometer by adapting the registers that are requested by the drivers. 