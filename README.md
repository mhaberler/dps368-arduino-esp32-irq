# DPS368: interrupt-driven driver example for arduino-esp32

I found the [DPS368](https://www.infineon.com/cms/en/product/sensor/pressure-sensors/pressure-sensors-for-iot/dps368/) devices are great pressure sensors

There is a [support library for Arduino](https://github.com/Infineon/arduino-xensiv-dps3xx) by Infineon which includes an [interrupt-driven example](https://github.com/Infineon/arduino-xensiv-dps3xx/blob/master/examples/i2c_interrupt/i2c_interrupt.ino)

Unfortunately, this does not work on the popular [ESP32 Arduino](https://github.com/espressif/arduino-esp32) platform due to its limitations - the Infineon code assumes one can do I2Coperations in an interrupt context bringing down the house on arduino-esp32 which just crashes.

I had the need for precise pressure measurement (long conversion time), precise timestamps, and no blocking wait in the driver.

This  example driver works around the problem and works fine with  arduino-esp32.

# hardware setup

- ESP32: M5Stack-CoreS3 (tensilica) and M5Stamp C3U (RISC-V)
- [DPS368 Pressure Shield2Go](https://www.mouser.at/datasheet/2/196/Infineon_DPS368_Shield2Go_Quick_Start_Guide_GS_v01-3131997.pdf) breakout board
- I2C connection via "red port" on CoreS3
- IRQ line connected to "blue port" pin17
- Jumper J2 soldered to connect SDO and IRQ pin, and a pulldown resistor - this also sets device address to 0x76

# software setup

Build system was PlatformIO.

The [dps368_probe()](https://github.com/mhaberler/dps368-arduino-esp32-irq/blob/main/irqtest/dps3xxprobe.cpp) function can be used to verify that

- the device is responding
- initialisation works
- interrupts are detected
- the number of interrupts seen makes sense

# multiple device support

the [multiple-device branch](https://github.com/mhaberler/dps368-arduino-esp32-irq/blob/multiple-devices/src/main.cpp) contains a more advanced example for:
- interrupt-driven operation of multiple DPS3xx sensors on any bus
- supports arbitrary number of sensors
- individual per-sensor configuration
