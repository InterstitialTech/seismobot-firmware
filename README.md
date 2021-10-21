# seismobot-firmware
microcontroller firmware for the Seismobot geophone board

## Dependencies
1. libopencm3
    1. add the following line to ld/devices.data:
```samd09?13* samd ROM=8K RAM=4K```
2. openocd

## Build + flash
```
make
make flash
```
