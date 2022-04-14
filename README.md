# seismobot-firmware

microcontroller firmware for the [Seismobot geophone board](https://github.com/InterstitialTech/seismobot-hardware)

## Dependencies

1. gcc-arm-none-eabi, libnewlib-arm-none-eabi, openocd>=0.10.0
```
sudo apt-get install gcc-arm-none-eabi libnewlib-arm-none-eabi openocd
```

2. libopencm3 @ [83f79e5](https://github.com/libopencm3/libopencm3/tree/83f79e5fdf2de2a63401fd38855c47ef377d470f):
```
git submodule update --init
cd libopencm3
make
```

## Build + flash

To compile, just run `make` from the top level of this repo. To flash the
compiled firmware, connect the geophone board via a
[Segger J-Link EDU Mini](https://www.segger.com/products/debug-probes/j-link/models/j-link-edu-mini/),
and run `make flash`.


