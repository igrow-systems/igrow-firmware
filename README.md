# iGrow Nordic Semiconductor based Firmware #

This README would normally document whatever steps are necessary to get your application up and running.

### What is this repository for? ###

* Quick summary
* Version
* [Learn Markdown](https://bitbucket.org/tutorials/markdowndemo)

### How do I get set up? ###

* Obtain a suitable ARM GCC toolchain `gcc-arm-none-eabi-4_7-2014q2` and set `TOOLCHAIN_PATH` in the `nrf51-pure-gcc` template `Makefile.posix
* Set the location of the Nordic nrf51 SDK in `Makefile.posix` or `Makefile.*` 
* Enter the source firmware source directory, currently pebble_1_1 and invoke GNU Make:

```
make -f gcc/Makefile clean debug
```

or for a release build:
```
make -f gcc/Makefile clean debug
```

* Configuration
* Dependencies
* Database configuration
* How to run tests
* Deployment - upload to the iGrow pebble device by invoking the GNU Make target `flash`.  If it's a blank device or contains unknown test code from a module vendor or similar, then flash the softdevice first with:
```
make -f gcc/Makefile flash-softdevice
```

```
make -f gcc/Makefile flash
```

Start the JLinkGDBServer and attach GDB to the process with:

```
make -f gcc/Makefile startdebug
```

`JLinkGDBServer -device nrf51822 -if swd -speed 1000  -select USB=<serial-number> -port 2331`

`~/toolchain/gcc-arm-none-eabi-4_7-2014q2/bin/arm-none-eabi-gdb ./_build/igrow-nordic_s110.elf`


`target extended-remote localhost:2331`

`monitor reset halt`

### Known good tool versions ###

`arm-none-eabi-gcc (GNU Tools for ARM Embedded Processors 6-2017-q2-update) 6.3.1 20170620 (release) [ARM/embedded-6-branch revision 249437]`

`SEGGER J-Link GDB Server V6.20h Command Line Version`

`JLinkARM.dll V6.20h (DLL compiled Oct 27 2017 16:21:03)`


### Contribution guidelines ###

* Writing tests
* Code review
* Other guidelines

### Who do I talk to? ###

* jsr [at] jsreeve [dot] org

