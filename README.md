# stm32f4-bare-metal

Bare metal STM32F4 examples to serve as starting points for projets. Educational purposes.

Some of the STM32F4xx family based processor headers are added in the `include` folder to get 
register locations. CMSIS library is added for a general support. No extra HAL libraries is used except the selected projects described below.

Common startup functions are moved to `include/system_stm32f4xx.c` to include in all projects.

## Installation

- Clone the project using `git clone --recurse-submodules https://github.com/fcayci/stm32f4-bare-metal`
with the included external tools/libraries such as CMSIS repo and any additional libraries some of the projects use.
- You will need a **toolchain** (for compiler and binutils). Get one free from - [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
- For my fork of this repository I am building with CygWin tools, ST-LINK, OpenOCD [OpenOCD-20181130], and gcc compiler chain 4_8-2013q4-major/arm-none-eabi [https://launchpad.net/gcc-arm-embedded/4.8/4.8-2013-q4-major]
- *For windows only*, you will need **make** tool. You can just get the make tool from [gnuwin32](http://gnuwin32.sourceforge.net/packages/make.htm). Alternatively you can install the minimalist GNU tools for windows from [mingw](https://mingw-w64.org/) and [MSYS](https://www.msys2.org/)
- For the **programmer/debugger**, you can use - [stlink](https://github.com/texane/stlink) or [OpenOCD](http://openocd.org/). Though only stlink utility support is added.
- You can use your favorite code editor to view/edit the contents. Here is an open source one: [Visual Studio Code](https://code.visualstudio.com/).

## Compile

[makefile](projects/armf4.mk) contains the necessary build scripts and compiler flags.

Browse into any directory and run `make` to compile.

```
cd projects/blinky
make
```

If everything is setup correctly, you should see the completed message.
```
Cleaning blinky
Building blinky.c
   text    data     bss     dec     hex filename
    368	      0	      0	    368	    170	blinky.elf
Successfully finished...
```

If you see any errors about command not found, make sure the toolchain binaries are in your `PATH`. On Windows check the *Environment Variables* for your account. On Linux/macOS run `echo $PATH` to verify your installation.

## Program

Run `make flash' (not 'make burn') to program the chip. See the modification in the file projects/armf4.mk for this enhancement to support programming the board using ST-LINK.
```
...
.. Flash written and verified! jolly good!
```

Install the [ST LINK](https://www.st.com/en/development-tools/st-link-v2.html) drivers if you cannot see your board when `make flash` is run.

## Disassemble

Run `make disass` / `make disass-all` to disassamble.

## Debug

In order to debug your code, connect your board to the PC, run `st-util` (comes with stlink utility) from one terminal, and from another terminal within the project directory run `make debug`. You can then use general **gdb** commands to browse through the code.

Alternatively, you can install *Cortex-Debug* plug-in from marus25 on Visual Studio Code, and debug using the VSCode interface. No need for additional terminals. An example launch script is given with the code.

## Projects

* [blinky](projects/blinky/) - Good old blink LEDs example
* [clock](projects/clock/) - Shows how to change clock frequencies on the fly
* [math](projects/math/) - A simple sine function to test math library operation
* [systick](projects/systick/) - Blinks LEDs using systick timer. Processor clock is set to max (168 Mhz)
* [timer](projects/timer/) - Blinks LEDs one at a time using the Timer module and Timer interrupt
* [pwm](projects/pwm/) - Fades an LED using pwm functionality using Timer module
* [extint](projects/extint/) - External interrupt example using the on-board push-button
* [usb-vcp](projects/usb-vcp/) - USB Virtual COM Port implementation example. It depends on the [libopencm3](https://github.com/libopencm3/libopencm3) library for the USB stack
* [dac](projects/dac/) - On-chip digital to analog converter operation
* [dac_with_timer](projects/dac_with_timer/) - On-chip digital to analog converter operation with timer trigger
* [uart](projects/uart/) - UART example to show how to send data over
* [uart_tx_int](projects/uart_tx_int/) - UART example with tx interrupt
* [spi](projects/spi/) - SPI example that is customized for on-board motion sensor (lis302dl)
* [wwdg](projects/wwdg/) - Window Watchdog example
* [itm](projects/itm/) - Message sending through CoreSight ITM port 0. Install [OpenOCD](http://openocd.org/) to capture the message
* [dma](projects/dma/) - Example DMA transfer using memory-to-memory mode

## C++ Projects

These projects demonstrate simple implementations of various C++ constructs executing on a a bare metal STM32F407-DISC board. They are all C++ variations of the 'C' blinky project described above. Each C++ project builds upon the previous C++ projects to demonstrate increasingly more complex embedded C++ techniques.

* [cpp](projects/cpp/) - C++ version of the blinky project. Used to demonstrate C++ compilation.
* [cpp2](projects/cpp2/) - Multi-file C++ version of the blinky project. Used to demonstrate linking with two source files.
* [cpp3](projects/cpp3/) - Mixed C/C++ version of the blinky project. Used to demonstrate calling/linking across C/C++ APIs. Calls objects are static (no use of new/delete).
* [cpp4](projects/cpp4/) - Mixed C/C++ version of the blinky project. Use new/delete to dynamically instantiate and destroy class objects.

