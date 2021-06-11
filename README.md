# OS

This OS is for the RISCV processor on the SiFive "HiFive1 Rev B", which has the "Freedom E310" processor implementing `RV32IMAC` ISA

From https://www.sifive.com/boards/hifive1-rev-b and https://www.sifive.com/chip-designer#fe310


# Setting Up

> NOTE: I did not end up using the following setup for the currently existing state of the project!

1. `git clone --recursive https://github.com/sifive/freedom-e-sdk.git`
2. Download "GNU Embedded Toolchain — v2020.12.8" and "OpenOCD — v2020.12.1" from https://www.sifive.com/software
   1. Unzip in a known directory
3. `export RISCV_OPENOCD_PATH=</path/to/OpenOCD>` in freedom-e-sdk directory
4. `export RISCV_PATH=</path/to/toolchain>` in freedom-e-sdk

## QEMU
Setup RISC-V qemu (https://risc-v-getting-started-guide.readthedocs.io/en/latest/linux-qemu.html)
   
1. `mkdir riscv64-linux`
2. `cd riscv64-linux`
3. `git clone https://github.com/qemu/qemu`
4. `cd qemu`
5. `git checkout v5.0.0`
6. `./configure --target-list=riscv64-softmmu`
7. `make -j 4`
8. `sudo make install`
9. `./configure --target-list=riscv32-softmmu`
10. `make -j 4`
11. `sudo make install`

## Prerequisites

1. Download and unzip [RISC-V GNU Toolchain](https://static.dev.sifive.com/dev-tools/riscv64-unknown-elf-gcc-8.1.0-2019.01.0-x86_64-linux-ubuntu14.tar.gz)

   > Set environment variable for `RISCV_PATH` to unzipped directly

   > `ls $RISCV_PATH/bin` should show many gnu tool executables targeted for RISC-V

2. Download and install the latest version of the [SEGGER JLink tool](https://www.segger.com/downloads/jlink/) (bottom of the page).

   > This is what we will use to connect to the hifive board

## Cargo setup

From http://osblog.stephenmarz.com/ch0.html

1. `rustup override set nightly` in directory
2. `rustup target add riscv32imac-unknown-none-elf`
3. Trying (from https://github.com/riscv-rust/riscv-rust-quickstart):
   1. `cargo install cargo-generate`
   2. `cargo generate --git https://github.com/riscv-rust/riscv-rust-quickstart`
    > A note, `qemu-system-riscv32 -nographic -machine sifive_e -kernel path/to/program.elf`

Using the quickstart above, we have generated this current project layout.

I modified the .cargo/config to  run emulation on hifive revb board.

## QEMU Setup

I downloaded and installed QEMU 6.0.0 from source from https://www.qemu.org/download/#source. This contains the specific hifive board that this project is currently targeted at. Once qemu 6.0.0 is in your path, you can emulate code using:
    `qemu-system-riscv32 -nographic -machine sifive_e,revb=true -m 128M -drive if=none,format=raw,file=hdd.dsk,id=foo -serial 'mon:stdio' -bios none -kernel path/to/code`
This is what is setup in the cargo config currently, so when `cargo run` is invoked, this qemu emulation is used.

## Hifive1 Revb Board Setup

Follow the instructions in the Hifive1 ["Getting Started"](https://sifive.cdn.prismic.io/sifive/cf239fd0-ae4f-4fd8-a944-fdafb5018153_hifive1b-getting-started-guide_v1.2.pdf) to setup the board.

1. Plugging in the board with a USB to a computer will power on the device, with the preloaded bootloader and sifive welcome program. The host computer should recognize that a USB device was just plugged in. 

   Additionally, with `ls -l /dev/tty*` you should see at least two tty devices named `/dev/ttyACM0` (with some suffix integer value)
2. Follow the instructions on page 14 of the the above "Getting Started" guide to modify the jlink udev rules adding the "plugdev" group.
3. Add your user to the plugdev group:

   ```
   sudo usermod -a -G plugdev <user_name>
   ```

   > This will add your user to the "plugdev" group, giving you non-sudo access to the hifive tty which is how you can view and interact with device in a terminal emulator.
4. Use terminal emulator like GNU Screen (suggested from the "Getting Started" guide) to interface with the hifive1 board:

   ```
    sudo screen /dev/ttyACM0 115200
   ```

   Resetting the device at this point should show the built-in SiFive Welcome program.

## hdd.dsk

Generate with `fallocate -x -l 32M hdd.dsk`

## Differences from Stephen Marz's Blog

Apparently the way I configured and built my qemu 6.0.0 may not be as full featured as what is presented in the blog. When I added the various `-device` options that are present in his blog in chapter 0, qemu could not find the devices properly giving:

> `No 'virtio-bus' bus found for device 'virtio-keyboard-device'`

And etc. So I removed them for now at least so the src/main.rs or the hello_world examples will compile and run.

## Acknowledgements and inspiration for setup and implementation
1. https://github.com/tock/tock/blob/master/boards/hifive1/README.md
2. https://github.com/sifive/freedom-e-sdk.git
3. http://osblog.stephenmarz.com/ch0.html
4. https://github.com/riscv-rust/riscv-rust-quickstart

# Running

`cargo run`

or

`cargo run --example hello_world`

Press `CTRL-A`, `x` to break out of the qemu emulation.

> By default code is run in the qemu emulator.

# Upload Code to Hifive1 Revb Board

Programs are uploaded using the `scripts/upload` bash script. It takes a parameter for the program with `--hex <path>` and the name of the JLink executable used to connect to the board with `--jlink JLinkExe` (JLinkExe should be in your path from step 2 of Prerequisites above).

The board requires hex versions of executables rather than direct elf executables. We perform this conversion with the `$RISCV_PATH/bin/riscv64-unknown-elf-objcopy` tool, and this is made easier in the Makefile `.hex` target. This hex file is what is ultimately uploaded into the device.

> Ensure all prerequisites are done in order to upload programs to the board
> The environment variable `RISCV_PATH` should be set in the environmnet (see Prerequisites above)

To build and upload `main` to the board, use the provided makefile:

```
make build upload
```

## Upload an Example

There are a few examples that can be emulated with:

```
cargo run --example <example name>
```

But to run these on the board required a couple steps. In the following steps we will use the `color_type` example program:

1. `cargo build --example color_type`
2. `make target/riscv32imac-unknown-none-elf/debug/examples/color_type.hex`
3. `./scripts/upload --hex target/riscv32imac-unknown-none-elf/debug/examples/color_type.hex --jlink JLinkExe`
