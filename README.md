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
5. Setup RISC-V qemu (https://risc-v-getting-started-guide.readthedocs.io/en/latest/linux-qemu.html)
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

## Cargo setup

From http://osblog.stephenmarz.com/ch0.html

1. `rustup override set nightly` in directory
2. `rustup target add riscv32imac-unknown-none-elf`
3. Trying (from https://github.com/riscv-rust/riscv-rust-quickstart):
   1. `cargo install cargo-generate`
   2. `cargo generate --git https://github.com/riscv-rust/riscv-rust-quickstart`
    > A note, `qemu-system-riscv32 -nographic -machine sifive_e -kernel path/to/program.elf`

Using the quickstart above, we have generated this current project layout.

I modified the .cargo/config to be specific to this project, and to run emulation on hifive revb board.

## QEMU Setup

I downloaded and installed QEMU 6.0.0 from source from https://www.qemu.org/download/#source. This contains the specific hifive board that this project is currently targeted at. Once qemu 6.0.0 is in your path, you can emulate code using:
    `qemu-system-riscv32 -nographic -machine sifive_e,revb=true -m 128M -drive if=none,format=raw,file=hdd.dsk,id=foo -serial 'mon:stdio' -bios none -kernel path/to/code`
This is what is setup in the cargo config currently, so when `cargo run` is invoked, this qemu emulation is used.

## hdd.dsk

Generate with `fallocate -x -l 32M hdd.dsk`

## Differences from Stephen Marz's Blog

Apparently the way I configured and built my qemu 6.0.0 may not be as full featured as what is presented in the blog. When I added the various `-device` options that are present in his blog in chapter 0, qemu could not find the devices properly giving:

> `No 'virtio-bus' bus found for device 'virtio-keyboard-device'`

And etc. So I removed them for now at least so the src/main.rs or the hello_world examples will compile and run.

# Running

`cargo run`

or

`cargo run --example hello_world`

Issue `CTRL-A`, `x` to break out of the qemu emulation.
