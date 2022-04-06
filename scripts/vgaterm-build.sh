#!/bin/bash

cargo build --release
make asm-vgaterm
riscv64-unknown-elf-objcopy -O ihex target/riscv32imac-unknown-none-elf/release/vgaterm target/riscv32imac-unknown-none-elf/release/vgaterm.hex
scripts/upload --hex target/riscv32imac-unknown-none-elf/release/vgaterm.hex --jlink JLinkExe