
NAME ?= rustbox
RISCV_GNU_TOOLCHAIN ?= $(RISCV_PATH)
CARGO_OPTS ?= ""
BUILD_ENV ?= debug

clean:
	cargo clean


build:
	cargo build


release:
	cargo build --release

upload-console:
	cargo build
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex target/riscv32imac-unknown-none-elf/debug/console target/riscv32imac-unknown-none-elf/debug/console.hex
	scripts/upload --hex target/riscv32imac-unknown-none-elf/$(BUILD_ENV)/console.hex --jlink JLinkExe

upload-vgaterm:
	cargo build
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex target/riscv32imac-unknown-none-elf/debug/vgaterm target/riscv32imac-unknown-none-elf/debug/vgaterm.hex
	scripts/upload --hex target/riscv32imac-unknown-none-elf/$(BUILD_ENV)/vgaterm.hex --jlink JLinkExe


%.hex:
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex $* $*.hex


upload:
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex target/riscv32imac-unknown-none-elf/debug/$(NAME) target/riscv32imac-unknown-none-elf/debug/$(NAME).hex
	scripts/upload --hex target/riscv32imac-unknown-none-elf/debug/$(NAME).hex --jlink JLinkExe

screen:
	sudo screen /dev/ttyACM0 115200