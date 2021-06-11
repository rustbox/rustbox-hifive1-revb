
NAME = rustbox-hifive1-revb
RISCV_GNU_TOOLCHAIN ?= $(RISCV_PATH)
CARGO_OPTS ?= ""

clean:
	cargo clean


build:
	cargo build


release:
	cargo build --release


%.hex:
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex $* $*.hex


upload:
	$(RISCV_GNU_TOOLCHAIN)/bin/riscv64-unknown-elf-objcopy -O ihex target/riscv32imac-unknown-none-elf/debug/$(NAME) target/riscv32imac-unknown-none-elf/debug/$(NAME).hex
	scripts/upload --hex target/riscv32imac-unknown-none-elf/debug/$(NAME).hex --jlink JLinkExe

