SHELL := /bin/bash
DC ?=
PROGRAM ?= proximity_warning
TARGET ?= yamuna
DEBUG ?=
MARCH ?= rv32imac
MABI  ?= ilp32
XLEN  ?= 32
FLAGS ?=

SDK_PATH ?= /home/priya/gc2025/GCSDK
bspinc   ?= $(SDK_PATH)/bsp/include
bspdri   ?= $(SDK_PATH)/bsp/drivers
bspboard ?= $(SDK_PATH)/bsp/board/$(TARGET)
GENLIB   ?= $(SDK_PATH)/software/examples/gen_lib

# Drivers used by the application
DRIVER_SRCS = $(wildcard $(bspdri)/gpio/gpio.c) \
              $(wildcard $(bspdri)/gpt/gpt.c) \
              $(wildcard $(bspdri)/platform/platform.c) \
              $(wildcard $(bspdri)/uart/uart.c)

objname = $(addprefix output/, $(patsubst %.c,%.o,$(subst /,_,$(1:$(bspdri)/%=%))))
DRIVER_OBJS = $(foreach src,$(DRIVER_SRCS),$(call objname,$(src)))

all: create_dir
	make $(PROGRAM).riscv

$(PROGRAM).riscv: create_dir $(DRIVER_OBJS)
	@riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) \
		-I$(bspinc) -I$(bspdri) -I$(bspboard) \
		-c ./$(PROGRAM).c -o ./output/$(PROGRAM).o -march=$(MARCH) -mabi=$(MABI) -lm -lgcc
	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) \
		-T $(bspboard)/link.ld \
		./output/$(PROGRAM).o $(DRIVER_OBJS) $(GENLIB)/gen_lib/libshakti$(XLEN).a \
		-o ./output/$(PROGRAM).shakti -static -nostartfiles -lm -lgcc
	@riscv$(XLEN)-unknown-elf-objdump -D ./output/$(PROGRAM).shakti > ./output/$(PROGRAM).dump

define DRIVER_RULE
$(call objname,$(1)): $(1)
	@riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) \
		-I$(bspinc) -I$(bspdri) -I$(bspboard) -c $$< -o $$@ -march=$(MARCH) -mabi=$(MABI)
endef
$(foreach src,$(DRIVER_SRCS),$(eval $(call DRIVER_RULE,$(src))))

create_dir:
	@mkdir -p ./output

clean:
	rm -rf ./output
