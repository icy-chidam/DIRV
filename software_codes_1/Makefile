#Created based on intruder_detection Makefile
#   Copyright (C) 2019  IIT Madras. All rights reserved.

SHELL := /bin/bash
DC ?=
PROGRAM ?= proximity_warning
TARGET ?= yamuna
DEBUG ?=
MARCH ?= rv32imac
MABI  ?= ilp32
XLEN  ?= 32
FLAGS ?=

# Adjust paths to your SDK layout
SDK_PATH ?= /home/priya/gc2025/GCSDK
bspinc   ?= $(SDK_PATH)/bsp/include
bspdri   ?= $(SDK_PATH)/bsp/drivers
bspboard ?= $(SDK_PATH)/bsp/board/$(TARGET)
GENLIB   ?= $(SDK_PATH)/software/examples/gen_lib

# FatFs paths – include all .c files from the directory
FATFS_PATH ?= /home/priya/gc2025/hw/boot-code-yamuna/sdcard
FATFS_INC  ?= $(FATFS_PATH)
FATFS_SRCS := $(wildcard $(FATFS_PATH)/*.c)

# Explicitly list the required driver source files (core SSPI and pinmux)
SSPI_CORE   = $(wildcard $(bspdri)/sspi/sspi_driver.c) \
              $(wildcard $(bspdri)/sspi/sspi.c) \
              $(wildcard $(bspdri)/sspi/sspi_common.c)
PINMUX_SRC  = $(wildcard $(bspdri)/pinmux/pinmux.c)
UTILS_SRC   = $(wildcard $(bspdri)/utils/utils.c)   # if needed

# Combine all driver sources (including FatFs)
DRIVER_SRCS = $(SSPI_CORE) $(PINMUX_SRC) $(UTILS_SRC) $(FATFS_SRCS)

# Generate unique object file names for files under bspdri
objname = $(addprefix output/, $(patsubst %.c,%.o,$(subst /,_,$(1:$(bspdri)/%=%))))

# For FatFs files (they are not under bspdri), use simple basename
DRIVER_OBJS = $(foreach src,$(SSPI_CORE) $(PINMUX_SRC) $(UTILS_SRC),$(call objname,$(src))) \
              $(addprefix output/, $(notdir $(FATFS_SRCS:.c=.o)))

# Extra flags for SD card driver to define SSPI0 and include BSP headers
CFLAGS_SD = -DSSPI0=sspi_instance[0] -I$(bspinc)

all: create_dir
	make $(PROGRAM).riscv

$(PROGRAM).riscv: create_dir $(DRIVER_OBJS)
	@riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) \
		-I$(bspinc) -I$(bspdri) -I$(bspboard) -I$(FATFS_PATH) -I$(FATFS_PATH)/.. \
		-c ./$(PROGRAM).c -o ./output/$(PROGRAM).o -march=$(MARCH) -mabi=$(MABI) -lm -lgcc
	@riscv$(XLEN)-unknown-elf-gcc -march=$(MARCH) -mabi=$(MABI) \
		-T $(bspboard)/link.ld \
		-Wl,--allow-multiple-definition \
		./output/$(PROGRAM).o $(DRIVER_OBJS) $(GENLIB)/gen_lib/libshakti$(XLEN).a \
		-o ./output/$(PROGRAM).shakti -static -nostartfiles -lm -lgcc
	@riscv$(XLEN)-unknown-elf-objdump -D ./output/$(PROGRAM).shakti > ./output/$(PROGRAM).dump

# Compile each SSPI/pinmux driver source (under bspdri)
define DRIVER_RULE
$(call objname,$(1)): $(1)
	@riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) \
		-I$(bspinc) -I$(bspdri) -I$(bspboard) -I$(FATFS_PATH) -I$(FATFS_PATH)/.. -c $$< -o $$@ -march=$(MARCH) -mabi=$(MABI)
endef
$(foreach src,$(SSPI_CORE) $(PINMUX_SRC) $(UTILS_SRC),$(eval $(call DRIVER_RULE,$(src))))

# Compile FatFs source files (including sdcard driver) with extra flags for SSPI0
output/%.o: $(FATFS_PATH)/%.c
	@riscv$(XLEN)-unknown-elf-gcc -w $(DC) -mcmodel=medany -static -std=gnu99 -fno-builtin-printf $(FLAGS) \
		-I$(bspinc) -I$(bspdri) -I$(bspboard) -I$(FATFS_PATH) -I$(FATFS_PATH)/.. $(CFLAGS_SD) -c $< -o $@ -march=$(MARCH) -mabi=$(MABI)

create_dir:
	@mkdir -p ./output

clean:
	rm -rf ./output
