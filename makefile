# Hey Emacs, this is a -*- makefile -*-
#
# Navi-Control Makefile
# by ARTI for the Mikrokopter Project www.mikrokopter.de
#
# Updated to include support for compiling in GPS.c from V0.15c
# by AnnihilaT
#
# based on
#
# WinARM template makefile
# by Martin Thomas, Kaiserslautern, Germany
# <eversmith@heizung-thomas.de>
#
# based on the WinAVR makefile written by Eric B. Weddington, J?g Wunsch, et al.
# Released to the Public Domain
# Please read the make user manual!
#
#
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# To rebuild project do "make clean" then "make all".
#
# Requires:
# - CodeSourcery Toolchain (tested & working with Lite version under Windows & Linux)
# - syscalls.c : Dummy implementation for some syscalls for the newlib libc replacement
# - GPS.h and GPS.h to be renamed gps.c and gps.h, adapt all occurences of includes in .c files to lower case letters
# - startup912.s and startup_generic.s to be renamed to .S with capital "S" to avoid deletion when calling "make clean"
# - Changes in file: scripts/flash_str9.ld
#       replace:        ./obj/startup912.o (.text) /* Startup code */
#       to:             ./startup912.o (.text)  /* Startup code */
#       or else change it to where this file is relative to current dir.
# - NaviCtrl V0.17e: Requires following Dummy Implementation to be added to GPS.c:
#   (or)
# - NaviCtrl V0.17f: Does not require the lines below to be added
#
#       u8 GPS_CalculateDeviation(GPS_Pos_t * pCurrentPos, GPS_Pos_t * pTargetPos, GPS_Deviation_t* pDeviationFromTarget)
#       {
#        return;
#       }
#

#
# Comments:
# Tested with KUbuntu and CodeSourcery Toolchain by Arti
# Tested with Fedora Core 8 & Windows XP with CodeSourcery Lite Toolchain by Annihilat
# *** for V0.17f of the firmware, please contact AnnihilaT on the mikrokopter.de forums. ***
#
# Changelog:
# - 13. Oct. 2009 initial version (arti)
# - 08. Mar. 2010 updated version with support for windows, FC8, and 0.15c gps.c (annihilat)

# Toolchain prefix (i.e arm-elf -> arm-elf-gcc.exe)
#TCHAIN = arm-elf
TCHAIN = arm-none-eabi

#USE_THUMB_MODE = YES
USE_THUMB_MODE = NO

#-------------------------------------------------------------------
VERSION_MAJOR = 0
VERSION_MINOR = 17
VERSION_PATCH = 4
#-------------------------------------------------------------------

# MCU name and submodel
MCU = arm9e
HEX_NAME = STR9

# Target file name (without extension).
TARGET = Navi-Ctrl_STR9_V0_24bzak

# List C source files here. (C dependencies are automatically generated.)
# use file-extension c for "c-only"-files
#SRC = $(TARGET).c

# List C source files here which must be compiled in ARM-Mode.
# use file-extension c for "c-only"-files
#SRCARM = vectors.c

SRCARM = ./libstr91x/src/91x_adc.c
SRCARM += ./libstr91x/src/91x_it.c
SRCARM += ./libstr91x/src/91x_ahbapb.c
SRCARM += ./libstr91x/src/91x_can.c
SRCARM += ./libstr91x/src/91x_dma.c
SRCARM += ./libstr91x/src/91x_emi.c
SRCARM += ./libstr91x/src/91x_fmi.c
SRCARM += ./libstr91x/src/91x_gpio.c
SRCARM += ./libstr91x/src/91x_i2c.c
SRCARM += ./libstr91x/src/91x_lib.c
SRCARM += ./libstr91x/src/91x_mc.c
SRCARM += ./libstr91x/src/91x_rtc.c
SRCARM += ./libstr91x/src/91x_scu.c
SRCARM += ./libstr91x/src/91x_ssp.c
SRCARM += ./libstr91x/src/91x_tim.c
SRCARM += ./libstr91x/src/91x_uart.c
SRCARM += ./libstr91x/src/91x_vic.c
SRCARM += ./libstr91x/src/91x_wdg.c
SRCARM += ./libstr91x/src/91x_wiu.c
SRCARM += ./usblibrary/src/usb_core.c
SRCARM += ./usblibrary/src/usb_init.c
SRCARM += ./usblibrary/src/usb_int.c
SRCARM += ./usblibrary/src/usb_mem.c
SRCARM += ./usblibrary/src/usb_regs.c
SRCARM += analog.c
SRCARM += main.c
SRCARM += mkprotocol.c
SRCARM += params.c
SRCARM += syscalls.c
SRCARM += crc16.c
SRCARM += fat16.c
SRCARM += fifo.c
SRCARM += gps.c
SRCARM += gpx.c
SRCARM += i2c.c
SRCARM += kml.c
SRCARM += led.c
SRCARM += logging.c
SRCARM += menu.c
SRCARM += printf_P.c
SRCARM += ramfunc.c
SRCARM += sdc.c
SRCARM += settings.c
SRCARM += spi_slave.c
SRCARM += ssc.c
SRCARM += timer1.c
SRCARM += timer2.c
SRCARM += uart0.c
SRCARM += uart1.c
SRCARM += uart2.c
SRCARM += ubx.c
SRCARM += compass.c
SRCARM += debug.c
SRCARM += eeprom.c
SRCARM += mk3mag.c
SRCARM += mymath.c
SRCARM += ncmag.c
SRCARM += usb.c
SRCARM += usb_desc.c
SRCARM += usb_endp.c
SRCARM += usb_istr.c
SRCARM += usb_prop.c
SRCARM += usb_pwr.c
SRCARM += waypoints.c
SRCARM += buffer.c

# thumb is possible too for vectors.c - keep ARM, TODO: profile

# List C++ source files here.
# use file-extension cpp for C++-files (use extension .cpp)
CPPSRC =

# List C++ source files here which must be compiled in ARM-Mode.
# use file-extension cpp for C++-files (use extension .cpp)
#CPPSRCARM = $(TARGET).cpp
CPPSRCARM =

# List Assembler source files here.
# Make them always end in a capital .S. Files ending in a lowercase .s
# will not be considered source files but generated files (assembler
# output from the compiler), and will be deleted upon "make clean"!
# Even though the DOS/Win* filesystem matches both .s and .S the same,
# it will preserve the spelling of the filenames, and gcc itself does
# care about how the name is spelled on its git://github.com/zakwiggy/navi.gitcommand-line.
ASRC =

# List Assembler source files here which must be assembled in ARM-Mode..
#ASRCARM = startup.S vector.S
ASRCARM = startup912.S

# Path to Linker-Scripts
LINKERSCRIPTPATH = ./scripts

## Output format. (can be ihex or binary or both)
## (binary i.e. for openocd and SAM-BA, hex i.e. for lpc21isp and uVision)
#FORMAT = ihex
#FORMAT = binary
#FORMAT = both
FORMAT = ihex

# Optimization level, can be [0, 1, 2, 3, s].
# 0 = turn off optimization. s = optimize for size.
# (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = 2
#OPT = 0

# Debugging format.
# Native formats for AVR-GCC's -g are stabs [default], or dwarf-2.
# AVR (extended) COFF requires stabs, plus an avr-objcopy run.
#DEBUG = stabs
#DEBUG = dwarf-2
DEBUG =

# List any extra directories to look for include files here.
# Each directory must be seperated by a space.
EXTRAINCDIRS = libstr91x/include usbinc usblibrary/inc

# List any extra directories to look for library files here.
# Each directory must be seperated by a space.
#EXTRA_LIBDIRS = ../arm7_efsl_0_2_4
EXTRA_LIBDIRS =


# Compiler flag to set the C Standard level.
# c89 - "ANSI" Cgit://github.com/zakwiggy/navi.git
# gnu89 - c89 plus GCC extensions
# c99 - ISO C99 standard (not yet fully implemented)
# gnu99 - c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Place -D or -U options for C here
CDEFS =


# Place -I options here
CINCS =

# Place -D or -U options for ASM here
ADEFS =

# Compiler flags.

ifeq ($(USE_THUMB_MODE),YES)
THUMB = -mthumb
THUMB_IW = -mthumb-interwork
else
THUMB =
THUMB_IW =
endif

# -g*: generate debugging information
# -O*: optimization level
# -f...: tuning, see GCC manual and avr-libc documentation
# -Wall...: warning level
# -Wa,...: tell GCC to pass this to the assembler.
# -adhlns...: create assembler listing
#
# Flags for C and C++ (arm-elf-gcc/arm-elf-g++)
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS) $(CINCS)
CFLAGS += -O$(OPT)
#CFLAGS += -Wall -Wcast-align -Wimplicit
#CFLAGS += -Wpointer-arith -Wswitch
#CFLAGS += -ffunction-sections -fdata-sections
#CFLAGS += -Wredundant-decls -Wreturn-type -Wshadow -Wunused
CFLAGS += -Wa,-adhlns=$(subst $(suffix $<),.lst,$<)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

# flags only for C
CONLYFLAGS += -Wnested-externs
CONLYFLAGS += $(CSTANDARD)

# flags only for C++ (arm-elf-g++)
# CPPFLAGS = -fno-rtti -fno-exceptions
CPPFLAGS =

# Assembler flags.
# -Wa,...: tell GCC to pass this to the assembler.
# -ahlns: create listing
# -g$(DEBUG): have the assembler create line number information
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:.S=.lst),-g$(DEBUG)


#Additional libraries.

# Extra libraries
# Each library-name must be seperated by a space.
# To add libxyz.a, libabc.a and libefsl.a:
# EXTRA_LIBS = xyz abc efsl
#EXTRA_LIBS = efsl
EXTRA_LIBS =

#Support for newlibc-lpc (file: libnewlibc-lpc.a)
#NEWLIBLPC = -lnewlib-lpc

MATH_LIB = -lm

# CPLUSPLUS_LIB = -lstdc++


# Linker flags.
# -Wl,...: tell GCC to pass this to linker.
# -Map: create map file
# --cref: add cross reference to map file
#LDFLAGS = -nostartfiles -Wl,-Map=$(TARGET).map,--cref,--gc-sections,-Ttext=0x000000
LDFLAGS = -Wl,-Map=$(TARGET).map,-Ttext=0x000000,-Tdata=0x4000000
LDFLAGS += -lc
LDFLAGS += $(NEWLIBLPC) $(MATH_LIB)
LDFLAGS += -lc -lgcc
LDFLAGS += $(CPLUSPLUS_LIB)
LDFLAGS += $(patsubst %,-L%,$(EXTRA_LIBDIRS))
LDFLAGS += $(patsubst %,-l%,$(EXTRA_LIBS))

# Set Linker-Script
LDFLAGS +=-T$(LINKERSCRIPTPATH)/flash_str9.ld

# Define programs and commands.
SHELL = sh
CC = $(TCHAIN)-gcc
CPP = $(TCHAIN)-g++
AR = $(TCHAIN)-ar
OBJCOPY = $(TCHAIN)-objcopy
OBJDUMP = $(TCHAIN)-objdump
SIZE = $(TCHAIN)-size
NM = $(TCHAIN)-nm
REMOVE = rm -f
REMOVEDIR = rm -f -r
COPY = cp

# Define Messages
# English
MSG_ERRORS_NONE = Errors: none
MSG_BEGIN = "-------- begin (mode: $(RUN_MODE)) --------"
MSG_END = -------- end --------
MSG_SIZE_BEFORE = Size before:
MSG_SIZE_AFTER = Size after:
MSG_FLASH = Creating load file for Flash:
MSG_EXTENDED_LISTING = Creating Extended Listing:
MSG_SYMBOL_TABLE = Creating Symbol Table:
MSG_LINKING = Linking:
MSG_COMPILING = Compiling C:
MSG_COMPILING_ARM = "Compiling C (ARM-only):"
MSG_COMPILINGCPP = Compiling C++:
MSG_COMPILINGCPP_ARM = "Compiling C++ (ARM-only):"
MSG_ASSEMBLING = Assembling:
MSG_ASSEMBLING_ARM = "Assembling (ARM-only):"
MSG_CLEANING = Cleaning project:
MSG_FORMATERROR = Can not handle output-format
MSG_LPC21_RESETREMINDER = You may have to bring the target in bootloader-mode now.

# Define all object files.
COBJ = $(SRC:.c=.o)
AOBJ = $(ASRC:.S=.o)
COBJARM = $(SRCARM:.c=.o)
AOBJARM = $(ASRCARM:.S=.o)
CPPOBJ = $(CPPSRC:.cpp=.o)
CPPOBJARM = $(CPPSRCARM:.cpp=.o)

# Define all listing files.
LST = $(ASRC:.S=.lst) $(ASRCARM:.S=.lst) $(SRC:.c=.lst) $(SRCARM:.c=.lst)
LST += $(CPPSRC:.cpp=.lst) $(CPPSRCARM:.cpp=.lst)

# Compiler flags to generate dependency files.
### GENDEPFLAGS = -Wp,-M,-MP,-MT,$(*F).o,-MF,.dep/$(@F).d
GENDEPFLAGS = -MD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS = -mcpu=$(MCU) $(THUMB_IW) -I. $(CFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS = -mcpu=$(MCU) $(THUMB_IW) -I. -x assembler-with-cpp $(ASFLAGS)


# Default target.
all: begin gccversion sizebefore build sizeafter finished end

ifeq ($(FORMAT),ihex)
build: elf hex lss sym
hex: $(TARGET).hex
IMGEXT=hex
else
ifeq ($(FORMAT),binary)
build: elf bin lss sym
bin: $(TARGET).bin
IMGEXT=bin
else
ifeq ($(FORMAT),both)
build: elf hex bin lss sym
hex: $(TARGET).hex
bin: $(TARGET).bin
else
$(error "$(MSG_FORMATERROR) $(FORMAT)")
endif
endif
endif

elf: $(TARGET).elf
lss: $(TARGET).lss
sym: $(TARGET).sym

# Eye candy
begin:
	@echo
	@echo $(MSG_BEGIN)

finished:
 @echo $(MSG_ERRORS_NONE)

end:
	@echo $(MSG_END)
	@echo


# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) -A $(TARGET).elf
sizebefore:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_BEFORE); $(ELFSIZE); echo; fi

sizeafter:
	@if [ -f $(TARGET).elf ]; then echo; echo $(MSG_SIZE_AFTER); $(ELFSIZE); echo; fi


# Display compiler version information.
gccversion :
	@$(CC) --version

# Create final output file (.hex) from ELF output file.
%.hex: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O ihex $< $@

# Create final output file (.bin) from ELF output file.
%.bin: %.elf
	@echo
	@echo $(MSG_FLASH) $@
	$(OBJCOPY) -O binary $< $@


# Create extended listing file from ELF output file.
# testing: option -C
%.lss: %.elf
	@echo
	@echo $(MSG_EXTENDED_LISTING) $@
	$(OBJDUMP) -h -S -C $< > $@


# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo
	@echo $(MSG_SYMBOL_TABLE) $@
	$(NM) -n $< > $@


# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
%.elf: $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM)
	@echo
	@echo $(MSG_LINKING) $@
	$(CC) $(THUMB) $(ALL_CFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS)
	# $(CPP) $(THUMB) $(ALL_CFLAGS) $(AOBJARM) $(AOBJ) $(COBJARM) $(COBJ) $(CPPOBJ) $(CPPOBJARM) --output $@ $(LDFLAGS)

# Compile: create object files from C source files. ARM/Thumb
$(COBJ) : %.o : %.c
	@echo
	@echo $(MSG_COMPILING) $<
	$(CC) -c $(THUMB) $(ALL_CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create object files from C source files. ARM-only
$(COBJARM) : %.o : %.c
	@echo
	@echo $(MSG_COMPILING_ARM) $<
	$(CC) -c $(ALL_CFLAGS) $(CONLYFLAGS) $< -o $@

# Compile: create object files from C++ source files. ARM/Thumb
$(CPPOBJ) : %.o : %.cpp
	@echo
	@echo $(MSG_COMPILINGCPP) $<
	$(CPP) -c $(THUMB) $(ALL_CFLAGS) $(CPPFLAGS) $< -o $@

# Compile: create object files from C++ source files. ARM-only
	$(CPPOBJARM) : %.o : %.cpp
	@echo
	@echo $(MSG_COMPILINGCPP_ARM) $<
	$(CPP) -c $(ALL_CFLAGS) $(CPPFLAGS) $< -o $@


# Compile: create assembler files from C source files. ARM/Thumb
## does not work - TODO - hints welcome
##$(COBJ) : %.s : %.c
## $(CC) $(THUMB) -S $(ALL_CFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM/Thumb
$(AOBJ) : %.o : %.S
	@echogit://github.com/zakwiggy/navi.git
	@echo $(MSG_ASSEMBLING) $<
	$(CC) -c $(THUMB) $(ALL_ASFLAGS) $< -o $@


# Assemble: create object files from assembler source files. ARM-only
$(AOBJARM) : %.o : %.S
	@echo
	@echo $(MSG_ASSEMBLING_ARM) $<
	$(CC) -c $(ALL_ASFLAGS) $< -o $@


# Target: clean project.
clean: begin clean_list finished end


clean_list :

	rm *.hex *.elf *.lss *.map *.lst *.o *.sym
	



# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)


# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex bin lss sym clean clean_list program
