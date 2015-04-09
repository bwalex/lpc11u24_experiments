CC=arm-none-eabi-gcc
LD=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
RM=rm -f
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

CHIPLIB=/home/alex/LPCXpresso/workspace/lpc_chip_11uxx_lib
ADAFRUIT_GFX=/home/alex/LPCXpresso/workspace/wip/adafruit/Adafruit-GFX-Library
ADAFRUIT_ILI9325=/home/alex/LPCXpresso/workspace/wip/adafruit/TFTLCD-Library
ADAFRUIT_ILI9341=/home/alex/LPCXpresso/workspace/wip/adafruit/Adafruit_ILI9341

CFLAGS=-std=c99
CFLAGS+=-mcpu=cortex-m0 -mthumb -mfloat-abi=soft
CFLAGS+=-Wall
#CFLAGS+=-specs=nano.specs -specs=nosys.specs
CFLAGS+=-Iinc
CFLAGS+=-I$(CHIPLIB)/inc
CFLAGS+=-D__USE_LPCOPEN -DNO_BOARD_LIB -DCORE_M0 -D__NEWLIB__
CFLAGS+=-Os -flto -g3
CFLAGS+=-ffunction-sections -fdata-sections
CFLAGS+=-I$(ADAFRUIT_GFX)
#CFLAGS+=-I$(ADAFRUIT_ILI9325)
CFLAGS+=-I$(ADAFRUIT_ILI9341)

CXXFLAGS=$(CFLAGS)

LDFLAGS+=$(CFLAGS)
LDFLAGS+=-specs=nano.specs -specs=nosys.specs -T linker_script.ld
LDFLAGS+=-Wl,--gc-sections -static
#LDFLAGS+=-u _printf_float
#LDFLAGS+=-u _scanf_float


OBJS=src/crp.o src/cr_startup_lpc11uxx.o  src/seeeduino_arch_test.o  src/sysinit.o
OBJS+=$(CHIPLIB)/src/sysctl_11xx.o	\
      $(CHIPLIB)/src/iocon_11xx.o 	\
      $(CHIPLIB)/src/ssp_11xx.o		\
      $(CHIPLIB)/src/clock_11xx.o 	\
      $(CHIPLIB)/src/chip_11xx.o	\
      $(CHIPLIB)/src/timer_11xx.o

CXXOBJS=$(ADAFRUIT_GFX)/Adafruit_GFX.o
#CXXOBJS+=$(ADAFRUIT_ILI9325)/Adafruit_TFTLCD.o
CXXOBJS+=$(ADAFRUIT_ILI9341)/Adafruit_ILI9341.o
CXXOBJS=

seeeduino_arch_test.bin: test.axf
	$(OBJCOPY) -O binary test.axf seeeduino_arch_test.bin

test.axf: $(OBJS) $(CXXOBJS)
	$(LD) $(LDFLAGS) -o test.axf $(OBJS) $(CXXOBJS)

#$(SIZE) test.axf

clean:
	$(RM) $(OBJS) $(CXXOBJS)
	$(RM) test.axf
