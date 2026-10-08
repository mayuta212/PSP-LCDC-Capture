TARGET = lcdc_capture
OBJS = main.o audio_capture.o audio_pops.o
CFLAGS += -O3 -G0 -Wall -Wextra -fno-pic -mno-check-zero-division
BUILD_PRX = 1
USE_KERNEL_LIBC = 1
USE_KERNEL_LIBS = 1
PSP_FW_VERSION = 660
LIBS = -lpspsystemctrl_kernel -lpspusb_driver -lpspusbbus_driver -lpspdisplay_driver -lpspkernel
PSPSDK = $(shell psp-config --pspsdk-path)
include $(PSPSDK)/lib/build.mak
