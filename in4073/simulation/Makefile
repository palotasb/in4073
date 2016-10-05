CC=gcc
CFLAGS = -std=gnu11 -g -Wall -DQUADCOPTER=2 -DSIMULATION=1
EXEC = ./sim

ifeq ($(OS),Windows_NT)
PLATFORM_CFILES = 
CFLAGS += -DWINDOWS=1 -D__WINDOWS=1
else
PLATFORM_CFILES = 
endif

CFILES = \
$(abspath ./simulation.c) \
$(abspath ./model.c) \
$(abspath ../log.c) \
$(abspath ../serialcomm.c) \
$(abspath ../qc_system.c) \
$(abspath ../qc_state.c) \
$(abspath ../qc_command.c) \
$(abspath ../fixedpoint.c) \
$(abspath ../mode_0_safe.c) \
$(abspath ../mode_1_panic.c) \
$(abspath ../mode_2_manual.c) \
$(abspath ../mode_3_calibrate.c) \
$(abspath ../mode_4_yaw.c) \
$(abspath ../mode_5_full.c) \
$(abspath ../mode_d_direct.c) \

all:
	$(CC) $(CFLAGS) $(CFILES) -lm -o $(EXEC)

run:
	$(EXEC)
