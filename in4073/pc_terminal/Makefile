CC=gcc
CFLAGS = -std=gnu11 -g -Wall -lm
EXEC = ./pc_terminal

ifeq ($(OS),Windows_NT)
PLATFORM_CFILES = console_win.c serial_win.c joystick_win.c
CFLAGS += -DWINDOWS=1 -D__WINDOWS=1
else
PLATFORM_CFILES = console_unix.c serial_unix.c joystick_unix.c
endif

CFILES = pc_terminal.c pc_command.c pc_log.c keyboard.c serial.c joystick.c console.c ../serialcomm.c ../qc_state.c $(PLATFORM_CFILES)

PC_FLAGS ?=

ifdef NS
PC_FLAGS += -ns
endif
ifdef JS
PC_FLAGS += -j
endif
ifdef VIRT
PC_FLAGS += -v
endif
ifdef TLOG
PC_FLAGS += | tee $(TLOG)
endif
ifndef NPLOT
PC_FLAGS += | ./2plot.py
endif

all: 
	$(CC) $(CFLAGS) $(CFILES) -o $(EXEC)

run:
	@echo =================================
	@echo Starting: pc_terminal
	@echo ---------------------------------
	$(EXEC) $(PC_RUN) $(PC_FLAGS)
	@echo =================================

