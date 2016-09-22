CC=gcc
CFLAGS = -g -Wall -lm 
EXEC = ./pc_terminal

ifeq ($(OS),Windows_NT)
PLATFORM_CFILES = console_win.c serial_win.c joystick_win.c
CFLAGS += -DWINDOWS=1 -D__WINDOWS=1
else
PLATFORM_CFILES = console_unix.c serial_unix.c joystick_unix.c
endif

CFILES = pc_terminal.c pc_log.c keyboard.c serial.c joystick.c ../serialcomm.c $(PLATFORM_CFILES)

all: 
	$(CC) $(CFLAGS) $(CFILES) -o $(EXEC)

run: all
	$(EXEC)

run-nj: all
	$(EXEC) -nj

