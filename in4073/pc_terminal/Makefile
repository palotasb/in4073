CC=gcc
CFLAGS = -g -Wall -lm 
EXEC = ./pc_terminal

ifeq ($(OS),Windows_NT)
PLATFORM_TERMINAL = console_win.c serial_win.c
CFLAGS += -DWINDOWS=1
else
PLATFORM_TERMINAL = console.c serial.c
endif


all: 
	$(CC) $(CFLAGS) pc_terminal.c keyboard.c joystick.c console.c serial.c ../serialcomm.c -o $(EXEC)

run: all
	$(EXEC)

run-nj: all
	$(EXEC) -nj

