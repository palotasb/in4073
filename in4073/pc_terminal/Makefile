CC = gcc
CFLAGS = -g -Wall -lm --std=gnu11 -I. -I../.
EXEC = ./pc-terminal

ifeq ($(OS),Windows_NT)
PLATFORM_TERMINAL = pc_terminal_windows.c
else
PLATFORM_TERMINAL = pc_terminal_unix.c
endif

CFILES = $(PLATFORM_TERMINAL) ../serialcomm.c pc_terminal.c

ifneq ($(wildcard local.makefile),) 
include local.makefile
endif

all:
	$(CC) $(CFLAGS) $(CFILES) -o $(EXEC)

run: all
	$(EXEC)

