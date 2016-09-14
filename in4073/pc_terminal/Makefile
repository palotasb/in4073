CC=gcc
CFLAGS = -g -Wall -lm --std=gnu11
EXEC = ./pc-terminal

all:
	$(CC) $(CFLAGS) pc_terminal.c ../serialcomm.c -o $(EXEC)

run: all
	$(EXEC)

