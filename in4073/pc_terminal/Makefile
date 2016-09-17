CC=gcc
CFLAGS = -g -Wall -lm 
EXEC = ./pc_terminal

all: 
	$(CC) $(CFLAGS) pc_terminal.c keyboard.c joystick.c console.c serial.c ../serialcomm.c -o $(EXEC)

run: all
	$(EXEC)

run-nj: all
	$(EXEC) -nj

