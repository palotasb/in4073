#ifndef __PC_TERMINAL_H
#define __PC_TERMINAL_H

#include <inttypes.h>
#include <stdbool.h>

#define JS_DEV	"/dev/input/js0"
#define SERIAL_DEV	"/dev/ttyUSB0"

typedef enum {
		SAFEMODE 	 = 0,
		PANICMODE 	 = 1,
		MANUALMODE   = 2,
		CALMODE		 = 3,
		YAWCTRLMODE  = 4,
		FULLCRTLMODE = 5,
		YAWMODE		 = 6,
		HEIGHTMODE	 = 7,
		WIRELESSMODE = 8,
		SHUTDOWNMODE = 15
} operation_mode;


typedef struct keyboad_state {
	bool updated;
	operation_mode mode;
	bool mode_change;
	int8_t lift;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
	int8_t P;		//yaw control p control
	int8_t P1;		//roll/pitch p1 control
	int8_t P2;		//roll/pitch p2 control
} keyboad_state;

typedef struct joystick_state {
	bool updated;
	bool abort;
	int8_t lift;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
	int fd;
} joystick_state;

typedef struct manualmode_state {
	int8_t lift;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
} manualmode_state;


inline int min(int a, int b);
inline int max(int a, int b);

manualmode_state init_manualmode_state();
manualmode_state update_manualmode_state(keyboad_state kb, joystick_state js);
void print_manualmode_state(manualmode_state state);

void init_joystick_state(joystick_state *js);
int open_joystick(const char *path, joystick_state *js);
int read_joystick(joystick_state *current);
int close_joystick(joystick_state *js);
void print_joystick_state(joystick_state *js);

keyboad_state init_keyboard_state();
void print_keyboard_state(keyboad_state state);
keyboad_state read_keyboard(keyboad_state current);

void print_help();
void run_terminal(char* serial, char* js) ;






#endif
