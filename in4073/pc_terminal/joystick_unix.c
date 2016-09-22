#include "pc_terminal.h"
#include "joystick.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


#include <asm/types.h>
#include <linux/input.h>

/*
 * Version
 */

#define JS_VERSION		0x020100z

/*
 * IOCTL commands for joystick driver
 */

#define JSIOCGVERSION		_IOR('j', 0x01, __u32)				/* get driver version */

#define JSIOCGAXES		_IOR('j', 0x11, __u8)				/* get number of axes */
#define JSIOCGBUTTONS		_IOR('j', 0x12, __u8)				/* get number of buttons */
#define JSIOCGNAME(len)		_IOC(_IOC_READ, 'j', 0x13, len)			/* get identifier string */

#define JSIOCSCORR		_IOW('j', 0x21, struct js_corr)			/* set correction values */
#define JSIOCGCORR		_IOR('j', 0x22, struct js_corr)			/* get correction values */

#define JSIOCSAXMAP		_IOW('j', 0x31, __u8[ABS_MAX])			/* set axis mapping */
#define JSIOCGAXMAP		_IOR('j', 0x32, __u8[ABS_MAX])			/* get axis mapping */
#define JSIOCSBTNMAP		_IOW('j', 0x33, __u16[KEY_MAX - BTN_MISC])	/* set button mapping */
#define JSIOCGBTNMAP		_IOR('j', 0x34, __u16[KEY_MAX - BTN_MISC])	/* get button mapping */

/*
 * Types and constants for get/set correction
 */

#define JS_CORR_NONE		0x00	/* returns raw values */
#define JS_CORR_BROKEN		0x01	/* broken line */

struct js_corr {
	__s32 coef[8];
	__s16 prec;
	__u16 type;
};

/*
 * v0.x compatibility definitions
 */

#define JS_RETURN		sizeof(struct JS_DATA_TYPE)
#define JS_TRUE			1
#define JS_FALSE		0
#define JS_X_0			0x01
#define JS_Y_0			0x02
#define JS_X_1			0x04
#define JS_Y_1			0x08
#define JS_MAX			2

#define JS_DEF_TIMEOUT		0x1300
#define JS_DEF_CORR		0
#define JS_DEF_TIMELIMIT	10L

#define JS_SET_CAL		1
#define JS_GET_CAL		2
#define JS_SET_TIMEOUT		3
#define JS_GET_TIMEOUT		4
#define JS_SET_TIMELIMIT	5
#define JS_GET_TIMELIMIT	6
#define JS_GET_ALL		7
#define JS_SET_ALL		8

struct JS_DATA_TYPE {
	int buttons;
	int x;
	int y;
};

struct JS_DATA_SAVE_TYPE {
	int JS_TIMEOUT;
	int BUSY;
	long JS_EXPIRETIME;
	long JS_TIMELIMIT;
	struct JS_DATA_TYPE JS_SAVE;
	struct JS_DATA_TYPE JS_CORR;
};

int fd;


/******************************
open_joystick()
*******************************
Description:
	Opens the I/O file for the joystick

Inputs:
	const char *path
		contains the path to the joysticks I/O file.
	joystick_state *js
		contains a pointer to the joystick state. 
		after a successfull call, the fd is set to
		represent the right filedescriptor

Returns:
	zero if succesfull or nonzero when it fails

Author:
	Koos Eerden
*******************************/
int open_joystick(const char *path) {

	int err = 0;
	if ((fd = open(path, O_RDONLY)) < 0) {
		err = fd;
		fd = 0;
	}
	fcntl(fd, F_SETFL, O_NONBLOCK);

	return err;
}

/******************************
close_joystick()
*******************************
Description:
	Closes the joystick

Inputs:
	-	joystick_state *state:
			pointer to the current state structure
Returns:
	zero on success otherwise nonzero
Author:
	Koos Eerden
*******************************/
int close_joystick(void) {
	int err = close(fd);
	if(!err)
		fd = 0;
	return err;
}

int read_js_events(struct js_event* js) {
	return
		read(fd, &js, sizeof(struct js_event))
		==	sizeof(struct js_event);
}
