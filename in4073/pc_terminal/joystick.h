#ifndef _LINUX_JOYSTICK_H
#define _LINUX_JOYSTICK_H

/*
 * $Id: joystick.h,v 1.4 2001/04/24 07:06:43 vojtech Exp $
 *
 *  Copyright (C) 1996-2000 Vojtech Pavlik
 *
 *  Sponsored by SuSE
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 * Should you need to contact me, the author, you can do so either by
 * e-mail - mail your message to <vojtech@suse.cz>, or by paper mail:
 * Vojtech Pavlik, Ucitelska 1576, Prague 8, 182 00 Czech Republic
 */

#include "pc_command.h"

/*
 * Types and constants for reading from /dev/js
 */

#define JS_EVENT_BUTTON		0x01	/* button pressed/released */
#define JS_EVENT_AXIS		0x02	/* joystick moved */
#define JS_EVENT_INIT		0x80	/* initial state of device */

struct js_event {
	uint32_t time;	/* event timestamp in milliseconds */
	int16_t value;	/* value */
	uint8_t type;	/* event type */
	uint8_t number;	/* axis/button number */
};

int open_joystick(const char *path);
int close_joystick(void);
int read_joystick(pc_command_t *command);
void read_js_event(struct js_event* evt);
int read_js_events(struct js_event* evt);

#endif /* _LINUX_JOYSTICK_H */
