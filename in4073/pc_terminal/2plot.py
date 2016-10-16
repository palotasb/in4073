#!/usr/bin/python
# coding=utf-8

import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
import itertools
from collections import deque
try:
    from Queue import Queue, Empty
except ImportError:
    from queue import Queue, Empty  # python 3.x
from threading  import Thread

# This example uses subclassing, but there is no reason that the proper
# function couldn't be set up and then use FuncAnimation. The code is long, but
# not really complex. The length is due solely to the fact that there are a
# total of 9 lines that need to be changed for the animation as well as 3
# subplots that need initial set up.
class SubplotAnimation(animation.TimedAnimation):
    def __init__(self):

        # VARIABLES

        #self.t = np.linspace(0, 80, 400)
        #self.x = np.cos(2 * np.pi * self.t / 10.)
        #self.y = np.sin(2 * np.pi * self.t / 10.)
        #self.z = 10 * self.t

        fig = plt.figure()
        d_len = 400
        nans = [np.nan for _ in xrange(d_len)]
        self.x_len = d_len

        self.v_mode = deque(nans, d_len)
        self.v_voltage = deque(nans, d_len)
        self.v_pressure = deque(nans, d_len)
        self.v_temp = deque(nans, d_len)
        self.v_pr0 = deque(nans, d_len)
        self.v_pr1 = deque(nans, d_len)
        self.v_pr2 = deque(nans, d_len)
        self.v_pr3 = deque(nans, d_len)
        self.v_pr4 = deque(nans, d_len)
        self.v_x = deque(nans, d_len)
        self.v_y = deque(nans, d_len)
        self.v_z = deque(nans, d_len)
        self.v_u = deque(nans, d_len)
        self.v_v = deque(nans, d_len)
        self.v_w = deque(nans, d_len)
        self.v_X = deque(nans, d_len)
        self.v_Y = deque(nans, d_len)
        self.v_Z = deque(nans, d_len)
        self.v_lift = deque(nans, d_len)
        self.v_roll = deque(nans, d_len)
        self.v_pitch = deque(nans, d_len)
        self.v_yaw = deque(nans, d_len)
        self.v_ae1 = deque(nans, d_len)
        self.v_ae2 = deque(nans, d_len)
        self.v_ae3 = deque(nans, d_len)
        self.v_ae4 = deque(nans, d_len)
        self.v_phi = deque(nans, d_len)
        self.v_theta = deque(nans, d_len)
        self.v_psi = deque(nans, d_len)
        self.v_p = deque(nans, d_len)
        self.v_q = deque(nans, d_len)
        self.v_r = deque(nans, d_len)
        self.v_L = deque(nans, d_len)
        self.v_M = deque(nans, d_len)
        self.v_N = deque(nans, d_len)
        self.v_sp = deque(nans, d_len)
        self.v_sq = deque(nans, d_len)
        self.v_sr = deque(nans, d_len)
        self.v_sax = deque(nans, d_len)
        self.v_say = deque(nans, d_len)
        self.v_saz = deque(nans, d_len)
        self.v_sphi = deque(nans, d_len) 
        self.v_stheta = deque(nans, d_len) 
        self.v_spsi = deque(nans, d_len) 

        #ax1 = fig.add_subplot(1, 2, 1)
        #ax2 = fig.add_subplot(2, 2, 2)
        #ax3 = fig.add_subplot(2, 2, 4)

        # AXIS DEFINITIONS

        self.ax_mode     = fig.add_subplot(5, 3, 1)
        self.ax_voltage  = fig.add_subplot(5, 3, 4)
        self.ax_pressure = fig.add_subplot(5, 3, 7)
        self.ax_temp     = fig.add_subplot(5, 3, 10)
        self.ax_prof     = fig.add_subplot(5, 3, 13)
        self.axes_1 = [self.ax_mode, self.ax_voltage, self.ax_pressure, self.ax_temp, self.ax_prof]

        self.ax_setpoint = fig.add_subplot(5, 3, 2)
        self.ax_att      = fig.add_subplot(5, 3, 5)
        self.ax_spin     = fig.add_subplot(5, 3, 8)
        self.ax_torque   = fig.add_subplot(5, 3, 11)
        self.ax_motor    = fig.add_subplot(5, 3, 14)
        self.axes_2 = [self.ax_setpoint, self.ax_att, self.ax_spin, self.ax_torque, self.ax_motor]


        self.ax_pos      = fig.add_subplot(5, 6, 5)
        self.ax_velo     = fig.add_subplot(5, 6, 6)
        self.ax_satt     = fig.add_subplot(5, 3, 6)
        self.ax_sp       = fig.add_subplot(5, 3, 9)
        self.ax_sacc     = fig.add_subplot(5, 3, 12)
        self.ax_force    = fig.add_subplot(5, 3, 15)
        self.axes_3 = [self.ax_pos, self.ax_velo, self.ax_satt, self.ax_sp, self.ax_sacc, self.ax_force]

        self.axes_all = self.axes_1 + self.axes_2 + self.axes_3

        # AXIS Y LABELS AND LIMITS

        self.axes_autoscale = []

        self.ax_mode.set_ylabel('mode')
        self.ax_mode.set_ylim(-0.5, 5.5)
        self.ax_voltage.set_ylabel('Voltage [V]')
        self.ax_voltage.set_ylim(0, 15)
        self.ax_pressure.set_ylabel('Pressure [kPa]')
        self.ax_pressure.set_ylim(380, 420)
        self.ax_temp.set_ylabel('Temp [C]')
        self.ax_temp.set_ylim(-5, 35)
        self.ax_prof.set_ylabel('Time [us]')
        self.ax_prof.set_ylim(0, 10000)

        self.ax_pos.set_ylabel('Pos [m]')
        self.axes_autoscale = self.axes_autoscale + [self.ax_pos]
        self.ax_velo.set_ylabel('Velo [m/s]')
        self.ax_velo.set_ylim(-3, 3)
        self.ax_force.set_ylabel('Force [N]')
        self.ax_force.set_ylim(-5, 5)
        self.ax_setpoint.set_ylabel('Setpoints [?]')
        self.axes_autoscale = self.axes_autoscale + [self.ax_setpoint]
        self.ax_motor.set_ylabel('Motor')
        self.ax_motor.set_ylim(0, 1000)

        self.ax_att.set_ylabel('Attitude [rad]')
        self.ax_att.set_ylim(-np.pi, np.pi)
        self.ax_spin.set_ylabel('Spin [rad/s]')
        self.ax_spin.set_ylim(-np.pi, np.pi)
        self.ax_torque.set_ylabel('Torque [N m]')
        self.ax_torque.set_ylim(-10, 10)
        self.ax_sp.set_ylabel('Sensor spin [rad/s]')
        self.ax_sp.set_ylim(-2*np.pi, 2*np.pi)
        self.ax_sacc.set_ylabel('Sensor acc [m/s/s]')
        self.ax_sacc.set_ylim(-2.1, 2.1)
        self.ax_satt.set_ylabel('Sensor att [rad]')
        #self.axes_autoscale = self.axes_autoscale + [self.ax_satt]
        self.ax_satt.set_ylim(-np.pi, np.pi)

        # COLORS
        c0 = 'black'
        c1 = 'darkred'
        c2 = 'darkgreen'
        c3 = 'blue'
        c4 = 'darkorange'

        # LINES

        self.line_mode = Line2D([], [], color=c0, label='Mode')
        self.ax_mode.add_line(self.line_mode)
        self.line_voltage = Line2D([], [], color=c0, label='V')
        self.ax_voltage.add_line(self.line_voltage)
        self.line_pressure = Line2D([], [], color=c0, label='Press')
        self.ax_pressure.add_line(self.line_pressure)
        self.line_temp = Line2D([], [], color=c0, label='T')
        self.ax_temp.add_line(self.line_temp)
        self.line_pr0 = Line2D([], [], color=c0, label='t0')
        self.line_pr1 = Line2D([], [], color=c1, label='t1')
        self.line_pr2 = Line2D([], [], color=c2, label='t2')
        self.line_pr3 = Line2D([], [], color=c3, label='t3')
        self.line_pr4 = Line2D([], [], color=c4, label='t4')
        self.lines_prof = [self.line_pr0, self.line_pr1, self.line_pr2, self.line_pr3, self.line_pr4]
        for l in self.lines_prof:
            self.ax_prof.add_line(l)
        self.lines_1 = [self.line_mode, self.line_voltage, self.line_pressure, self.line_temp] + self.lines_prof

        self.line_x = Line2D([], [], color=c1, label='x')
        self.line_y = Line2D([], [], color=c2, label='y')
        self.line_z = Line2D([], [], color=c3, label='z')
        self.lines_pos = [self.line_x, self.line_y, self.line_z]
        for l in self.lines_pos:
            self.ax_pos.add_line(l)
        self.line_u = Line2D([], [], color=c1, label='u')
        self.line_v = Line2D([], [], color=c2, label='v')
        self.line_w = Line2D([], [], color=c3, label='w')
        self.lines_velo = [self.line_u, self.line_v, self.line_w]
        for l in self.lines_velo:
            self.ax_velo.add_line(l)
        self.line_X = Line2D([], [], color=c1, label='X')
        self.line_Y = Line2D([], [], color=c2, label='Y')
        self.line_Z = Line2D([], [], color=c3, label='Z')
        self.lines_force = [self.line_X, self.line_Y, self.line_Z]
        for l in self.lines_force:
            self.ax_force.add_line(l)
        self.line_lift  = Line2D([], [], color=c1, label='lift')
        self.line_roll  = Line2D([], [], color=c2, label='roll')
        self.line_pitch = Line2D([], [], color=c3, label='pitch')
        self.line_yaw   = Line2D([], [], color=c4, label='yaw')
        self.lines_setpoint = [self.line_lift, self.line_roll, self.line_pitch, self.line_yaw]
        for l in self.lines_setpoint:
            self.ax_setpoint.add_line(l)
        self.line_ae1 = Line2D([], [], color=c1, label='ae1')
        self.line_ae2 = Line2D([], [], color=c2, label='ae2')
        self.line_ae3 = Line2D([], [], color=c3, label='ae3')
        self.line_ae4 = Line2D([], [], color=c4, label='ae4')
        self.lines_motor = [self.line_ae1, self.line_ae2, self.line_ae3, self.line_ae4]
        for l in self.lines_motor:
            self.ax_motor.add_line(l)
        self.lines_2 = self.lines_pos + self.lines_velo + self.lines_force + self.lines_setpoint + self.lines_motor
        
        self.line_phi   = Line2D([], [], color=c1, label='phi')
        self.line_theta = Line2D([], [], color=c2, label='theta')
        self.line_psi   = Line2D([], [], color=c3, label='psi')
        self.lines_att = [self.line_phi, self.line_theta, self.line_psi]
        for l in self.lines_att:
            self.ax_att.add_line(l)
        self.line_p = Line2D([], [], color=c1, label='p')
        self.line_q = Line2D([], [], color=c2, label='q')
        self.line_r = Line2D([], [], color=c3, label='r')
        self.lines_spin = [self.line_p, self.line_q, self.line_r]
        for l in self.lines_spin:
            self.ax_spin.add_line(l)
        self.line_L = Line2D([], [], color=c1, label='L')
        self.line_M = Line2D([], [], color=c2, label='M')
        self.line_N = Line2D([], [], color=c3, label='N')
        self.lines_torque = [self.line_L, self.line_M, self.line_N]
        for l in self.lines_torque:
            self.ax_torque.add_line(l)
        self.line_sp = Line2D([], [], color=c1, label='sp')
        self.line_sq = Line2D([], [], color=c2, label='sq')
        self.line_sr = Line2D([], [], color=c3, label='sr')
        self.lines_sp = [self.line_sp, self.line_sq, self.line_sr]
        for l in self.lines_sp:
            self.ax_sp.add_line(l)
        self.line_sax = Line2D([], [], color=c1, label='sax')
        self.line_say = Line2D([], [], color=c2, label='say')
        self.line_saz = Line2D([], [], color=c3, label='saz')
        self.lines_sa = [self.line_sax, self.line_say, self.line_saz]
        for l in self.lines_sa:
            self.ax_sacc.add_line(l)
        self.line_sphi =    Line2D([], [], color=c1, label='sphi')
        self.line_stheta =  Line2D([], [], color=c2, label='stheta')
        self.line_spsi =    Line2D([], [], color=c3, label='spsi')
        self.lines_satt = [self.line_sphi, self.line_stheta, self.line_spsi]
        for l in self.lines_satt:
            self.ax_satt.add_line(l) 
        self.lines_3 = self.lines_att + self.lines_spin + self.lines_torque + self.lines_sp + self.lines_sa + self.lines_satt

        self.lines_all = self.lines_1 + self.lines_2 + self.lines_3

        for a in self.axes_all:
            a.set_xlim(0, self.x_len)
            a.legend(fontsize='xx-small', loc='upper left')

        animation.TimedAnimation.__init__(self, fig, interval=50, blit=True)

    def _draw_frame(self, framedata):
        self._drawn_artists = []

        # read line without blocking
        while 1:
            try:  line = q.get_nowait() # or q.get(timeout=.1)
            except Empty:
                break
            else: # got line
                self.process_line(line)
        
        for ax in (self.axes_autoscale):
            ax.relim()
            ax.autoscale('y')
            ax.autoscale_view(False, False, True)

    def process_line(self, line):
        words = ['NaN'] + line.split("\t")
        # Nan is added to match indices in pc_log.c comments

        self.put_word(words[2], self.v_mode, self.line_mode)
        self.put_word(words[3], self.v_lift, self.line_lift)
        self.put_word(words[4], self.v_roll, self.line_roll)
        self.put_word(words[5], self.v_pitch, self.line_pitch)
        self.put_word(words[6], self.v_yaw, self.line_yaw)
        self.put_word(words[7], self.v_ae1, self.line_ae1)
        self.put_word(words[8], self.v_ae2, self.line_ae2)
        self.put_word(words[9], self.v_ae3, self.line_ae3)
        self.put_word(words[10], self.v_ae4, self.line_ae4)
        self.put_word(words[11], self.v_sp, self.line_sp)
        self.put_word(words[12], self.v_sq, self.line_sq)
        self.put_word(words[13], self.v_sr, self.line_sr)
        self.put_word(words[14], self.v_sax, self.line_sax)
        self.put_word(words[15], self.v_say, self.line_say)
        self.put_word(words[16], self.v_saz, self.line_saz)
        self.put_word(words[17], self.v_temp, self.line_temp)
        self.put_word(words[18], self.v_pressure, self.line_pressure)
        self.put_word(words[19], self.v_voltage, self.line_voltage)
        self.put_word(words[20], self.v_x, self.line_x)
        self.put_word(words[21], self.v_y, self.line_y)
        self.put_word(words[22], self.v_z, self.line_z)
        self.put_word(words[23], self.v_phi, self.line_phi)
        self.put_word(words[24], self.v_theta, self.line_theta)
        self.put_word(words[25], self.v_psi, self.line_psi)
        self.put_word(words[26], self.v_X, self.line_X)
        self.put_word(words[27], self.v_Y, self.line_Y)
        self.put_word(words[28], self.v_Z, self.line_Z)
        self.put_word(words[29], self.v_L, self.line_L)
        self.put_word(words[30], self.v_M, self.line_M)
        self.put_word(words[31], self.v_N, self.line_N)
        self.put_word(words[32], self.v_u, self.line_u)
        self.put_word(words[33], self.v_v, self.line_v)
        self.put_word(words[34], self.v_w, self.line_w)
        self.put_word(words[35], self.v_p, self.line_p)
        self.put_word(words[36], self.v_q, self.line_q)
        self.put_word(words[37], self.v_r, self.line_r)
        #self.put_word(words[38], self.v_yawp, self.line_yawp)
        #self.put_word(words[39], self.v_p1, self.line_p1)
        #self.put_word(words[40], self.v_p2, self.line_p2)
        self.put_word(words[41], self.v_pr0, self.line_pr0)
        self.put_word(words[45], self.v_pr1, self.line_pr1)
        self.put_word(words[49], self.v_pr2, self.line_pr2)
        self.put_word(words[53], self.v_pr3, self.line_pr3)
        self.put_word(words[57], self.v_pr4, self.line_pr4)
        self.put_word(words[61], self.v_sphi, self.line_sphi)
        self.put_word(words[62], self.v_stheta, self.line_stheta)
        self.put_word(words[63], self.v_spsi, self.line_spsi)

    def put_word(self, word, dqv, line):
        if word == "NaN":
            # dqv.append(np.nan)
            ''' do nothing '''
        else:
            dqv.append(float(word))
            line.set_ydata(dqv)
            if ((line in self._drawn_artists) == False):
                self._drawn_artists = self._drawn_artists + [line]

    def new_frame_seq(self):
        return itertools.count()

    def _init_draw(self):
        for l in self.lines_all:
            l.set_data(xrange(self.x_len), [np.nan for _ in xrange(self.x_len)])
            l.set_linewidth(0.5)

def enqueue_output(out, queue):
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()
    exit(0)



q = Queue()

t = Thread(target=enqueue_output, args=(sys.stdin, q))
t.daemon = True # thread dies with the program
t.start()

ani = SubplotAnimation()
#ani.save('test_sub.mp4')
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()