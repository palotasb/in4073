#include "pc_log.h"
#include "../serialcomm.h"
#include "../fixedpoint.h"
#include "../qc_mode.h"
#include <string.h>
#include <stdarg.h>
#ifdef WINDOWS
    #include <windows.h>
#endif

// Value to print in placeof on unknown number
#define _NAN    "NaN"

// Value to print to separate values
#define _SEP    "\t"

// Value to print to separate entries
#define _END    "\n"

static void pc_log_flush(pc_log_t* log);
static void pc_log_clear(pc_log_t* log);
static void pc_log_print(pc_log_t* log, const char * fmt, pc_log_item_t item, ...);

/******************************
pc_logfiles_open_default()
*******************************
Description:
	Opens the default log files and places the descriptors in the struct

Inputs:
	-	pc_logfiles_t* files:
			Pointer to the file structure.
			the structure will be updated with the new filedescriptors
			split_files will be set to true 

Returns:
	true when all files are successfully opened

Author:
	Koos Eerden
*******************************/

bool pc_logfiles_open_default(pc_logfiles_t*  files) {
	files->split_files = true;
	if(!(files->setpoints = fopen(SETPOINTS_FILE, "a")))
		return false;
	if(!(files->motors = fopen(MOTORS_FILE, "a")))
		return false;
	if(!(files->gyro = fopen(GYRO_FILE, "a")))
		return false;
	if(!(files->accelero = fopen(ACCELERO_FILE, "a")))
		return false;
	if(!(files->position = fopen(POSITION_FILE, "a")))
		return false;
	if(!(files->angle = fopen(ANGLE_FILE, "a")))
		return false;
	if(!(files->force = fopen(FORCE_FILE, "a")))
		return false;
	if(!(files->torque = fopen(TORQUE_FILE, "a")))
		return false;
	if(!(files->spin = fopen(SPIN_FILE, "a")))
		return false;
	if(!(files->trim = fopen(TRIM_FILE, "a")))
		return false;
	if(!(files->state = fopen(STATE_FILE, "a")))
		return false;
	if(!(files->baro = fopen(BARO_FILE, "a")))
		return false;
	return true;
}

/******************************
pc_logfiles_open_single()
*******************************
Description:
	Opens a single logfile

Inputs:
	-	pc_logfiles_t* files:
			Pointer to the file structure.
			split_files will be set to false 

	- FILE* f filedescriptor to the file

Author:
	Koos Eerden
*******************************/

void pc_logfiles_set_single(pc_logfiles_t* files, FILE* f){
	files->split_files = false;
	files->single = f;
}

/******************************
pc_logfiles_close()
*******************************
Description:
	Closes all the files if split_files is set to true,
	otherwise the single logfile has to be closed manually

Inputs:
	-	pc_logfiles_t* files:
			Pointer to the file structure.

Returns:
	true if successful, or if split_files was false
Author:
	Koos Eerden
*******************************/
bool pc_logfiles_close(pc_logfiles_t* files) {
	if(files->split_files) {
		if(fclose(files->setpoints))
			return false;
		if(fclose(files->motors))
			return false;
		if(fclose(files->gyro))
			return false;
		if(fclose(files->accelero))
			return false;
		if(fclose(files->position))
			return false;
		if(fclose(files->angle))
			return false;
		if(fclose(files->force))
			return false;
		if(fclose(files->torque))
			return false;
		if(fclose(files->spin))
			return false;
		if(fclose(files->trim))
			return false;
		if(fclose(files->state))
			return false;
		if(fclose(files->baro))
			return false;
	}

	return true;
}


bool pc_log_init(pc_log_t* log, pc_logfiles_t* files) {

    log->files = files;
    qc_state_init(&log->state);
    log->time = 0;
    log->mode = MODE_UNKNOWN;
    log->initialised = false;
    pc_log_clear(log);
    return true;
}

void pc_log_receive(pc_log_t* log, message_t* message) {
    int pr_id;
    switch (message->ID) {
        case MESSAGE_LOG_END_ID:
            if (log->initialised && !log->files->split_files) {
                pc_log_flush(log);
            }
            break;
        case MESSAGE_TIME_MODE_VOLTAGE_ID:
            if (log->initialised && !log->files->split_files) {
                pc_log_flush(log);
                pc_log_clear(log);
            }
            log->initialised = true;
            log->time =                 MESSAGE_TIME_VALUE(message);
            log->state.sensor.voltage = MESSAGE_VOLTAGE_VALUE(message);
            log->mode = (qc_mode_t)     MESSAGE_MODE_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->state, "%u" _SEP "%hhu" _SEP "%d\n", log->time, log->mode, log->state.sensor.voltage);
				}else{
		         log->set[PC_LOG_time] = true;
		         log->set[PC_LOG_mode] = true;
		         log->set[PC_LOG_voltage] = true;
				}
            break;
        case MESSAGE_SPQR_ID:
            log->state.sensor.sp = MESSAGE_SP_VALUE(message);
            log->state.sensor.sq = MESSAGE_SQ_VALUE(message);
            log->state.sensor.sr = MESSAGE_SR_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->gyro, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.sensor.sp, 
																											log->state.sensor.sq, log->state.sensor.sr);
				}else{
		         log->set[PC_LOG_sp] = true;
		         log->set[PC_LOG_sq] = true;
		         log->set[PC_LOG_sr] = true;
				}
            break;
        case MESSAGE_SAXYZ_ID:
            log->state.sensor.sax = MESSAGE_SAX_VALUE(message);
            log->state.sensor.say = MESSAGE_SAY_VALUE(message);
            log->state.sensor.saz = MESSAGE_SAZ_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->accelero, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.sensor.sax,
																												log->state.sensor.say, log->state.sensor.saz);
				}else{
		         log->set[PC_LOG_sax] = true;
		         log->set[PC_LOG_say] = true;
		         log->set[PC_LOG_saz] = true;
				}
            break;
        case MESSAGE_AE1234_ID:
            log->state.motor.ae1 = MESSAGE_AE1_VALUE(message);
            log->state.motor.ae2 = MESSAGE_AE2_VALUE(message);
            log->state.motor.ae3 = MESSAGE_AE3_VALUE(message);
            log->state.motor.ae4 = MESSAGE_AE4_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->motors, "%u" _SEP "%d" _SEP "%d" _SEP  "%d" _SEP "%d\n",	log->time, log->state.motor.ae1,
																															log->state.motor.ae2,  log->state.motor.ae3,
																															log->state.motor.ae4);
				}else{
		         log->set[PC_LOG_ae1] = true;
		         log->set[PC_LOG_ae2] = true;
		         log->set[PC_LOG_ae3] = true;
		         log->set[PC_LOG_ae4] = true;
				}
				break;
        case MESSAGE_TEMP_PRESSURE_ID:
            log->state.sensor.temperature = MESSAGE_TEMP_VALUE(message);
            log->state.sensor.pressure =    MESSAGE_PRESSURE_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->baro, "%u" _SEP "%f" _SEP "%f\n",	log->time, FLOAT_FP(log->state.sensor.temperature, 8),
																							FLOAT_FP(log->state.sensor.pressure, 8));
				}else{
            	log->set[PC_LOG_temperature] =  true;
            	log->set[PC_LOG_pressure] =     true;
				}
            break;
        case MESSAGE_XYZPOS_ID:
            log->state.pos.x = MESSAGE_XPOS_VALUE(message);
            log->state.pos.y = MESSAGE_YPOS_VALUE(message);
            log->state.pos.z = MESSAGE_ZPOS_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->position, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.pos.x,
																												log->state.pos.y, log->state.pos.z);
				}else{
		         log->set[PC_LOG_x] = true;
		         log->set[PC_LOG_y] = true;
		         log->set[PC_LOG_z] = true;
				}
		      break;
        case MESSAGE_PHI_THETA_PSI_ID:
            log->state.att.phi =    MESSAGE_PHI_VALUE(message);
            log->state.att.theta =  MESSAGE_THETA_VALUE(message);
            log->state.att.psi =    MESSAGE_PSI_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->angle, "%u" _SEP "%f" _SEP  "%f" _SEP "%f\n",	log->time, FLOAT_FP(log->state.att.phi, 8),
																							FLOAT_FP(log->state.att.theta, 8),
																							FLOAT_FP(log->state.att.psi, 8));
				}else{
            	log->set[PC_LOG_phi] =      true;
            	log->set[PC_LOG_theta] =    true;
            	log->set[PC_LOG_psi] =      true;
				}
            break;
        case MESSAGE_XYZFORCE_ID:
            log->state.force.X = MESSAGE_XFORCE_VALUE(message);
            log->state.force.Y = MESSAGE_YFORCE_VALUE(message);
            log->state.force.Z = MESSAGE_ZFORCE_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->force, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.force.X,
																												log->state.force.Y, log->state.force.Z);
				}else{
		         log->set[PC_LOG_X] = true;
		         log->set[PC_LOG_Y] = true;
		         log->set[PC_LOG_Z] = true;
				}
            break;
        case MESSAGE_LMN_ID:
            log->state.torque.L = MESSAGE_L_VALUE(message);
            log->state.torque.M = MESSAGE_M_VALUE(message);
            log->state.torque.N = MESSAGE_N_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->torque, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.torque.L,
																												log->state.torque.M, log->state.torque.N);
				}else{
		         log->set[PC_LOG_L] = true;
		         log->set[PC_LOG_M] = true;
		         log->set[PC_LOG_N] = true;
				}
            break;
        case MESSAGE_PQR_ID:
            log->state.spin.p = MESSAGE_P_VALUE(message);
            log->state.spin.q = MESSAGE_Q_VALUE(message);
            log->state.spin.r = MESSAGE_R_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->spin, "%u" _SEP "%d" _SEP "%d" _SEP "%d\n",	log->time, log->state.spin.p,
																												log->state.spin.q, log->state.spin.r);
				}else{
		         log->set[PC_LOG_p] = true;
		         log->set[PC_LOG_q] = true;
		         log->set[PC_LOG_r] = true;
				}
            break;
        case MESSAGE_P12_ID:
            log->state.trim.p1 = MESSAGE_P1_VALUE(message);
            log->state.trim.p2 = MESSAGE_P2_VALUE(message);
				if(log->files->split_files) {
					fprintf(log->files->trim, "%u" _SEP "%d" _SEP "%d\n",	log->time, log->state.trim.p1, log->state.trim.p2);
				}else{
		         log->set[PC_LOG_p1] = true;
		         log->set[PC_LOG_p2] = true;
				}
            log->set[PC_LOG_p1] = true;
            log->set[PC_LOG_p2] = true;
            break;
        case MESSAGE_PROFILE_0_CURR_ID:
        case MESSAGE_PROFILE_1_CURR_ID:
        case MESSAGE_PROFILE_2_CURR_ID:
        case MESSAGE_PROFILE_3_CURR_ID:
        case MESSAGE_PROFILE_4_CURR_ID:
            pr_id = message->ID - (int)MESSAGE_PROFILE_0_CURR_ID;
            log->state.prof.pr[pr_id].last_delta = MESSAGE_PROFILE_TIME_VALUE(message);
            log->state.prof.pr[pr_id].last_tag = MESSAGE_PROFILE_TAG_VALUE(message);
            log->set[PC_LOG_PR0_CURR + pr_id] = true;
            break;
        case MESSAGE_PROFILE_0_MAX_ID:
        case MESSAGE_PROFILE_1_MAX_ID:
        case MESSAGE_PROFILE_2_MAX_ID:
        case MESSAGE_PROFILE_3_MAX_ID:
        case MESSAGE_PROFILE_4_MAX_ID:
            pr_id = message->ID - (int)MESSAGE_PROFILE_0_MAX_ID;
            log->state.prof.pr[pr_id].max_delta = MESSAGE_PROFILE_TIME_VALUE(message);
            log->state.prof.pr[pr_id].max_tag = MESSAGE_PROFILE_TAG_VALUE(message);
            log->set[PC_LOG_PR0_MAX + pr_id] = true;
            break;
        default:
            break;
    }
}


void pc_log_clear(pc_log_t* log) {
	int i;
    for (i = 0; i < PC_LOG_ITEM_COUNT; i++) {
        log->set[i] = false;
    }
}

void pc_log_flush(pc_log_t* log) {
    pc_log_print(log, "%u"  _SEP, PC_LOG_time, log->time);
    pc_log_print(log, "%hhu"_SEP, PC_LOG_mode, log->mode);
    pc_log_print(log, "%d"  _SEP, PC_LOG_lift, log->state.orient.lift);
    pc_log_print(log, "%d"  _SEP, PC_LOG_roll, log->state.orient.roll);
    pc_log_print(log, "%d"  _SEP, PC_LOG_pitch, log->state.orient.pitch);
    pc_log_print(log, "%d"  _SEP, PC_LOG_yaw, log->state.orient.yaw);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae1, log->state.motor.ae1);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae2, log->state.motor.ae2);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae3, log->state.motor.ae3);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae4, log->state.motor.ae3);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sp, log->state.sensor.sp);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sq, log->state.sensor.sq);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sr, log->state.sensor.sr);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sax, log->state.sensor.sax);
    pc_log_print(log, "%d"  _SEP, PC_LOG_say, log->state.sensor.say);
    pc_log_print(log, "%d"  _SEP, PC_LOG_saz, log->state.sensor.saz);
    pc_log_print(log, "%f"  _SEP, PC_LOG_temperature, FLOAT_FP(log->state.sensor.temperature, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_pressure, FLOAT_FP(log->state.sensor.pressure, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_voltage, (float)(log->state.sensor.voltage) / 100.0f);
    pc_log_print(log, "%d"  _SEP, PC_LOG_x, log->state.pos.x);
    pc_log_print(log, "%d"  _SEP, PC_LOG_y, log->state.pos.y);
    pc_log_print(log, "%d"  _SEP, PC_LOG_z, log->state.pos.z);
    pc_log_print(log, "%f"  _SEP, PC_LOG_phi, FLOAT_FP(log->state.att.phi, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_theta, FLOAT_FP(log->state.att.theta, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_psi, FLOAT_FP(log->state.att.psi, 8));
    pc_log_print(log, "%d"  _SEP, PC_LOG_X, log->state.force.X);
    pc_log_print(log, "%d"  _SEP, PC_LOG_Y, log->state.force.Y);
    pc_log_print(log, "%d"  _SEP, PC_LOG_Z, log->state.force.Z);
    pc_log_print(log, "%d"  _SEP, PC_LOG_L, log->state.torque.L);
    pc_log_print(log, "%d"  _SEP, PC_LOG_M, log->state.torque.M);
    pc_log_print(log, "%d"  _SEP, PC_LOG_N, log->state.torque.N);
    pc_log_print(log, "%d"  _SEP, PC_LOG_u, log->state.velo.u);
    pc_log_print(log, "%d"  _SEP, PC_LOG_v, log->state.velo.v);
    pc_log_print(log, "%d"  _SEP, PC_LOG_w, log->state.velo.w);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p, log->state.spin.p);
    pc_log_print(log, "%d"  _SEP, PC_LOG_q, log->state.spin.q);
    pc_log_print(log, "%d"  _SEP, PC_LOG_r, log->state.spin.r);
    pc_log_print(log, "%d"  _SEP, PC_LOG_yaw_p, log->state.trim.yaw_p);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p1, log->state.trim.p1);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p2, log->state.trim.p2);



    for (int i = 0; i < QC_STATE_PROF_CNT; i++) {
        pc_log_print(log, "%u"_SEP"%u" _SEP, PC_LOG_PR0_CURR+i, log->state.prof.pr[i].last_delta, log->state.prof.pr[i].last_tag);
        pc_log_print(log, "%u"_SEP"%u" _SEP, PC_LOG_PR0_MAX+i, log->state.prof.pr[i].max_delta, log->state.prof.pr[i].max_tag);
    }
    fprintf(log->files->single, _END);
    fflush(log->files->single);
    #ifdef WINDOWS
        FlushFileBuffers((HANDLE) _fileno(log->files->single));
    #endif

}

void pc_log_print(pc_log_t* log, const char * fmt, pc_log_item_t item, ...) {
    if (log->set[item]) {
        // If the value exists we print it with the specified format string
        va_list argptr;
        va_start(argptr, item);
        vfprintf(log->files->single, fmt, argptr);
        va_end(argptr);
    } else {
        // If no value then print as many NANs as many %_ format arguments we got
        const char * result = fmt;
        while (result && (result = strchr(result, '%')) != 0) {
            fprintf(log->files->single, _NAN _SEP);
            if (result[1] == '\0' || result[2] == '\0') {
                result = 0;
            } else {
                result = result + 2; // Step 2 to avoid %% escape sequence
            }
        }
    }
}
