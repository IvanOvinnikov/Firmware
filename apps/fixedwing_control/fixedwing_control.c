/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Ivan Ovinnikov <oivan@ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file fixedwing_control.c
 * Implementation of a fixed wing attitude and position controller.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <math.h>
#include <termios.h>
#include <time.h>
#include <arch/board/up_hrt.h>
#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <nuttx/spi.h>
#include "../mix_and_link/mix_and_link.h" //TODO: add to Makefile
#include <uORB/uORB.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/fixedwing_control.h>

#ifndef F_M_PI
#define F_M_PI ((float)M_PI)
#endif

__EXPORT int fixedwing_control_main(int argc, char *argv[]);

#define PID_DT 5e-3
#define PID_SCALER 1.0f
#define PID_DERIVMODE_CALC 0
#define HIL_MODE 32
#define AUTO -1000
#define MANUAL 3000
#define SERVO_MIN 1000
#define SERVO_MAX 2000

/**
 * Servo channels function enumerator used for
 * the servo writing part
 */
enum SERVO_CHANNELS_FUNCTION {

	AIL_1    = 0,
	AIL_2    = 1,
	MOT      = 2,
	ACT_1    = 3,
	ACT_2    = 4,
	ACT_3    = 5,
	ACT_4    = 6,
	ACT_5    = 7
};

/**
 * The plane_data structure.
 *
 * The plane data structure is the local storage of all the flight information of the aircraft
 */
typedef struct {
	int32_t lat;
	int32_t lon;
	float alt;
	float vx;
	float vy;
	float vz;
	float yaw;
	float hdg;
	float pitch;
	float roll;
	float yawspeed;
	float pitchspeed;
	float rollspeed;
	float rollb;	/* body frame angles */
	float pitchb;
	float yawb;
	float p;
	float q;
	float r;	/* body angular rates */

	/* Next waypoint*/

	int32_t wp_x;
	int32_t wp_y;
	float wp_z;

	/* Setpoints */

	float airspeed;
	float groundspeed;
	float roll_setpoint;
	float pitch_setpoint;
	float throttle_setpoint;

	/* Navigation mode*/
	int mode;

} plane_data_t;

/**
 * The PID structure.
 */

typedef struct {
	/* PID parameters*/

	float Kp_att;
	float Ki_att;
	float Kd_att;
	float Kp_pos;
	float Ki_pos;
	float Kd_pos;
	float intmax_att;
	float intmax_pos;
	float lerror;
	float lderiv;

} pid_s_t;

/*
 * Rotation matrix structure
 */

typedef struct {

	float m11;
	float m12;
	float m13;
	float m21;
	float m22;
	float m23;
	float m31;
	float m32;
	float m33;

} rmatrix_t;

/**
 * The control_outputs structure.
 *
 * The control outputs structure contains the control outputs
 * of the aircraft
 */
typedef struct {
	float roll_ailerons;
	float pitch_elevator;
	float yaw_rudder;
	float throttle;
	// set the aux values to 0 per default
	float aux1;
	float aux2;
	float aux3;
	float aux4;
	uint8_t mode;	// HIL_ENABLED: 32
	uint8_t nav_mode;
} control_outputs_t;

/**
 * Generic PID algorithm with PD scaling
 */
static float pid(float error, float error_deriv, float dt, float scaler, float K_p, float K_i, float K_d, float intmax, float lerror, float lderiv);

/*
 * Output calculations
 */

static void calc_body_angular_rates(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
static void calc_rotation_matrix(rmatrix_t * rmatrix, float roll, float pitch, float yaw);
static void calc_angles_from_rotation_matrix(rmatrix_t * rmatrix, float roll, float pitch, float yaw);
static void multiply_matrices(rmatrix_t* rmatrix1, rmatrix_t* rmatrix2, rmatrix_t *rmatrix3);
static float calc_bearing(plane_data_t *plane_data);
static float calc_roll_ail(plane_data_t *plane_data, pid_s_t * pid_s);
static float calc_pitch_elev(plane_data_t *plane_data, pid_s_t * pid_s);
static float calc_yaw_rudder(plane_data_t *plane_data, pid_s_t * pid_s);
static float calc_throttle(plane_data_t *plane_data, pid_s_t * pid_s);
static float calc_gnd_speed(plane_data_t *plane_data);
static void get_parameters(plane_data_t *plane_data, pid_s_t * pid_s);
static float calc_roll_setpoint(float sp, plane_data_t *plane_data);
static float calc_pitch_setpoint(float sp, plane_data_t *plane_data);
static float calc_throttle_setpoint(plane_data_t *plane_data);
static float calc_wp_distance(plane_data_t *plane_data);
static void set_plane_mode(plane_data_t *plane_data);

/****************************************************************************
 * Public Data
 ****************************************************************************/

/*
 * Global structure declarations
 */
plane_data_t plane_data;
control_outputs_t control_outputs;
pid_s_t pid_s;

/*
 * Rotation matrices for the control, attitude, and body frame attitude angles
 */
rmatrix_t rmatrix_att, rmatrix_c, rmatrix_b;

float scaler = 1; //M_PI;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * Calculates the PID control output given an error.
 * 
 * Incorporates PD scaling and low-pass filter for the derivative component.
 *
 * @param error the input error
 * @param error_deriv the derivative of the input error
 * @param dt time constant
 * @param scale PD scaler
 * @param Kp P gain
 * @param Ki I gain
 * @param Kd D gain
 * @param intmax Integration limit
 * @param lerror Last error
 * @param lderiv Last error derivative
 *
 * @return the PID control output
 */

static float pid(float error, float error_deriv, float dt, float scale, float Kp, float Ki, float Kd, float intmax, float lerror, float lderiv)
{
	float delta_time = dt;
	float imax = intmax;
	float integrator;
	float derivative;
	int fCut = 20;		/* anything above 20 Hz is considered noise - low pass filter for the derivative */
	float output = 0;

	output += error * Kp;

	if ((fabsf(Kd) > 0) && (dt > 0)) {

		if (PID_DERIVMODE_CALC) {
			derivative = (error - lerror) / delta_time;

			/*
			 * discrete low pass filter, cuts out the
			 * high frequency noise that can drive the controller crazy
			 */
			float RC = 1.0 / (2.0f * F_M_PI * fCut);
			derivative = lderiv +
				     (delta_time / (RC + delta_time)) * (derivative - lderiv);

			/* update state */
			pid_s.lerror = error;
			pid_s.lderiv  = derivative;

		} else {
			derivative = -error_deriv;
		}

		/* add in derivative component */
		output 	+= Kd * derivative;
	}

	/* scale the P and D components with the PD scaler */
	output *= scale;

	/* Compute integral component if time has elapsed */
	if ((fabsf(Ki) > 0) && (dt > 0)) {
		integrator 		+= (error * Ki) * scaler * delta_time;

		if (integrator < -imax) {
			integrator = -imax;

		} else if (integrator > imax) {
			integrator = imax;
		}

		output += integrator;
	}
	return output;
}

/**
 * Load parameters from global storage.
 *
 * @param plane_data Fixed wing data structure
 * @param pid_s PID structure
 *
 * Fetches the current parameters from the global parameter storage and writes them
 * to the plane_data structure
 */

static void get_parameters(plane_data_t * plane_data, pid_s_t * pid_s)
{
	pid_s->Kp_att = global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_P];
	pid_s->Ki_att = global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_I];
	pid_s->Kd_att = global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_D];
	pid_s->Kp_pos = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_P];
	pid_s->Ki_pos = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_I];
	pid_s->Kd_pos = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_D];
	pid_s->intmax_att = global_data_parameter_storage->pm.param_values[PARAM_PID_ATT_AWU];
	pid_s->intmax_pos = global_data_parameter_storage->pm.param_values[PARAM_PID_POS_AWU];
	plane_data->airspeed =  global_data_parameter_storage->pm.param_values[PARAM_AIRSPEED];
	plane_data->wp_x =  global_data_parameter_storage->pm.param_values[PARAM_WPLON];
	plane_data->wp_y =  global_data_parameter_storage->pm.param_values[PARAM_WPLAT];
	plane_data->wp_z =  global_data_parameter_storage->pm.param_values[PARAM_WPALT];
	plane_data->mode = global_data_parameter_storage->pm.param_values[PARAM_FLIGHTMODE];
}

/**
 * Calculates the body angular rates.
 *
 * Calculates the rates of the plane using inertia matrix and
 * writes them to the plane_data structure
 *
 * @param roll
 * @param pitch
 * @param yaw
 * @param rollspeed
 * @param pitchspeed
 * @param yawspeed
 *
 */
static void calc_body_angular_rates(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
{
	plane_data.p = rollspeed - sinf(pitch) * yawspeed;
	plane_data.q = cosf(roll) * pitchspeed + sinf(roll) * cosf(pitch) * yawspeed;
	plane_data.r = -sinf(roll) * pitchspeed + cosf(roll) * cosf(pitch) * yawspeed;
}

/**
 * calc_rotation_matrix
 *
 * Calculates the rotation matrix
 *
 * @param rmatrix Rotation matrix structure
 * @param roll
 * @param pitch
 * @param yaw
 *
 */

static void calc_rotation_matrix(rmatrix_t * rmatrix, float roll, float pitch, float yaw)
{
	rmatrix->m11 = cosf(yaw) * cosf(pitch);
	rmatrix->m12 = (cosf(yaw) * sinf(pitch) * sinf(roll) + sinf(yaw) * cosf(roll));
	rmatrix->m13 = (-cosf(yaw) * sinf(pitch) * cosf(roll)  + sinf(yaw) * sinf(roll));
	rmatrix->m21 = -sinf(yaw) * cosf(pitch);
	rmatrix->m22 = (-sinf(yaw) * sinf(pitch) * sinf(roll) + cosf(yaw) * cosf(roll));
	rmatrix->m23 = (sinf(yaw) * sinf(pitch) * cosf(roll) + cosf(yaw) * sinf(roll));
	rmatrix->m31 = sinf(pitch);
	rmatrix->m32 = -cosf(pitch) * sinf(roll);
	rmatrix->m33 = cosf(pitch) * cosf(roll);
}

/**
 * calc_angles_from_rotation_matrix
 *
 * Calculates the angles from a rotation matrix
 *
 * @param rmatrix Rotation matrix structure
 * @param roll
 * @param pitch
 * @param yaw
 *
 */

static void calc_angles_from_rotation_matrix(rmatrix_t * rmatrix, float roll, float pitch, float yaw)
{
	// TODO
}

/**
 * multiply_matrices
 *
 * Multiply two rotation matricec
 *
 * @param rmatrix1 Rotation matrix 1
 * @param rmatrix2 Rotation matrix 2
 * @param rmatrix3 Rotation matrix output
 *
 *
 */

static void multiply_matrices(rmatrix_t * rmatrix1, rmatrix_t * rmatrix2, rmatrix_t * rmatrix3)
{
	// TODO
}


/**
 * calc_bearing
 *
 * Calculates the bearing error of the plane compared to the waypoints

 * @param plane_data Plane data structure
 *
 * @return bearing Bearing error
 *
 */
static float calc_bearing(plane_data_t * plane_data)
{
	float bearing = F_M_PI/2.0f + atan2f(-(plane_data->wp_y - plane_data->lat), (plane_data->wp_x - plane_data->lon));

	if (bearing < 0.0f) {
		bearing += 2*F_M_PI;
	}
	return bearing;
}

/**
 * calc_roll_ail
 *
 * Calculates the roll ailerons control output
 *
 * @param plane_data Plane data structure
 * @param pid_s PID structure
 *
 * @return Roll ailerons control output (-1,1)
 */

static float calc_roll_ail(plane_data_t *plane_data, pid_s_t *pid_s)
{
	float ret = pid((plane_data->roll_setpoint - plane_data->roll), plane_data->rollspeed, PID_DT, PID_SCALER,
			pid_s->Kp_att, pid_s->Ki_att, pid_s->Kd_att, pid_s->intmax_att, pid_s->lerror, pid_s->lderiv);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_pitch_elev
 *
 * Calculates the pitch elevators control output
 *
 * @param plane_data Plane data structure
 * @param pid_s PID structure
 *
 * @return Pitch elevators control output (-1,1)
 */
static float calc_pitch_elev(plane_data_t *plane_data, pid_s_t * pid_s)
{
	float ret = pid((plane_data->pitch_setpoint - plane_data->pitch), plane_data->pitchspeed, PID_DT, PID_SCALER,
			pid_s->Kp_att, pid_s->Ki_att, pid_s->Kd_att, pid_s->intmax_att, pid_s->lerror, pid_s->lderiv);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_yaw_rudder
 *
 * Calculates the yaw rudder control output (only if yaw rudder exists on the model)
 *
 * @param plane_data Plane data structure
 * @param pid_s PID structure
 *
 * @return Yaw rudder control output (-1,1)
 */
static float calc_yaw_rudder(plane_data_t *plane_data, pid_s_t *pid_s)
{
	float ret = pid((plane_data->yaw - plane_data->yaw), plane_data->yawspeed, PID_DT, PID_SCALER,
			pid_s->Kp_pos, pid_s->Ki_pos, pid_s->Kd_pos, pid_s->intmax_pos, pid_s->lerror, pid_s->lderiv);

	if (ret < -1)
		return -1;

	if (ret > 1)
		return 1;

	return ret;
}

/**
 * calc_throttle
 *
 * Calculates the throttle control output
 *
 * @param plane_data Plane data structure
 * @param pid_s PID structure
 *
 * @return Throttle control output (0,1)
 */

static float calc_throttle(plane_data_t * plane_data, pid_s_t *pid_s)
{
	float ret = pid(plane_data->throttle_setpoint - calc_gnd_speed(plane_data), 0, PID_DT, PID_SCALER,
			pid_s->Kp_pos, pid_s->Ki_pos, pid_s->Kd_pos, pid_s->intmax_pos, pid_s->lerror, pid_s->lderiv);

	if (ret < 0.2f)
		return 0.2f;

	if (ret > 1.0f)
		return 1.0f;

	return ret;
}

/**
 * calc_gnd_speed
 *
 * Calculates the ground speed using the x and y components
 *
 * @param plane_data Plane data structure
 *
 * Input: none (operation on global data)
 *
 * Output: Ground speed of the plane
 *
 */

static float calc_gnd_speed(plane_data_t * plane_data)
{
	float gnd_speed = sqrtf(plane_data->vx * plane_data->vx + plane_data->vy * plane_data->vy);
	return gnd_speed;
}

/**
 * calc_wp_distance
 *
 * Calculates the distance to the next waypoint
 *
 * @param plane_data Plane data structure
 *
 * @return the distance to the next waypoint
 *
 */

static float calc_wp_distance(plane_data_t * plane_data)
{
	return sqrtf((plane_data->lat - plane_data->wp_x) * (plane_data->lat - plane_data->wp_y) +
		     (plane_data->lon - plane_data->wp_x) * (plane_data->lon - plane_data->wp_x));
}

/**
 * calc_roll_setpoint
 *
 * Calculates the offset angle for the roll plane,
 * saturates at +- 35 deg.
 *
 * @param sp Desired setpoint
 * @param plane_data Plane data structure
 *
 * @return setpoint on which attitude control should stabilize while changing heading
 *
 */

static float calc_roll_setpoint(float sp, plane_data_t *plane_data)
{
	float setpoint = 0.0f;

	if (plane_data->mode == TAKEOFF) {
		setpoint = 0.0f;

	} else {
		setpoint = calc_bearing(&plane_data) - plane_data->yaw;

		if (setpoint < (-sp/180.0f)*F_M_PI)
			return (-sp/180.0f)*F_M_PI;

		if (setpoint > (sp/180.0f)*F_M_PI)
			return (sp/180.0f)*F_M_PI;
	}

	return setpoint;
}

/**
 * calc_pitch_setpoint
 *
 * Calculates the offset angle for the pitch plane
 * saturates at +- 35 deg.
 *
 * @param sp Desired setpoint
 * @param plane_data Plane data structure
 *
 * @return setpoint on which attitude control should stabilize while changing altitude
 *
 */

static float calc_pitch_setpoint(float sp, plane_data_t * plane_data)
{
	float setpoint = 0.0f;

	if (plane_data->mode == TAKEOFF) {
		setpoint = ((sp/180.0f)*F_M_PI);

	} else {
		setpoint = atanf((plane_data->wp_z - plane_data->alt) / calc_wp_distance(plane_data));

		if (setpoint < (-sp/180.0f)*F_M_PI)
			return (-sp/180.0f)*F_M_PI;

		if (setpoint > (sp/180.0f)*F_M_PI)
			return (sp/180.0f)*F_M_PI;
	}

	return setpoint;
}

/**
 * calc_throttle_setpoint
 *
 * Calculates the throttle setpoint for different flight modes
 *
 * @param plane_data Plane data structure
 *
 * @return throttle output setpoint
 *
 */

static float calc_throttle_setpoint(plane_data_t * plane_data)
{
	float setpoint = 0;

	// if TAKEOFF full throttle
	if (plane_data->mode == TAKEOFF) {
		setpoint = 60.0f;
	}

	// if CRUISE - parameter value
	if (plane_data->mode == CRUISE) {
		setpoint = plane_data->airspeed;
	}

	// if LAND no throttle
	if (plane_data->mode == LAND) {
		setpoint = 0.0f;
	}

	return setpoint;
}

/**
 * set_plane_mode
 *
 * @param plane_data Plane data structure
 *
 * Sets the plane mode
 * (TAKEOFF, CRUISE, LOITER or LAND)
 *
 */

static void set_plane_mode(plane_data_t *plane_data)
{
	// TODO: add flag to includes
	if (plane_data->alt < 10.0f/* && MAV_MODE_FLAG_HIL_ENABLED == 32*/) {
		plane_data->mode = TAKEOFF;

	} else {
		plane_data->mode = CRUISE;
		// TODO: if reached waypoint and no further waypoint exists, go to LOITER mode
	}
	// Debug override
	plane_data->mode = CRUISE;
}

/*
 * fixedwing_control_main
 *
 * @param argc number of arguments
 * @param argv argument array
 *
 * @return 0
 *
 */

int fixedwing_control_main(int argc, char *argv[])
{
	/* default values for arguments */
	char *fixedwing_uart_name = "/dev/ttyS1";
	char *commandline_usage = "\tusage: %s -d fixedwing-devicename\n";

	/* read arguments */
	bool verbose = false;

	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				fixedwing_uart_name = argv[i + 1];

			} else {
				printf(commandline_usage, argv[0]);
				return 0;
			}
		}
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0) {
			verbose = true;
		}
	}

	/* welcome user */
	printf("[fixedwing control] started\n");

	/* Set up to publish fixed wing control messages */
	struct fixedwing_control_s control;
	int fixedwing_control_pub = orb_advertise(ORB_ID(fixedwing_control), &control);

	/* Subscribe to global position, attitude and rc */
	struct vehicle_global_position_s global_pos;
	int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_attitude_s att;
	int attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	struct rc_channels_s rc;
	int rc_sub = orb_subscribe(ORB_ID(rc_channels));
	struct vehicle_global_position_setpoint_s global_setpoint;
	int global_setpoint_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));

	/* Mainloop setup */
	unsigned int loopcounter = 0;
	unsigned int failcounter = 0;

	/* Control constants */
	control_outputs.mode = HIL_MODE;
	control_outputs.nav_mode = 0;

	/* Servo setup */

	int fd;
	servo_position_t data[PWM_OUTPUT_MAX_CHANNELS];

	fd = open("/dev/pwm_servo", O_RDWR);

	if (fd < 0) {
		printf("[fixedwing control] Failed opening /dev/pwm_servo\n");
	}

	ioctl(fd, PWM_SERVO_ARM, 0);

	int16_t buffer_rc[3];
	int16_t buffer_servo[3];
	mixer_data_t mixer_buffer;
	mixer_buffer.input  = buffer_rc;
	mixer_buffer.output = buffer_servo;

	mixer_conf_t mixers[3];

	mixers[0].source = PITCH;
	mixers[0].nr_actuators = 2;
	mixers[0].dest[0] = AIL_1;
	mixers[0].dest[1] = AIL_2;
	mixers[0].dual_rate[0] = 1;
	mixers[0].dual_rate[1] = 1;

	mixers[1].source = ROLL;
	mixers[1].nr_actuators = 2;
	mixers[1].dest[0] = AIL_1;
	mixers[1].dest[1] = AIL_2;
	mixers[1].dual_rate[0] = 1;
	mixers[1].dual_rate[1] = -1;

	mixers[2].source = THROTTLE;
	mixers[2].nr_actuators = 1;
	mixers[2].dest[0] = MOT;
	mixers[2].dual_rate[0] = 1;

	/*
	 * Main control, navigation and servo routine
	 */

	while(1) {
		/*
		 * DATA Handling
		 * Fetch current flight data
		 */

		/* get position, attitude and rc inputs */
		// XXX add error checking
		orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
		orb_copy(ORB_ID(vehicle_global_position_setpoint), global_setpoint_sub, &global_setpoint);
		orb_copy(ORB_ID(vehicle_attitude), attitude_sub, &att);
		orb_copy(ORB_ID(rc_channels), rc_sub, &rc);

		/* scaling factors are defined by the data from the APM Planner
		 * TODO: ifdef for other parameters (HIL/Real world switch)
		 */

		/* position values*/
		plane_data.lat = global_pos.lat; /// 10000000.0;
		plane_data.lon = global_pos.lon; /// 10000000.0;
		plane_data.alt = global_pos.alt; /// 1000.0f;
		plane_data.vx = global_pos.vx / 100.0f;
		plane_data.vy = global_pos.vy / 100.0f;
		plane_data.vz = global_pos.vz / 100.0f;

		/* attitude values*/
		plane_data.roll = att.roll;
		plane_data.pitch = att.pitch;
		plane_data.yaw = att.yaw;
		plane_data.rollspeed = att.rollspeed;
		plane_data.pitchspeed = att.pitchspeed;
		plane_data.yawspeed = att.yawspeed;

		/* parameter values */
		get_parameters(&plane_data, &pid_s);

		/* Attitude control part */

		if (verbose && loopcounter % 20 == 0) {
			/******************************** DEBUG OUTPUT ************************************************************/

			printf("Parameter: %i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i,%i \n", (int)pid_s.Kp_att, (int)pid_s.Ki_att,
					(int)pid_s.Kd_att, (int)pid_s.intmax_att, (int)pid_s.Kp_pos, (int)pid_s.Ki_pos,
					(int)pid_s.Kd_pos, (int)pid_s.intmax_pos, (int)plane_data.airspeed,
					(int)plane_data.wp_x, (int)plane_data.wp_y, (int)plane_data.wp_z,
					(int)global_data_parameter_storage->pm.param_values[PARAM_WPLON],
					(int)global_data_parameter_storage->pm.param_values[PARAM_WPLAT],
					(int)global_data_parameter_storage->pm.param_values[PARAM_WPALT],
					global_setpoint.lat,
					global_setpoint.lon,
					(int)global_setpoint.altitude);

			printf("PITCH SETPOINT: %i\n", (int)(100.0f*plane_data.pitch_setpoint));
			printf("ROLL SETPOINT: %i\n", (int)(100.0f*plane_data.roll_setpoint));
			printf("THROTTLE SETPOINT: %i\n", (int)(100.0f*plane_data.throttle_setpoint));

			printf("\n\nVx: %i\t Vy: %i\t Current speed:%i\n\n", (int)plane_data.vx, (int)plane_data.vy, (int)(calc_gnd_speed(&plane_data)));

			printf("Current Position: \n %i \n %i \n %i \n", (int)plane_data.lat, (int)plane_data.lon, (int)plane_data.alt);

			printf("\nAttitude values: \n R:%i \n P: %i \n Y: %i \n\n RS: %i \n PS: %i \n YS: %i \n ",
					(int)(180.0f * plane_data.roll/F_M_PI), (int)(180.0f * plane_data.pitch/F_M_PI), (int)(180.0f * plane_data.yaw/F_M_PI),
					(int)(180.0f * plane_data.rollspeed/F_M_PI), (int)(180.0f * plane_data.pitchspeed/F_M_PI), (int)(180.0f * plane_data.yawspeed)/F_M_PI);

			printf("\nCalculated outputs: \n R: %i\n P: %i\n Y: %i\n T: %i \n",
					(int)(10000.0f * control_outputs.roll_ailerons), (int)(10000.0f * control_outputs.pitch_elevator),
					(int)(10000.0f * control_outputs.yaw_rudder), (int)(10000.0f * control_outputs.throttle));

			/************************************************************************************************************/
		}

		/*
		 * Computation section
		 *
		 * The function calls to compute the required control values happen
		 * in this section.
		 */

		/* Set plane mode */
		set_plane_mode(&plane_data);

		/* Calculate the output values */
		control_outputs.roll_ailerons = calc_roll_ail(&plane_data, &pid_s);
		control_outputs.pitch_elevator = calc_pitch_elev(&plane_data, &pid_s);
		control_outputs.yaw_rudder = calc_yaw_rudder(&plane_data, &pid_s);
		control_outputs.throttle = calc_throttle(&plane_data, &pid_s);

		/*
		 * Calculate rotation matrices
		 */
		calc_rotation_matrix(&rmatrix_att, plane_data.roll, plane_data.pitch, plane_data.yaw);
		calc_rotation_matrix(&rmatrix_c, control_outputs.roll_ailerons, control_outputs.pitch_elevator, control_outputs.yaw_rudder);

		multiply_matrices(&rmatrix_att, &rmatrix_c, &rmatrix_b);

		calc_angles_from_rotation_matrix(&rmatrix_b, plane_data.rollb, plane_data.pitchb, plane_data.yawb);


		// TODO: fix RC input
		//if (rc.chan[rc.function[OVERRIDE]].scale < MANUAL) { // if we're flying in automated mode

			/*
			 * TAKEOFF hack for HIL
			 */
			if (plane_data.mode == TAKEOFF) {
				control.attitude_control_output[ROLL] = 0;
				control.attitude_control_output[PITCH] = 5000;
				control.attitude_control_output[THROTTLE] = 10000;
				//global_data_fixedwing_control->attitude_control_output[YAW] = (int16_t)(control_outputs.yaw_rudder);
			}

			if (plane_data.mode == CRUISE) {

				// TODO: substitute output with the body angles (rollb, pitchb)
				control.attitude_control_output[ROLL] = control_outputs.roll_ailerons;
				control.attitude_control_output[PITCH] = control_outputs.pitch_elevator;
				control.attitude_control_output[THROTTLE] = control_outputs.throttle;
				//control->attitude_control_output[YAW] = (int16_t)(control_outputs.yaw_rudder);
			}

			control.counter++;
			control.timestamp = hrt_absolute_time();
		//}

		/* Navigation part */

		/*
		 * TODO: APM Planner Waypoint communication
		 */
		// Get GPS Waypoint

		//	plane_data.wp_x = global_setpoint.lon;
		//	plane_data.wp_y = global_setpoint.lat;
		//	plane_data.wp_z = global_setpoint.altitude;

		/*
		 * TODO: fix RC input
		 */
		//if (rc.chan[rc.function[OVERRIDE]].scale < MANUAL) {	// AUTO mode
			// AUTO/HYBRID switch

			//if (rc.chan[rc.function[OVERRIDE]].scale < AUTO) {
				plane_data.roll_setpoint = calc_roll_setpoint(35.0f, &plane_data);
				plane_data.pitch_setpoint = calc_pitch_setpoint(35.0f, &plane_data);
				plane_data.throttle_setpoint = calc_throttle_setpoint(&plane_data);

//			} else {
//				plane_data.roll_setpoint = rc.chan[rc.function[ROLL]].scale / 200;
//				plane_data.pitch_setpoint = rc.chan[rc.function[PITCH]].scale / 200;
//				plane_data.throttle_setpoint = rc.chan[rc.function[THROTTLE]].scale / 200;
//			}

			//control_outputs.yaw_rudder = calc_yaw_rudder(plane_data.hdg);

//		} else {
//			control.attitude_control_output[ROLL] = rc.chan[rc.function[ROLL]].scale/10000;
//			control.attitude_control_output[PITCH] = rc.chan[rc.function[PITCH]].scale/10000;
//			control.attitude_control_output[THROTTLE] = rc.chan[rc.function[THROTTLE]].scale/10000;
			// since we don't have a yaw rudder
			//control.attitude_control_output[YAW] = rc.chan[rc.function[YAW]].scale/10000;

			control.counter++;
			control.timestamp = hrt_absolute_time();
		//}

		/* publish the control data */

		orb_publish(ORB_ID(fixedwing_control), fixedwing_control_pub, &control);

		/* Servo part */

		buffer_rc[ROLL] = (int16_t)(10000*control.attitude_control_output[ROLL]);
		buffer_rc[PITCH] = (int16_t)(10000*control.attitude_control_output[PITCH]);
		buffer_rc[THROTTLE] = (int16_t)(10000*control.attitude_control_output[THROTTLE]);

		//mix_and_link(mixers, 3, 2, &mixer_buffer);

		// Scaling and saturation of servo outputs happens here

		data[AIL_1] = buffer_servo[AIL_1] / global_data_parameter_storage->pm.param_values[PARAM_SERVO_SCALE]
		                                  + global_data_parameter_storage->pm.param_values[PARAM_SERVO1_TRIM];

		if (data[AIL_1] > SERVO_MAX)
			data[AIL_1] = SERVO_MAX;

		if (data[AIL_1] < SERVO_MIN)
			data[AIL_1] = SERVO_MIN;

		data[AIL_2] = buffer_servo[AIL_2] / global_data_parameter_storage->pm.param_values[PARAM_SERVO_SCALE]
		                                  + global_data_parameter_storage->pm.param_values[PARAM_SERVO2_TRIM];

		if (data[AIL_2] > SERVO_MAX)
			data[AIL_2] = SERVO_MAX;

		if (data[AIL_2] < SERVO_MIN)
			data[AIL_2] = SERVO_MIN;

		data[MOT] = buffer_servo[MOT] / global_data_parameter_storage->pm.param_values[PARAM_SERVO_SCALE]
		                              + global_data_parameter_storage->pm.param_values[PARAM_SERVO3_TRIM];

		if (data[MOT] > SERVO_MAX)
			data[MOT] = SERVO_MAX;

		if (data[MOT] < SERVO_MIN)
			data[MOT] = SERVO_MIN;

		int result = write(fd, &data, sizeof(data));

		if (result != sizeof(data)) {
			if (failcounter < 10 || failcounter % 20 == 0) {
				printf("[fixedwing_control] failed writing servo outputs\n");
			}
			failcounter++;
		}

		loopcounter++;

		/* 20Hz loop*/
		usleep(50000);
	}

	return 0;
}
