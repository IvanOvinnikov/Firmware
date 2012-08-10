/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Ivan Ovinnikov <oivan@ethz.ch>
 *
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
 * @file parameter_storage.h
 * Definition of the parameter storage uORB topic.
 */

#ifndef TOPIC_PARAMETER_STORAGE_H_
#define TOPIC_STORAGE_STORAGE_H_

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

#define PX4_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN 16
#define MAX_PARAM_NAME_LEN 16


enum MODE {
	TAKEOFF = 1,
	CRUISE,
	LOITER,
	LAND
};

enum PARAM {
	PARAM_SYSTEM_ID = 0,   ///< System ID used for communication
	PARAM_COMP_ID,		   ///< Component ID
	PARAM_FLIGHT_ENV,      ///< Flight environment, indoor or outdoor
	PARAM_BATTERYVOLTAGE_CONVERSION,      ///< Conversion factor from adc measurement to millivolts. if not set the sensor app will use the default value for the ardrone board
	PARAM_PID_YAWPOS_P,
	PARAM_PID_YAWPOS_I,
	PARAM_PID_YAWPOS_D,
	PARAM_PID_YAWPOS_AWU,
	PARAM_PID_YAWPOS_LIM,
	PARAM_PID_YAWSPEED_P,
	PARAM_PID_YAWSPEED_I,
	PARAM_PID_YAWSPEED_D,
	PARAM_PID_YAWSPEED_AWU,
	PARAM_PID_YAWSPEED_LIM,
	PARAM_PID_ATT_P,
	PARAM_PID_ATT_I,
	PARAM_PID_ATT_D,
	PARAM_PID_ATT_AWU,
	PARAM_PID_ATT_LIM,
	PARAM_PID_POS_P,
	PARAM_PID_POS_I,
	PARAM_PID_POS_D,
	PARAM_PID_POS_AWU,
	PARAM_PID_POS_LIM,
	PARAM_PID_POS_Z_P,
	PARAM_PID_POS_Z_I,
	PARAM_PID_POS_Z_D,
	PARAM_PID_POS_Z_AWU,
	PARAM_PID_POS_Z_LIM,
	PARAM_AIRSPEED,
	PARAM_WPLON,
	PARAM_WPLAT,
	PARAM_WPALT,
	PARAM_FLIGHTMODE,
	PARAM_SENSOR_GYRO_XOFFSET,
	PARAM_SENSOR_GYRO_YOFFSET,
	PARAM_SENSOR_GYRO_ZOFFSET,
	PARAM_SENSOR_MAG_XOFFSET,
	PARAM_SENSOR_MAG_YOFFSET,
	PARAM_SENSOR_MAG_ZOFFSET,
	PARAM_ATT_XOFFSET,
	PARAM_ATT_YOFFSET,
	PARAM_RC1_MIN,
	PARAM_RC1_MAX,
	PARAM_RC1_TRIM,
	PARAM_RC1_REV,
	PARAM_RC2_MIN,
	PARAM_RC2_MAX,
	PARAM_RC2_TRIM,
	PARAM_RC2_REV,
	PARAM_RC3_MIN,
	PARAM_RC3_MAX,
	PARAM_RC3_TRIM,
	PARAM_RC3_REV,
	PARAM_RC4_MIN,
	PARAM_RC4_MAX,
	PARAM_RC4_TRIM,
	PARAM_RC4_REV,
	PARAM_RC5_MIN,
	PARAM_RC5_MAX,
	PARAM_RC5_TRIM,
	PARAM_RC5_REV,
	PARAM_RC6_MIN,
	PARAM_RC6_MAX,
	PARAM_RC6_TRIM,
	PARAM_RC6_REV,
	PARAM_RC7_MIN,
	PARAM_RC7_MAX,
	PARAM_RC7_TRIM,
	PARAM_RC7_REV,
	PARAM_RC8_MIN,
	PARAM_RC8_MAX,
	PARAM_RC8_TRIM,
	PARAM_RC8_REV,
	PARAM_THROTTLE_CHAN,
	PARAM_ROLL_CHAN,
	PARAM_PITCH_CHAN,
	PARAM_YAW_CHAN,
	PARAM_OVERRIDE_CHAN,
	PARAM_SERVO1_MIN,
	PARAM_SERVO1_MAX,
	PARAM_SERVO1_TRIM,
	PARAM_SERVO2_MIN,
	PARAM_SERVO2_MAX,
	PARAM_SERVO2_TRIM,
	PARAM_SERVO3_MIN,
	PARAM_SERVO3_MAX,
	PARAM_SERVO3_TRIM,
	PARAM_SERVO4_MIN,
	PARAM_SERVO4_MAX,
	PARAM_SERVO4_TRIM,
	PARAM_SERVO_SCALE,
	PARAM_MAX_COUNT        ///< LEAVE THIS IN PLACE AS LAST ELEMENT!
};

struct px4_parameter_storage_t {
	float param_values[PARAM_MAX_COUNT];      ///< Parameter values
	const char *param_names[PARAM_MAX_COUNT]; ///< Parameter names
	bool param_needs_write[PARAM_MAX_COUNT];
	uint16_t next_param;
	uint16_t size;
};


#define PX4_FLIGHT_ENVIRONMENT_INDOOR 0
#define PX4_FLIGHT_ENVIRONMENT_OUTDOOR 1
#define PX4_FLIGHT_ENVIRONMENT_TESTING 2 //NO check for position fix in this environment

/**
 * Parameter storage structure
 */

struct global_data_parameter_storage_t {

	/* use of a counter and timestamp recommended (but not necessary) */

//	uint16_t counter; //incremented by the writing thread everytime new data is stored
//	uint64_t timestamp; //in microseconds since system start, is set whenever the writing thread stores new data

	/* Actual data, this is specific to the type of data which is stored in this struct */

	//***** Start: Add your variables here *****

	/* Parameters (set by a param_set mavlink message */
	struct px4_parameter_storage_t pm;

	//*****END: Add your variables here *****

};
/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(parameter_storage);

#endif //TOPIC_PARAMETER_STORAGE_T_H_
