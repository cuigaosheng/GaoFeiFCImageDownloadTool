/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
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
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <ctype.h>
#include "mavlink/include/mavlink/v1.0/common/mavlink.h"

// ------------------------------------------------------------------------------
//   参数的读写
// ------------------------------------------------------------------------------

//设为2仅仅是为了和原文件一致
extern unsigned int read_param_value_time_out; 
extern unsigned int write_param_value_time_out ;



//两变量用于写参数和读取参数的同步判断,待处理的情况有两种：
//第一种：飞控无回复;
//第二种：返回的参数值和设置的不同 TODO;
//解决办法：重新向飞控设置数值。

extern char previous_write_param_id[30];
extern char previous_read_param_id[30];
extern float write_prev_value;
extern bool program_exit;
extern char status_path[];
#define WRITE_PARAM_VALUE_TIME_OUT_COUNT 200
extern int write_param_value_time_out_count;/*利用write_param_value_time_out_count对等待读取到PARAM_VALUE计数*/


#define PX4F_PATH "param596.params"
#define APMF_PATH "ALTCTL_20161202.param"

struct paras{
        char param_id[30];
        char param_value[30];
};

/*get param_value from every line for apm*/
void apm_get_params_from_line(char *_str, struct paras* __one_param);

/*get param_value from every line for px4*/
void px4_get_params_from_line(char *_str, struct paras* __one_param);

void write_status_to_file(char status);


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------
struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t param_value;
	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t highres_imu;
	uint64_t attitude;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		highres_imu = 0;
		attitude = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	//Param value
	mavlink_param_value_t param_value;
	
	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
    uint64_t write_count;

    int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;

	void read_messages();
	int  write_message(mavlink_message_t message);

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);
	int send_param_request_list();
	int send_param_request_read();
	int send_param_set(char* _param_id, float _param_value);
	//int send_param_set();

	void handle_quit( int sig );


private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	void read_thread();
	void write_thread(void);
};



#endif // AUTOPILOT_INTERFACE_H_


