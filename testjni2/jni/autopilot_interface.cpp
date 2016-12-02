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
 * @file autopilot_interface.cpp
 *
 * @brief Autopilot interface functions
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_interface.h"

unsigned int read_param_value_time_out = 2;
unsigned int write_param_value_time_out = 2;

char previous_write_param_id[30] = {};
char previous_read_param_id[30] = {};
char status_path[30] = {};

int write_param_value_time_out_count = WRITE_PARAM_VALUE_TIME_OUT_COUNT;
float write_prev_value;
bool program_exit = false;

/*
status有3种，分别是:
0:firmware和param都未写.
1:firmware已写，parma未写.
2:firmware已写，param已写.
*/
void write_status_to_file(char status){
	if(NULL == status_path){
		printf("status_path 为空\n");
	}
	else{
        	printf("%s\n", status_path);
	}
		
        FILE *fp = fopen(status_path, "w+r");
        fwrite(&status, sizeof(char), 1, fp);
        printf("write_status_to_file\n");
        fclose(fp);

        return;
}

//get param_value from every line
void
apm_get_params_from_line(char *_str, struct paras* __one_param)
{
        char *i = NULL;
        char *j = NULL;
        char *k = NULL;

        i = _str;/*apm参数表中无mav_id和component_id.*/
        j = __one_param->param_id;
        k = __one_param->param_value;

        while((*i)!=','){
                *(j++) = *i++;
        }

        *j = '\0'; /*每一个字符串都需要'\0'来结尾*/
        i++; /*跳过参数名和值之间的空格*/

        while(!isspace(*i)){
                *(k++) = *i++;
        }
        *k = '\0';

        //printf("%s %s\n", _param_one->param_id, _param_one->param_value);

        return;
}


//get param_value from every line*
void
px4_get_params_from_line(char *_str, struct paras* __one_param)
{
        char *i = NULL;
        char *j = NULL;
        char *k = NULL;

        i = _str+4;/*去掉了mav_id还有component_id*/
        j = __one_param->param_id;
        k = __one_param->param_value;

        while(!isspace(*i)){
                *(j++) = *i++;
        }
        *j = '\0';

        i++; /*跳过参数名和值之间的空格*/

        while(!isspace(*i)){
                *(k++) = *i++;
        }
        *k = '\0';
        //printf("%s %s\n", _param_one->param_id, _param_one->param_value);

        return;
}


// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
uint64_t
get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Autopilot_Interface::
Autopilot_Interface(Serial_Port *serial_port_)
{
	// initialize attributes
	write_count = 0;

	reading_status = 0;      // whether the read thread is running
	writing_status = 0;      // whether the write thread is running
	time_to_exit   = false;  // flag to signal thread exit

	read_tid  = 0; // read thread id
	write_tid = 0; // write thread id

	system_id    = 0; // system id
	autopilot_id = 0; // autopilot component id
	companion_id = 0; // companion computer component id

	current_messages.sysid  = system_id;
	current_messages.compid = autopilot_id;

	serial_port = serial_port_; // serial port management object

}

Autopilot_Interface::
~Autopilot_Interface()
{}

// ------------------------------------------------------------------------------
//   Read Messages
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_messages()
{
	printf("Auto::read_messages()\n");
	bool success;               // receive success flag
	bool received_all = false;  // receive only one message
	Time_Stamps this_timestamps;

	// Blocking wait for new data
	while ( !received_all and !time_to_exit )
	{
		// ----------------------------------------------------------------------
		//   READ MESSAGE
		// ----------------------------------------------------------------------
		mavlink_message_t message;
		success = serial_port->read_message(message);

		// ----------------------------------------------------------------------
		//   HANDLE MESSAGE
		// ----------------------------------------------------------------------
		if( success )
		{
			// Store message sysid and compid.
			// Note this doesn't handle multiple message sources.
			current_messages.sysid  = message.sysid;
			current_messages.compid = message.compid;

			// Handle Message ID
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_PARAM_VALUE:
				{
					//printf("MAVLINK_MSG_ID_PARAM_VALUE\n");
					mavlink_msg_param_value_decode(&message, &(current_messages.param_value));
					current_messages.time_stamps.param_value = get_time_usec();
					this_timestamps.param_value = current_messages.time_stamps.param_value;

					//只比较参数的前14位字符，14位之后有乱码。乱码问题待改善 TODO
					if(!strncmp(previous_write_param_id, current_messages.param_value.param_id,14))
					{
						if(strcmp(previous_read_param_id, current_messages.param_value.param_id))
						{
							//本次读到的参数名和上一次读到的参数名一致.
							read_param_value_time_out++; 
						}
						
						//将本次读到的参数名保存到上一次读到的参数中.
						strcpy(previous_read_param_id, current_messages.param_value.param_id);
					}

					printf("写=%s 读=%s 参数值=%f 写计数=%d 读计数==%d\n",\
						 previous_write_param_id, current_messages.param_value.param_id, \
						current_messages.param_value.param_value, write_param_value_time_out, read_param_value_time_out);

					break;
				}

				case MAVLINK_MSG_ID_HEARTBEAT:
				{
					//printf("MAVLINK_MSG_ID_HEARTBEAT\n");
					mavlink_msg_heartbeat_decode(&message, &(current_messages.heartbeat));
					current_messages.time_stamps.heartbeat = get_time_usec();
					this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
					break;
				}

				case MAVLINK_MSG_ID_SYS_STATUS:
				{
					//printf("MAVLINK_MSG_ID_SYS_STATUS\n");
					mavlink_msg_sys_status_decode(&message, &(current_messages.sys_status));
					current_messages.time_stamps.sys_status = get_time_usec();
					this_timestamps.sys_status = current_messages.time_stamps.sys_status;
					break;
				}

				case MAVLINK_MSG_ID_BATTERY_STATUS:
				{
					//printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
					mavlink_msg_battery_status_decode(&message, &(current_messages.battery_status));
					current_messages.time_stamps.battery_status = get_time_usec();
					this_timestamps.battery_status = current_messages.time_stamps.battery_status;
					break;
				}

				case MAVLINK_MSG_ID_RADIO_STATUS:
				{
					//printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
					mavlink_msg_radio_status_decode(&message, &(current_messages.radio_status));
					current_messages.time_stamps.radio_status = get_time_usec();
					this_timestamps.radio_status = current_messages.time_stamps.radio_status;
					break;
				}

				
				case MAVLINK_MSG_ID_HIGHRES_IMU:
				{
					//printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
					mavlink_msg_highres_imu_decode(&message, &(current_messages.highres_imu));
					current_messages.time_stamps.highres_imu = get_time_usec();
					this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
					break;
				}

				case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					mavlink_msg_attitude_decode(&message, &(current_messages.attitude));
					current_messages.time_stamps.attitude = get_time_usec();
					this_timestamps.attitude = current_messages.time_stamps.attitude;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}


			} // end: switch msgid

		} // end: if read message

		// Check for receipt of all items
		received_all =
				this_timestamps.heartbeat                  &&
//				this_timestamps.battery_status             &&
//				this_timestamps.radio_status               &&
//				this_timestamps.highres_imu                &&
//				this_timestamps.attitude                   &&
				this_timestamps.sys_status
				;

		// give the write thread time to use the port
		if ( writing_status > false ) {
			usleep(100); // look for components of batches at 10kHz
		}

	} // end: while not received all

	return;
}

// ------------------------------------------------------------------------------
//   Write Message
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
write_message(mavlink_message_t message)
{
	printf("Auto::write_message()");
	// do the write
	int len = serial_port->write_message(message);

	// book keep
	write_count++;

	// Done!
	return len;
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start()
{
	int result;

	// --------------------------------------------------------------------------
	//   CHECK SERIAL PORT
	// --------------------------------------------------------------------------

	if ( serial_port->status != 1 ) // SERIAL_PORT_OPEN
	{
		fprintf(stderr,"ERROR: serial port not open\n");
		throw 1;
	}


	// --------------------------------------------------------------------------
	//   READ THREAD
	// --------------------------------------------------------------------------


	result = pthread_create( &read_tid, NULL, &start_autopilot_interface_read_thread, this );
	if ( result ) throw result;

	// now we're reading messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   CHECK FOR MESSAGES
	// --------------------------------------------------------------------------

	printf("CHECK FOR MESSAGES\n");

	while ( not current_messages.sysid )
	{
		if ( time_to_exit )
			return;
		usleep(500000); // check at 2Hz
	}

	printf("Found\n");

	// now we know autopilot is sending messages
	printf("\n");


	// --------------------------------------------------------------------------
	//   GET SYSTEM and COMPONENT IDs
	// --------------------------------------------------------------------------

	// This comes from the heartbeat, which in theory should only come from
	// the autopilot we're directly connected to it.  If there is more than one
	// vehicle then we can't expect to discover id's like this.
	// In which case set the id's manually.

	// System ID
	if ( not system_id )
	{
		system_id = current_messages.sysid;
		printf("GOT VEHICLE SYSTEM ID: %i\n", system_id );
	}

	// Component ID
	if ( not autopilot_id )
	{
		autopilot_id = current_messages.compid;
		printf("GOT AUTOPILOT COMPONENT ID: %i\n", autopilot_id);
		printf("\n");
	}


	// --------------------------------------------------------------------------
	//   WRITE THREAD
	// --------------------------------------------------------------------------
	printf("START WRITE THREAD \n");

	result = pthread_create( &write_tid, NULL, &start_autopilot_interface_write_thread, this );
	if ( result ) throw result;
	

	// wait for it to be started
	while ( not writing_status )
		usleep(100000); // 10Hz

	printf("START WRITE THREAD end\n\n");
	// now we're streaming setpoint commands
	printf("\n");


	// Done!
	return;

}


// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
stop()
{
	// --------------------------------------------------------------------------
	//   CLOSE THREADS
	// --------------------------------------------------------------------------
	printf("CLOSE THREADS\n");

	// signal exit
	time_to_exit = true;

	// wait for exit
	pthread_join(read_tid ,NULL);
	pthread_join(write_tid,NULL);

	// now the read and write threads are closed
	printf("\n");

	// still need to close the serial_port separately
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_read_thread()
{

	if ( reading_status != 0 )
	{
		fprintf(stderr,"read thread already running\n");
		return;
	}
	else
	{
		read_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
start_write_thread(void)
{
	if ( not writing_status == false )
	{
		fprintf(stderr,"write thread already running\n");
		return;
	}

	else
	{
		write_thread();
		return;
	}

}


// ------------------------------------------------------------------------------
//   Quit Handler
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
handle_quit( int sig )
{
	try {
		stop();

	}
	catch (int error) {
		fprintf(stderr,"Warning, could not stop autopilot interface\n");
	}

}



// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
read_thread()
{
	reading_status = true;

	while ( ! time_to_exit )
	{
		read_messages();
		usleep(100000); // Read batches at 10Hz
	}

	reading_status = false;

	return;
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Autopilot_Interface::
write_thread(void)
{
	struct paras one_param = {0};
	char line_conent[60] = {};
        FILE *fp = NULL;
        int comment_lines = 1;//APM第一行是注释,PX4前三行是注释

        if ((NULL == (fp = fopen(APMF_PATH, "r"))))
	{
		printf("file path = %s\n", APMF_PATH);
                printf("不能打开此文件!\n");
                exit(0);
        }

        while ( !feof(fp) ){
		if (write_param_value_time_out_count != 0)
		{
			//为保证百分之百写入正确参数，在write_thread()和read_thread()处，分别对写入的参数和读出的参数计数。也就是写一个参数后必须回一个正确的参数值，否则重写.
			if (write_param_value_time_out == read_param_value_time_out)
			{
                		if(NULL == fgets(line_conent, 60, fp))
				{
					/*到了文件末尾，结束程序*/
					program_exit = true;
					printf("我已经读到了文件末尾\n");
				}
			
				//第一行注释略去	
                		comment_lines--; 
                		if (comment_lines < 0)
				{
					//从APM参数文件中一行行读取，并写入飞控.
                        		apm_get_params_from_line(line_conent, &one_param);
	
					char *end=NULL;
					float float_param_value = strtof((one_param.param_value), &end);	

					usleep(3000);
					send_param_set(one_param.param_id, float_param_value);
				
					//参数名称用来和读回的参数名称比较，以确认是否正确返回
					strcpy(previous_write_param_id, one_param.param_id); 

					//参数值用来和读回多的参数值比较，已确认数值是否正确写入.
					write_prev_value = float_param_value;

		
					write_param_value_time_out++;
        	        	}

				//第一次发送参数设置请求后，将write_param_value_time_out_count置为10开始计数.
				write_param_value_time_out_count = WRITE_PARAM_VALUE_TIME_OUT_COUNT; 
				//printf("第一次发送\n");
			}
			else
			{
				//读和写一旦异步，开始做超时判断, 等待读取到飞控返回的该参数值.
				//printf("减一计数\n");
				write_param_value_time_out_count--;
			}
		}
		else
		{
			//printf("重新发送数据\n");
			//向飞控写参数后，超时计数已经到0，需要重新发送参数，并再次开启超时计数.
			usleep(3000);
			send_param_set(previous_write_param_id, write_prev_value);
			write_param_value_time_out_count = WRITE_PARAM_VALUE_TIME_OUT_COUNT;
		}
		if(program_exit){
			write_status_to_file('2');
			exit(1);
		}
        }	
} 

// ------------------------------------------------------------------------------
//   send MAVLINK_MSG_ID_PARAM_REQUEST_LIST
// ------------------------------------------------------------------------------
int
Autopilot_Interface::
send_param_request_list()
{
	mavlink_param_request_list_t msg = {0};

	msg.target_system = system_id;
	msg.target_component = companion_id;
	
	//Encode
	mavlink_message_t message;
	mavlink_msg_param_request_list_encode(system_id, companion_id, &message, &msg);
	
	//Send the message
	int len = serial_port->write_message(message);
	
	//Done
	return len;
}

// ------------------------------------------------------------------------------
//   send MAVLINK_MSG_ID_PARAM_REQUEST_READ
// ------------------------------------------------------------------------------
int 
Autopilot_Interface::
send_param_request_read()
{
	mavlink_param_request_read_t msg = {0};

	msg.target_system = system_id;
	msg.target_component = companion_id;
	strcpy(msg.param_id ,"SYSID_SW_TYPE"); 
	//printf("msg.param_id = %s\n", msg.param_id);
	msg.param_index = 1;/*如果param_index = -1,那么直接用param_id来定位参数。 */

	//Encode
	mavlink_message_t message;
	mavlink_msg_param_request_read_encode(system_id, companion_id, &message, &msg);
	
	//Send the message
	int len = serial_port->write_message(message);
	//printf("Have write %d size message\n", len);
	
	//Done
	return len;
}

// ------------------------------------------------------------------------------
//   send MAVLINK_MSG_ID_PARAM_SET
// ------------------------------------------------------------------------------

int 
Autopilot_Interface::
send_param_set(char* _param_id, float _param_value)
{
	//printf("%s = %f\n", _param_id, _param_value);	
	mavlink_param_set_t msg = {0};

	msg.target_system = system_id;
	msg.target_component = companion_id;
	strcpy(msg.param_id, _param_id);
	msg.param_value = _param_value;
	//printf("%s = %f\n", msg.param_id, msg.param_value);	
	msg.param_type = MAV_PARAM_TYPE_REAL32;
	
	//Encode
        mavlink_message_t message;
        mavlink_msg_param_set_encode(system_id, companion_id, &message, &msg);
        
        //Send the message
        int len = serial_port->write_message(message);
        
        //Done
        return len;	
}

// End Autopilot_Interface


// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
start_autopilot_interface_read_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_read_thread();

	// done!
	return NULL;
}

void*
start_autopilot_interface_write_thread(void *args)
{
	// takes an autopilot object argument
	Autopilot_Interface *autopilot_interface = (Autopilot_Interface *)args;

	// run the object's read thread
	autopilot_interface->start_write_thread();

	// done!
	return NULL;
}



