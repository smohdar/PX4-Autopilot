/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file electron.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_log.h>
#include <nuttx/config.h>
#include <stdio.h>
#include <errno.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_outputs.h>
// #include <uORB/topics/vehicle_gps_position.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <nuttx/sched.h>
//#include <systemlib/systemlib.h>
#include <systemlib/err.h>


static bool thread_should_exit = false; /**< daemon exit flag */
static bool thread_running = false; /**< daemon status flag */
static int daemon_task; /**< Handle of daemon task / thread */


__EXPORT int electron_main(int argc, char *argv[]);

/**
* Mainloop of daemon.
*/
int electron_daemon_app_main(int argc, char *argv[]);

/**
* Print the correct usage.
*/
static void usage(const char *reason);

static void usage(const char *reason) {
	if (reason) {
	 	warnx("%s\n", reason);
	}
 	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int electron_main(int argc, char *argv[]) {

	if (argc < 2) {
 		usage("Missing command");
 		return 1;
 	}

 	else if (!strcmp(argv[1], "start")) {
 		if (thread_running) {
 			warnx("daemon already running\n");
 			return 0;
 		}

 		thread_should_exit = false;

 		daemon_task = px4_task_spawn_cmd("electron",
 			SCHED_DEFAULT,
 			SCHED_PRIORITY_DEFAULT,
 			2000,
 			electron_daemon_app_main,
 			(argv) ? (char * const *)&argv[2] : (char * const*)NULL);

		return 0;
 	}

 	else if (!strcmp(argv[1], "stop")) {
 		thread_should_exit = true;
 		return 0;
 	}

 	else if (!strcmp(argv[1], "status")) {
 		if (thread_running) {
 			warnx("\trunning\n");
 		}
 		else {
 			warnx("\tnot started\n");
 		}
 		return 0;
 	}
 	else {
 		usage("unrecognized command");
 		return 1;
 	}

 	return 1;
}

int electron_daemon_app_main(int argc, char *argv[]) {

	warnx("[Daemon] starting\n");
	thread_running = true;

	printf("Started Thread! Subscribing to battery UOrb...\n");

	//Structs to contain the system state
	struct battery_status_s battStatus;
	memset(&battStatus, 0, sizeof(battStatus));
	// v1
	struct actuator_outputs_s outputsStatus;
	memset(&outputsStatus, 1, sizeof(outputsStatus));

	//Subscribe to topics
	int batt_sub = orb_subscribe(ORB_ID(battery_status));
	// v1
	int outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));

	while(!thread_should_exit) {

		//check for any new update
		bool battStatusUpdated;
		orb_check(batt_sub, &battStatusUpdated);
		// v1
		bool actuatorStatusUpdated;
		orb_check(outputs_sub, &actuatorStatusUpdated);

		//copy a local copy, can also check for any change with above boolean
		orb_copy(ORB_ID(battery_status), batt_sub, &battStatus);
		// v1
		orb_copy(ORB_ID(actuator_outputs), outputs_sub, &outputsStatus);

 		// printf("BattVoltage=%.2f | ", (double)battStatus.voltage_v);
 		// printf("BattCurrent=%.2f\n", (double)battStatus.current_a);
		// v1
		printf("Left Motor = %f | ", (double)outputsStatus.output[0]);
		printf("Right Motor = %f\n", (double)outputsStatus.output[1]);
		printf("Front Left Servo = %f | ", (double)outputsStatus.output[4]);
		printf("Front Right Servo = %f\n", (double)outputsStatus.output[5]);
		printf("Rear Left Servo = %f | ", (double)outputsStatus.output[6]);
		printf("Rear Right Servo = %f\n", (double)outputsStatus.output[7]);
		printf("\n");
 		sleep(1);
	}

	warnx("[Daemon] exiting.\n");
 	thread_running = false;
 	return OK;
 }
