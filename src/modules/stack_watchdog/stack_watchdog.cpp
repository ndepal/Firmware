/****************************************************************************
*
*   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
* @file stack_watchdog.c
* Watchdog for high stack usage
*
* @author Nicolas de Palezieux <ndepal@gmail.com>
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <drivers/drv_hrt.h>

#include <px4_config.h>
#include <nuttx/sched.h>
#include <px4_getopt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <systemlib/mavlink_log.h>

#include <mavlink/mavlink_shell.h>

#include <regex>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
MavlinkShell* mavlink_shell;

struct stack_warning {
	int pid;
	int last_warn_time;
};

/**
* daemon management function.
*/
extern "C" __EXPORT int stack_watchdog_main(int argc, char *argv[]);

/**
* Mainloop of daemon.
*/
int stack_watchdog_thread_main(int argc, char *argv[]);

/**
* Print the correct usage.
*/
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: stack_watchdog {start|stop|status} [-r <loop_rate>] [-t <warn_thresh>]\nloop_rate: Rate at which stack usage is checked, in ms\nwarn_thresh: Percentage threshold above which a warning is published, 0..100\n");
}

/**
* The daemon app only briefly exists to start
* the background job. The stack size assigned in the
* Makefile does only apply to this management task.
*
* The actual stack size should be set in the call
* to task_create().
*/
int stack_watchdog_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("already running\n");
			/* this is not an error */
			return 0;
		}

		daemon_task = px4_task_spawn_cmd("stack_watchdog",
		SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT,
		12000,
		stack_watchdog_thread_main,
		(argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int stack_watchdog_thread_main(int argc, char *argv[])
{
	warnx("starting main thread\n");

	int loop_rate = 1000000;
	float warn_thresh = 0.9;

	int ch;

	int myoptind = 1;
	const char *myoptarg = NULL;
	while ((ch = px4_getopt(argc, argv, "r:t:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'r': {
				unsigned long r = strtoul(myoptarg, NULL, 10);

				if (r <= 0) {
					r = 1000;
				}

				loop_rate = r * 1000;
			}
			break;

		case 't': {
				unsigned long t = strtoul(myoptarg, NULL, 10);

				if (t <= 0) {
					t = 1;
				}

				warn_thresh = 0.01 * t;
			}
			break;

		default:
			usage("unrecognized flag");
			return -1;
		}
	}

	// MavlinkShell doesn't really have anything to do with mavlink,
	// it just provides an interface to the nuttx shell
	mavlink_shell = new MavlinkShell();

	if (!mavlink_shell) {
		PX4_ERR("Failed to allocate a shell");

	} else {
		int ret = mavlink_shell->start();

		if (ret != 0) {
			PX4_ERR("Failed to start shell (%i)", ret);
			delete mavlink_shell;
			mavlink_shell = nullptr;
		}
	}

	int max_warnings = 500;
	stack_warning warnings[max_warnings];
	for (int i = 0; i < max_warnings; i++) {
		stack_warning w;
		w.pid = -1;
		warnings[i] = w;
	}

	thread_running = true;

	size_t command_len = 4;
	uint8_t command[] = "top\n";

	mavlink_shell->write(command, command_len);

	size_t buffer_len = 2048;
	uint8_t buffer[buffer_len];
	memset(buffer, 0, buffer_len);

	orb_advert_t mavlink_log_pub = nullptr;

	int cntr = 0;

	while (!thread_should_exit) {
		/* check for shell output */
		if (mavlink_shell && mavlink_shell->available() > 0) {
			int cnt = mavlink_shell->read(buffer, buffer_len);
			if (cnt) {
				size_t start = 0;
				size_t end = 0;
				while (end < buffer_len) {
					// find one line in the read data
					if (buffer[end] == '\n') {

						// the PID is at the beginning
						int pid_start = 2;
						int pid_len = 0;
						// skip the first non-number characters
						while ((buffer[start + pid_start] < '0' || buffer[start + pid_start] > '9') && start + pid_start + pid_len < end) {
							pid_start++;
						}
						// find the length of the pid
						while ((buffer[start + pid_start + pid_len] >= '0' && buffer[start + pid_start + pid_len] <= '9') && start + pid_start + pid_len < end) {
							pid_len++;
						}

						char pid_chars[16];
						memset(pid_chars, 0, 16);
						memcpy(&pid_chars[0], &buffer[start+pid_start], (pid_len) * sizeof(uint8_t));
						pid_chars[pid_len] = '\0';
						int process_id = atoi(pid_chars);
						// not all lines contain process info, so we might not find a PID
						if (process_id) {

							int process_name_start = pid_start + pid_len + 1;
							int process_name_len = 0;
							while (buffer[start + process_name_start + process_name_len] != ' ' && start + process_name_start + process_name_len< end) {
								process_name_len++;
							}

							// not all lines contain process info, so we might not find a process name
							if (process_name_len) {

								char process_name_chars[32];
								memset(process_name_chars, 0, 32);
								memcpy(&process_name_chars[0], &buffer[start + process_name_start], (process_name_len) * sizeof(uint8_t));
								process_name_chars[process_name_len] = '\0';

								// go through the current line and search for the stack statement, which contains a '/'
								int stack_slash = 0;
								while (buffer[start + stack_slash] != '/' && start + stack_slash < end) {
									stack_slash++;
								}

								if (start + stack_slash < end-1) {
									// found a '/'
									// go backwards from there to get the current stack usage
									int current_stack_len = 0;
									while ((buffer[start + stack_slash - current_stack_len -1 ] >= '0' && buffer[start + stack_slash - current_stack_len -1] <= '9') && -current_stack_len < stack_slash) {
										current_stack_len++;
									}
									char current_stack_chars[16];
									memset(current_stack_chars, 0, 16);
									memcpy(&current_stack_chars[0], &buffer[start + stack_slash - current_stack_len -1], (current_stack_len + 1) * sizeof(uint8_t));
									int current_stack_size = atoi(current_stack_chars);

									// go forward from the slash char to get the max stack usage
									int max_stack_len = 0;
									while ((buffer[start + stack_slash + max_stack_len + 2] >= '0' && buffer[start + stack_slash + max_stack_len + 2] <= '9') && start + max_stack_len < end) {
										max_stack_len++;
									}
									char max_stack_chars[16];
									memset(max_stack_chars, 0, 16);
									memcpy(&max_stack_chars[0], &buffer[start + stack_slash + 1], (max_stack_len+1) * sizeof(uint8_t));
									int max_stack_size = atoi(max_stack_chars);

									// skip if parsing failed
									if (max_stack_size > 0) {
										float stack_ratio = (float) current_stack_size / (float) max_stack_size;

										// try to find this PID in the list of PIDs we've already warned about
										if (stack_ratio > warn_thresh) {
											int found = -1;
											for (int i = 0; i < max_warnings; i ++) {
												if (warnings[i].pid == process_id) {
													found = i;
													break;
												}
											}
											bool do_warn = true;
											if (found >= 0) {
												// don't warn about the same PID's every loop
												if (warnings[found].last_warn_time + 10 > cntr) {
													do_warn = false;
												}
											}
											if (do_warn) {
												mavlink_log_critical(&mavlink_log_pub, "Process %s (PID %d) is using %.3f of it's stack", process_name_chars, process_id, (double)stack_ratio);
											}
											if (found >= 0) {
												if (do_warn) {
													// if we warned about a PID that was already in the list, update it's warn time
													warnings[found].last_warn_time = cntr;
												}
											} else {
												// This was a new PID, insert it into the list of PIDs
												for (int i = 0; i < max_warnings; i ++) {
													if (warnings[i].pid < 0) {
														stack_warning w;
														w.pid = process_id;
														w.last_warn_time = cntr;
														warnings[i] = w;
														break;
													}
												}
											}
										}
									}
								}
							}
						}

						start = end+1;
					}
					end++;
				}
				memset(buffer, 0, buffer_len);
			}
		}

		memset(buffer, 0, buffer_len);

		cntr++;

		usleep(loop_rate);
	}

	if (mavlink_shell) {
		delete mavlink_shell;
		mavlink_shell = nullptr;
	}

	warnx("exiting.\n");

	thread_running = false;

	return 0;
}
