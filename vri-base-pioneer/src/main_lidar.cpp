/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  Example of Sick LMS 291.
 *               The velocity of each wheel is set
 *               according to a Braitenberg-like algorithm which takes the values returned by the Sick as input.
 */

// ============================================================================
//  Header
// ============================================================================

#include <math.h>
#include <iostream>
#include <ufr.h>
#include <unistd.h>

#include "sl_lidar.h" 
#include "sl_lidar_driver.h"

using namespace std;
using namespace sl;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define LIDAR_SERIAL   "/dev/ttyUSB1"

ILidarDriver* g_lidar_drv;

// ============================================================================
//  Main
// ============================================================================

bool checkSLAMTECLIDARHealth(ILidarDriver * drv)
{
    sl_result     op_result;
    sl_lidar_response_device_health_t healthinfo;

    op_result = drv->getHealth(healthinfo);
    if (SL_IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
        printf("SLAMTEC Lidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == SL_LIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, slamtec lidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want slamtec lidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }

    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}

int start_lidar() {
	///  Create a communication channel instance
	sl_lidar_response_device_info_t devinfo;
	g_lidar_drv = *createLidarDriver();
	printf("%p\n", g_lidar_drv);
	IChannel* channel = *createSerialPortChannel(LIDAR_SERIAL, 115200);
	if (SL_IS_OK((g_lidar_drv)->connect(channel))) {
		sl_result op_result = g_lidar_drv->getDeviceInfo(devinfo);
		if ( SL_IS_OK(op_result) == 0 ) {
			printf("ERROR\n");
		}
	}

	printf("SLAMTEC LIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }
	printf("\n");

	// check health...
    if (!checkSLAMTECLIDARHealth(g_lidar_drv)) {
        return 1;
    }

	g_lidar_drv->setMotorSpeed();
	g_lidar_drv->startScan(0,1);
	printf("%p\n", g_lidar_drv);

	return 0;
}


int main(int argc, char **argv) {

	link_t pub_lidar = ufr_sys_open("lidar", "@new zmq:topic @host 192.168.43.141 @port 5002 @coder msgpack:obj");
    lt_start_publisher(&pub_lidar, NULL);

	start_lidar();

	while(1) {

		int count[360];
		float values[360];
		for (int i=0; i<360; i++) {
			values[i] = 0;
			count[i] = 0;
		}

		// send lidar data
		static sl_lidar_response_measurement_node_hq_t nodes[8192];
		size_t   nodes_size = _countof(nodes);
		sl_result op_result = g_lidar_drv->grabScanDataHq(nodes, nodes_size);

		if (SL_IS_OK(op_result)) {
			g_lidar_drv->ascendScanData(nodes, nodes_size);
			for (int pos = 0; pos < (int)nodes_size ; ++pos) {
				// char* flag = (nodes[pos].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) ? "S ":"  ", 
				double angle = (nodes[pos].angle_z_q14 * 90.f) / 16384.f;
				double dist = nodes[pos].dist_mm_q2 / 4.0f;

				if ( angle >= 360 ) {
					break;
				}

				const int i32_angle = (int) angle;
				if ( dist != INFINITY ) {
					values[i32_angle] += dist;
					count[i32_angle] += 1;
				}
				// uint8_t quality = nodes[pos].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
			}

			for (int i=0; i<360; i++) {
				if ( count[i] > 0 ) {
					values[i] /= count[i];
				}
			}

			lt_put(&pub_lidar, "af\n", 360, values);
		}


		// wait 50ms
		usleep(50000);
	}

    return 0;
}