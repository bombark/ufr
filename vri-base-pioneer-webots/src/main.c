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
#include <stdio.h>
#include <stdlib.h>
#include <webots/lidar.h>
#include <webots/motor.h>
#include <webots/compass.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <ufr.h>

#define TIME_STEP 32
#define MAX_SPEED 6.4
#define CRUISING_SPEED 5.0
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char **argv) {
    link_t pub_lidar = ufr_sys_publisher("lidar", "@new zmq:topic @host 127.0.0.1 @port 5001 @coder msgpack:obj");

    // 
    link_t sub_motors = ufr_sys_open("motor", "@new zmq:topic @host 127.0.0.1 @port 5002 @coder msgpack:obj");
    ufr_start_subscriber(&sub_motors, NULL);
    
    //
    link_t pub_compass = ufr_sys_publisher("compass", "@new zmq:topic @host 127.0.0.1 @port 5003 @coder msgpack:obj @message ff");
    
    //
    link_t pub_encoder = ufr_sys_publisher("encoder", "@new zmq:topic @host 127.0.0.1 @port 5004 @coder msgpack:obj");


/*
    link_t pub_lidar = ufr_sys_open("lidar", "@new mqtt:topic @host 185.209.160.8 @topic robo/lidar @coder msgpack:obj");
    lt_start_publisher(&pub_lidar, NULL);

    // 
    link_t sub_motors = ufr_sys_open("motor", "@new mqtt:topic @host 185.209.160.8 @topic robo/motor @coder msgpack:obj");
    lt_start_subscriber(&sub_motors, NULL);
    
    //
    link_t pub_compass = ufr_sys_open("compass", "@new mqtt:topic @host 185.209.160.8 @topic robo/compass @coder msgpack:obj");
    lt_start_publisher(&pub_compass, NULL);
    
    //
    link_t pub_encoder = ufr_sys_open("encoder", "@new mqtt:topic @host 185.209.160.8 @topic robo/pose @coder msgpack:obj");
    lt_start_publisher(&pub_encoder, NULL);
*/


    // init webots stuff
    wb_robot_init();

    // get devices
    WbDeviceTag compass = wb_robot_get_device("compass");
    WbDeviceTag lms291 = wb_robot_get_device("Sick LMS 291");
    WbDeviceTag front_left_wheel = wb_robot_get_device("front left wheel");
    WbDeviceTag front_right_wheel = wb_robot_get_device("front right wheel");
    WbDeviceTag back_left_wheel = wb_robot_get_device("back left wheel");
    WbDeviceTag back_right_wheel = wb_robot_get_device("back right wheel");

    WbDeviceTag left_position_sensor = wb_robot_get_device("front left wheel sensor");
    WbDeviceTag right_position_sensor = wb_robot_get_device("front right wheel sensor");

    // init compass
    wb_compass_enable(compass, TIME_STEP);

    // init lms291
    wb_lidar_enable(lms291, TIME_STEP);
    const int lms291_width = wb_lidar_get_horizontal_resolution(lms291);
    const int half_width = lms291_width / 2;
    const int max_range = wb_lidar_get_max_range(lms291);
    const double range_threshold = max_range / 20.0;
    const float *lms291_values = NULL;

    // init motors
    wb_motor_set_position(front_left_wheel, INFINITY);
    wb_motor_set_position(front_right_wheel, INFINITY);
    wb_motor_set_position(back_left_wheel, INFINITY);
    wb_motor_set_position(back_right_wheel, INFINITY);
    wb_position_sensor_enable(left_position_sensor, TIME_STEP);
    wb_position_sensor_enable(right_position_sensor, TIME_STEP);

    // init speed for each wheel
    double back_left_speed = 0.0, back_right_speed = 0.0;
    double front_left_speed = 0.0, front_right_speed = 0.0;
    wb_motor_set_velocity(front_left_wheel, front_left_speed);
    wb_motor_set_velocity(front_right_wheel, front_right_speed);
    wb_motor_set_velocity(back_left_wheel, back_left_speed);
    wb_motor_set_velocity(back_right_wheel, back_right_speed);

    // control loop
    while (wb_robot_step(TIME_STEP) != -1) {
        // publisher the lidar values
        lms291_values = wb_lidar_get_range_image(lms291);
        // lt_put(&pub_lidar, "af\n", lms291_width, lms291_values);

        // compass
        const double* compass_values = wb_compass_get_values(compass);
        float rad = atan2(compass_values[1], compass_values[0]);
        rad -= 1.5708;
        if ( rad < 0 ) {
            rad += 2.0*M_PI;
        }
	
        /* nao lembro
        float bearing = (rad - 1.5708) / M_PI * 180.0;
        if (bearing < 0.0)
            bearing = bearing + 360.0;
        */
        lt_put(&pub_compass, "f\n", rad);

        // wheel encoders
        const int left = wb_position_sensor_get_value(left_position_sensor);
        const int right = wb_position_sensor_get_value(right_position_sensor);
        lt_put(&pub_encoder, "ii\n", left, right);
        // printf("%f %f\n", left, right);

        // set actuators when receive new data
        if ( lt_recv_async(&sub_motors) ) {           
            int vel, rotvel;
            lt_get(&sub_motors, "ii", &vel, &rotvel);

            const int left = vel/10 + rotvel/10;
            const int right = vel/10 - rotvel/10;
            printf("Set Motors %d %d -> %d %d\n", vel, rotvel, left, right);

            wb_motor_set_velocity(front_left_wheel, left);
            wb_motor_set_velocity(front_right_wheel, right);
            wb_motor_set_velocity(back_left_wheel, left);
            wb_motor_set_velocity(back_right_wheel, right);
        }

    }

    wb_robot_cleanup();
    return 0;
}
