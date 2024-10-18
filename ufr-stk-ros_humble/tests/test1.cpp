#include <ufr.h>
#include "ufr_gtw_ros_humble.hpp"

void test_twist() {
    link_t topic;
    ufr_args_t args = {.text="@topic scan @msg twist @debug 4"};
    ufr_gtw_ros_humble_new_topic(&topic, UFR_START_PUBLISHER);
    topic.log_level = 4;
    ufr_boot_gtw(&topic, &args);
    ufr_start_publisher(&topic, &args);

    for (int i=0; i<10; i++) {
        ufr_put(&topic, "iii\n",i+1,i+2,i+3);
        sleep(1);
    }

    ufr_close(&topic);
}

void test_pose() {
    link_t topic;
    ufr_args_t args = {.text="@topic teste @msg pose @debug 4"};
    ufr_gtw_ros_humble_new_topic(&topic, UFR_START_PUBLISHER);
    topic.log_level = 4;
    ufr_boot_gtw(&topic, &args);
    ufr_start_publisher(&topic, &args);

    for (int i=0; i<10; i++) {
        ufr_put(&topic, "iii\n",i+1,i+2,i+3);
        sleep(1);
    }

    ufr_close(&topic);
}

void test_laserscan_publisher() {
    link_t topic;
    ufr_args_t args = {.text="@topic teste @msg laser_scan @debug 4"};
    ufr_gtw_ros_humble_new_topic(&topic, UFR_START_PUBLISHER);
    topic.log_level = 4;
    ufr_start_publisher(&topic, &args);

    float laser[10] = {9.0,8.0,7.0,6.0,5.0,4.0,3.0,2.0,1.0,0.0};
    for (int i=0; i<5; i++) {
        ufr_put(&topic, "iiiiiii",i+1,i+2,i+3,1,1,1,1);
        ufr_put_af32(&topic, laser, 10);
        ufr_put_af32(&topic, laser, 10);
        ufr_put(&topic, "\n");
        sleep(1);
    }

    ufr_close(&topic);
}


void test_laserscan() {
    link_t link = ufr_subscriber("@new ros_humble:topic @msg laserscan @topic scan");

    float angle_min, angle_max, angle_increment;
    float time_increment, scan_time;
    float range_min, range_max;
    float ranges[1200];
    float intensities[1200];
    // ufr_recv(&link);
    for (int i=0; i<5000; i++) {
        if ( ufr_recv_async(&link) == UFR_OK ) {
            ufr_get(&link, "fff", &angle_min, &angle_max, &angle_increment);
            ufr_get(&link, "ff", &time_increment, &scan_time);
            ufr_get(&link, "ff", &range_min, &range_max);
            ufr_get(&link, "af", ranges, 1200);
            ufr_get(&link, "af", intensities, 1200);
            printf("%f %f %f : %f\n", angle_min, angle_max, angle_increment, ranges[100]);
        }
    }

    ufr_close(&link);
}


/*
void test2() {
    link_t topic = ufr_publisher("@new ros_humble:topic @topic /cmd_vel @msg twist");

    for (int i=0; i<10; i++) {
        float vel=1, rotvel=0;
        scanf("%f %f", &vel, &rotvel);
        ufr_put(&topic, "fiiiif\n",vel,0,0,0,0,rotvel);
        // sleep(1);
    }

    ufr_close(&topic);
}
*/

int main() {
    test_laserscan();
    return 0;
}