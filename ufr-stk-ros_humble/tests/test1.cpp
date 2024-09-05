#include <ufr.h>
#include "ufr_gtw_ros_humble.hpp"

void test_twist() {
    link_t topic;
    ufr_args_t args = {.text="@topic teste @msg twist @debug 4"};
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
    test_pose();
    return 0;
}