#include <ufr.h>
#include "ufr_gtw_ros_humble.hpp"

void test1() {
    link_t topic;
    lt_args_t args = {.text="@topic teste @msg twist"};
    ufr_new_gtw_ros_humble_topic(&topic, &args);
    lt_start_publisher(&topic, &args);

    for (int i=0; i<10; i++) {
        lt_put(&topic, "iii\n",i+1,i+2,i+3);
        sleep(1);
    }

    lt_close(&topic);
}

void test2() {
    link_t topic = ufr_publisher("@new ros_humble:topic @topic teste @msg twist");

    for (int i=0; i<10; i++) {
        lt_put(&topic, "iii\n",i+1,i+2,i+3);
        sleep(1);
    }

    lt_close(&topic);
}

int main() {
    test2();
    return 0;
}