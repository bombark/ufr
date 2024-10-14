#include <ufr.h>
#include "ufr_gtw_ros_noetic.hpp"

void test_twist() {
    link_t topic;
    ufr_args_t args = {.text="@topic scan @msg twist @debug 4"};
    ufr_gtw_ros_noetic_new_topic(&topic, UFR_START_PUBLISHER);
    topic.log_level = 4;
    ufr_boot_gtw(&topic, &args);
    ufr_start_publisher(&topic, &args);

    for (int i=0; i<5; i++) {
        ufr_put(&topic, "iii\n", 10,11,12);
        sleep(2);
    }

    ufr_close(&topic);
}

int main() {
    test_twist();
    return 0;
}