<?php

// putenv("LD_LIBRARY_PATH=/opt/ros/humble/lib/");

$link = ufr_publisher("@new ros_humble:topic @msg twist @topic odom");
for ($i=0; $i<10; $i++) {
    ufr_put($link, "iii\n", 10+$i, 11, 12);
    sleep(1);
}
ufr_close($link);

?>