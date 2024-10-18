<?php

$link = ufr_subscriber("@new zmq:topic @coder msgpack @port 3002");
while(1) {
    $val = ufr_get($link, "^a");
    print_r($val);
}
ufr_close($link);

?>