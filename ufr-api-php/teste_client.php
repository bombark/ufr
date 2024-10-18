<?php

$link = ufr_client("@new zmq:socket @coder msgpack");
ufr_put($link, "s\n\n", "scan");
$val = ufr_get($link, "^iii\n");
ufr_close($link);

print_r($val);

?>