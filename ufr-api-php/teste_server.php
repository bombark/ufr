<?php

$link = ufr_server("@new zmq:socket @coder msgpack");
$resp = ufr_get($link, "^s");
// ufr_put($link, "s\n", "OK");
ufr_close($link);

echo "$resp\n";

?>