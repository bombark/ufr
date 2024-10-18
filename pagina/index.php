<!DOCTYPE html>
<html><head>
    <title> VRI-Pioneer </title>
    <!-- <link href="/bootstrap.min.css" rel="stylesheet"> -->

</head><body>

<div class="container">

    <h1> VRI-Pioneer </h1><hr>

    <h2> Tópicos </h2>
    <?php
        $link = ufr_client("@new zmq:socket @coder msgpack @debug 4");

        ufr_put($link, "s\n\n", "scan");
        while (1) {
            $res = ufr_get($link, "^a");
            if ( $res == false ) {
                break;
            }
            print_r($res[0]);
        }
        ufr_close($link);

        // $link = ufr_subscriber("@new zmq:topic");
        // ufr_close($link);
        // echo "opa";
    ?>

</div>
<!-- <script src="bootstrap.bundle.min.js"></script> -->
</body></html>
