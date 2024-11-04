<!DOCTYPE html>
<html><head>
    <title> VRI-Pioneer </title>
    <!-- <link href="/bootstrap.min.css" rel="stylesheet"> -->

</head><body>

<div class="container">

    <h1> VRI-Pioneer </h1><hr>
    <?php
        $link = ufr_client("@new zmq:socket @coder msgpack");
        ufr_put($link, "s\n\n", "odom");
        $pose = ufr_get($link, "^fff\n");

        /*ufr_put($link, "s\n\n", "scan");
        $scan = ufr_get($link, "^af\n");*/
    ?>

    Controle: 
        <button onclick="fetch('http://localhost:2000/cmd_vel.php?vel=1&rot=0')"> Frente </button>
        <button onclick="fetch('http://localhost:2000/cmd_vel.php?vel=0&rot=0')"> Parar </button>
        <button onclick="fetch('http://localhost:2000/cmd_vel.php?vel=0&rot=-1')"> Esquerda </button>
        <button onclick="fetch('http://localhost:2000/cmd_vel.php?vel=0&rot=1')"> Direita </button>
     <br>

    Velocidade: <br>

    Posição: <div id="odom"></div> <br>

    Lidar: <img id="lidar" src="/image.php" width=320>
    

    <h2> Tópicos </h2>
    <?php
        /*$link = ufr_client("@new zmq:socket @coder msgpack @debug 4");
        ufr_put($link, "s\n\n", "scan");
        while (1) {
            $res = ufr_get($link, "^af");
            if ( $res == false ) {
                echo "opa";
                break;
            }
            print_r($res);
            echo "<br><hr>";
        }
        ufr_close($link);*/
    ?>

</div>

<script>

function update_image() {
    const imagem = document.getElementById('lidar');
    const timestamp = new Date().getTime(); // Adiciona um timestamp para evitar cache
    imagem.src = `image.php?t=${timestamp}`; // Atualiza a imagem
}

setInterval(update_image, 500);



function update_odom() {
    fetch('http://localhost:2000/odom.php').then(response => {
        if (!response.ok) {
            throw new Error('Erro na rede: ' + response.statusText);
        }
        return response.json();
    })
    .then(data => {
        const div = document.getElementById('odom');
        div.innerHTML = `${data}`; // Atualiza o conteúdo da DIV
    })
    .catch(error => {
        console.error('Houve um problema com a requisição:', error);
    });
}
setInterval(update_odom, 500);

</script>

<!-- <script src="bootstrap.bundle.min.js"></script> -->
</body></html>
