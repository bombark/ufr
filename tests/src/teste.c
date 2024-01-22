#include <stdio.h>
#include <string.h>
#include <lt_api.h>
#include <unistd.h>

int main_publisher() {
    link_t file = lt_new("@new zmq:topic @path output.txt @mode w @host 127.0.0.1 @port 5000");
    lt_load_encoder(&file, "msgpack:obj", NULL);
    lt_start_publisher(&file);

    for (int i=0; i<1; i++) {
        lt_put(&file, "iii\n", 0, 5, 10);
    }
    
    lt_close(&file);
    return 0;
}

int main_subscriber() {
    link_t file = lt_new("@new zmq:topic @host  185.209.160.8  @port 5001");
    lt_load_decoder(&file, "msgpack:obj", NULL);
    lt_start_subscriber(&file);

    for (int i=0; i<5; i++) {
        int a,b,c;
        float lidar[32];
        lt_get(&file, "^iii", &a, &b, &c);
        printf("%d %d %d\n", a,b,c);

/*
        lt_recv(&file);
        lt_copy_af32(&file, 32, lidar);
        for (int i=0; i<32; i++) {
            printf("%f ", lidar[i]);
        }
        printf("\n");
*/
    }
 
    lt_close(&file);
    return 0;
}


int main_subscriber_async() {
    lt_args_t args = {.text="@host 127.0.0.1 @port 5000"};
    link_t file;// = lt_load("zmq:topic", &args);
    lt_load_decoder(&file, "msgpack:obj", NULL);
    lt_start_subscriber(&file);

    while(1) {
        int a,b;

        if ( lt_recv_async(&file) ) {
            lt_get(&file, "ii", &a, &b);
            printf("%d %d\n", a, b);
        } else {
            // printf("nada\n");
            // sleep(1);
        }
    }
 
    lt_close(&file);
    return 0;
}

// copiar os dados, retornar um ponteiro dos dados e seu tamanho
int main_server() {
    lt_args_t args = {.text="@host 127.0.0.1 @port 5001"};
    link_t file;// = lt_load("zmq:socket", &args);
    lt_load_encoder(&file, "msgpack:obj", NULL);
    lt_load_decoder(&file, "msgpack:obj", NULL);
    lt_start_bind(&file);

    for (int i=0; i<5; i++) {
        float arr[8];
        char* command = NULL;

        lt_recv(&file);
        lt_copy_af32(&file, 8, arr);

        for (int i=0; i<8; i++) {
            printf("%f ", arr[i]);
        }
        printf("\n");

        // lt_get(&file, "^ai", 8, &arr_size, arr_data);
        // printf("%s %d ddd\n", command, a);
        lt_put(&file, "i\n", 32);
    }
    
    lt_close(&file);
    return 0;
}

const char* file = "@new zmq:socket @host 127.0.0.1 @port 5001";

int main_client() {
    lt_args_t args = {.text="@host 127.0.0.1 @port 5001"};
    link_t file;// = lt_load("zmq:socket", &args);
    lt_load_encoder(&file, "msgpack:obj", NULL);
    lt_load_decoder(&file, "msgpack:obj", NULL);
    lt_start_connect(&file);

    float buffer[8] = {10.0,20.0,30.0,40.0,50.0,60.0,70.0,80.0};
    int resp;
    lt_put(&file, "af\n", 8, buffer);
    lt_get(&file, "^i", &resp);
    printf("resp %d\n", resp);

    lt_close(&file);
    return 0;
}


int main_ros_pub() {
    printf("Ros\n");
    // link_t file = lt_new("@new ros2:topic @encoder ros2:twist");
    link_t file = lt_new("@new posix:file @path saida.txt @encoder ros2:twist");
    lt_start_publisher(&file);
    // lt_encoder_order("@linear.x 0 @linear.y 1");
    
    for (int i=0; i<5; i++) {
        printf("enviando\n");
        lt_put(&file, "iiiiii\n", 1,2,3,4,5,6);
        sleep(1);
    }

    // lt_close(&file);
    return 0;
}

int main_ros_sub() {
    printf("Ros\n");
    link_t file = lt_new("@new ros2:topic @decoder ros2:twist");
    lt_start_subscriber(&file);
    // lt_decoder_format(&file, "linear.y linear.x angular.x");
    
    int a,b,c,d,e,f;
    for(int i=0; i<3; i++) {
        lt_get(&file, "^iiiiii", &a, &b, &c, &d, &e, &f);
        printf("%d %d %d\n", a,b,c);
    }

    lt_close(&file);
    return 0;
}


/*
3
 - 0 - 0
1 - 1 - 8
2 - 2 - 16
3 - 3 - 24
4 - 4 - 32

*/


int main(int argc, char** argv){
    if ( argc == 1 ) {
        return main_publisher();
    }

    const char* cmd = argv[1];
    if ( strcmp(cmd, "sub") == 0 ) {
        return main_subscriber();
    } else if ( strcmp(cmd, "asub") == 0 ) {
        return main_subscriber_async();
    } else if ( strcmp(cmd, "pub") == 0 ) {
        return main_publisher();
    } else if ( strcmp(cmd, "srv") == 0 ) {
        return main_server();
    } else if ( strcmp(cmd, "cli") == 0 ) {
        return main_client();
    } else if ( strcmp(cmd, "rosp") == 0 ) {
        return main_ros_pub();
    } else if ( strcmp(cmd, "ross") == 0 ) {
        return main_ros_sub();
    }
}




