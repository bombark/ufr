/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica Imagem (VRI)
 *   Felipe Bombardelli
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */

// ============================================================================
//  Documentation
// ============================================================================

/*
fazer:
 - realocar o buffer da mensage quando receber mensagem maior que o buffer alocado

erros:
 - iniciar com publisher mas tentar receber dados
 - ufr_new cria um link temporario e nao pode ser guardado na estrutura
  - solucao foi colocar a inicializacao do cliente na funcao start
*/

// ============================================================================
//  Header
// ============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <mosquitto.h>
#include <ufr.h>

#define MQTT_QOS_0 0

typedef struct {
    char broker_hostname[128];
    char topic_name[128];
    uint16_t broker_port;
} ll_shr_t;

typedef struct {
    uint8_t start_type;
    bool is_received;
    struct mosquitto* mosq;
    size_t msg_size;
    size_t msg_size_max;
    size_t msg_read_idx;
    char* msg_data;
} ll_obj_t;

size_t g_mosq_count = 0;

// ============================================================================
//  Subscriber Receive Callback
// ============================================================================

static
void urf_gtw_mqtt_recv_cb(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    ll_obj_t* obj = userdata;
    // ufr_info(link, "received %ld bytes %p", message->payloadlen, obj);
    // printf("received %ld bytes %p\n", message->payloadlen, obj);

    // expand the buffer case payload is bigger
    if ( message->payloadlen > obj->msg_size_max ) {
        free(obj->msg_data);
        obj->msg_data = malloc(message->payloadlen);
        obj->msg_size_max = message->payloadlen;
    }

    // copy the message to the buffer
    obj->msg_size = message->payloadlen;
    obj->msg_read_idx = 0;
    memcpy(obj->msg_data, message->payload, obj->msg_size);
    obj->is_received = true;
} 

// ============================================================================
//  Topic
// ============================================================================

static
int urf_gtw_mqtt_type(const link_t* link) {
    return 0;
}

static
int urf_gtw_mqtt_state(const link_t* link) {
    return 0;
}

static
size_t urf_gtw_mqtt_size(const link_t* link, int type) {
    return 0;
}

static
int urf_gtw_mqtt_boot (link_t* link, const ufr_args_t* args) {
    // initialize the mosquitto library on first time
    if ( g_mosq_count == 0 ) {
        mosquitto_lib_init();
    
        //Get libmosquitto version info
        int major, minor, revision;
        mosquitto_lib_version(&major, &minor, &revision);
        ufr_info(link, "Libmosquitto version: %d.%d.%d", major, minor, revision);
    }
    
    // get the arguments
    const char* host = ufr_args_gets(args, "@host", "127.0.0.1");
    const uint16_t port = ufr_args_geti(args, "@port", 1883);
    const char* topic = ufr_args_gets(args, "@topic", "");

    // prepare the shared object
    ll_shr_t* shr = malloc(sizeof(ll_shr_t));
    if ( shr == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }
    strcpy(shr->broker_hostname, host);
    strcpy(shr->topic_name, topic);
    shr->broker_port = port;

    // prepare the private object
    ll_obj_t* obj = malloc(sizeof(ll_obj_t));
    if ( obj == NULL ) {
        free(shr);
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    obj->start_type = 0;
    obj->msg_size_max = 4096;
    obj->msg_data = malloc(obj->msg_size_max);
    if ( obj->msg_data == NULL ) {
        return ufr_error(link, ENOMEM, strerror(ENOMEM));
    }

    obj->is_received = false;
    obj->mosq = NULL;

    // success
    link->gtw_shr = shr;
    link->gtw_obj = obj;
    g_mosq_count += 1;
    return UFR_OK;
}

static
int urf_gtw_mqtt_start (link_t* link, int type, const ufr_args_t* args) {
    ll_shr_t* shr = link->gtw_shr;
    ll_obj_t* obj = link->gtw_obj;

    if ( type == UFR_START_PUBLISHER ) {
        ufr_info(link, "starting publisher");

        // initialize the mosquitto client
        // *** iniatilize the client in the start function instead of boot
        obj->mosq = mosquitto_new(NULL, true, obj);
        if ( obj->mosq == NULL ) {
            return ufr_error(link, 1, "failed to create mosquitto client");
        }

        // connect the mosquitto client
        if ( mosquitto_connect(obj->mosq, shr->broker_hostname, shr->broker_port, 60) != MOSQ_ERR_SUCCESS) {
            return ufr_error(link, 1, "connecting to MQTT broker failed");
        }
        ufr_info(link, "connected");
        obj->start_type = UFR_START_PUBLISHER;

    } else if ( type == UFR_START_SUBSCRIBER ) {
        ufr_info(link, "starting subscriber");

        // initialize the mosquitto client
        obj->mosq = mosquitto_new(NULL, true, obj);
        if ( obj->mosq == NULL ) {
            return ufr_error(link, 1, "failed to create mosquitto client");
        }

        // connect the mosquitto client
        if ( mosquitto_connect(obj->mosq, shr->broker_hostname, shr->broker_port, 60) != MOSQ_ERR_SUCCESS) {
            return ufr_error(link, 1, "connecting to MQTT broker failed");
        }

        // configure the subscriber
        mosquitto_subscribe(obj->mosq, NULL, shr->topic_name, 0);
        mosquitto_message_callback_set(obj->mosq, urf_gtw_mqtt_recv_cb);
        ufr_info(link, "connected");
        obj->start_type = UFR_START_SUBSCRIBER;
    }

    // success
    return UFR_OK;
}

static
void urf_gtw_mqtt_stop(link_t* link, int type) {
    if ( type == UFR_STOP_CLOSE ) {
        // free the private object
        ufr_info(link, "close link");
        ll_obj_t* obj = link->gtw_obj;
        mosquitto_destroy(obj->mosq);
        free(obj);
        link->gtw_obj = NULL;

        // free library
        if ( g_mosq_count == 1 ) {
            ufr_info(link, "closing the libmosquitto library");
            mosquitto_lib_cleanup();
            g_mosq_count = 0;
        } else {
            g_mosq_count -= 1;
        }
    }
}

static
int urf_gtw_mqtt_recv(link_t* link) {
    ll_obj_t* obj = link->gtw_obj;
    if (obj->start_type != UFR_START_SUBSCRIBER) {
        ufr_error(link, 1, "link is not subscriber");
        return -1;
    }

    // wait for the message
    while( obj->is_received == false ) {
        mosquitto_loop(obj->mosq, 1, 1);
    }
    obj->is_received = false;

    // decoder the message
    if ( link->dcr_api != NULL ) {
        link->dcr_api->recv_cb(link, obj->msg_data, obj->msg_size);
    }

    return UFR_OK;
}

static
int urf_gtw_mqtt_recv_async(link_t* link) {
    ll_obj_t* obj = link->gtw_obj;
    if (obj->start_type != UFR_START_SUBSCRIBER) {
        ufr_error(link, 1, "link is not subscriber");
        return -1;
    }

    // wait for the message
    mosquitto_loop(obj->mosq, 1, 1);

    // Case received a message
    if ( obj->is_received == true ) {
        obj->is_received = false;

        // decoder the message
        if ( link->dcr_api != NULL ) {
            link->dcr_api->recv_cb(link, obj->msg_data, obj->msg_size);
        }

        // there is a message
        return UFR_OK;
    }

    // No message
    return -1;
}

static
size_t urf_gtw_mqtt_read(link_t* link, char* buffer, size_t max_size) {
    ll_obj_t* obj = link->gtw_obj;

    if ( obj == NULL || obj->msg_data == NULL ) {
        return 0;
    }
    if ( obj->msg_read_idx >= obj->msg_size ) {
        return 0;
    }

    const size_t rest = obj->msg_size - obj->msg_read_idx;
    if ( max_size > rest ) {
        memcpy(buffer, &obj->msg_data[obj->msg_read_idx], rest);
        obj->msg_read_idx += rest;
        return rest;
    }

    memcpy(buffer, &obj->msg_data[obj->msg_read_idx], max_size);
    obj->msg_read_idx += max_size;
    return max_size;
}

static
size_t urf_gtw_mqtt_write(link_t* link, const char* buffer, size_t size) {
    ll_shr_t* shr = link->gtw_shr;
    ll_obj_t* obj = link->gtw_obj;
    if (obj->start_type != UFR_START_PUBLISHER) {
        ufr_error(link, 1, "link is not publisher");
        return 0;
    }

    if ( obj->mosq == NULL ) {
        return ufr_error(link, 1, "aaa");
    }

    ufr_info(link, "writing %ld bytes on %s", size, shr->topic_name);
    const int error = mosquitto_publish(obj->mosq, NULL, shr->topic_name, size, buffer, MQTT_QOS_0, false);
    if ( error != MOSQ_ERR_SUCCESS ) {
        return ufr_error(link, 0, "error");
    }
    return size;
}

static
ufr_gtw_api_t urf_gtw_mqtt_socket_api = {
	.type = urf_gtw_mqtt_type,
	.state = urf_gtw_mqtt_state,
	.size = urf_gtw_mqtt_size,
	.boot = urf_gtw_mqtt_boot,
	.start = urf_gtw_mqtt_start,
	.stop = urf_gtw_mqtt_stop,
	.copy = NULL,
    .recv = urf_gtw_mqtt_recv,
    .recv_async = urf_gtw_mqtt_recv_async,
	.read = urf_gtw_mqtt_read,
	.write = urf_gtw_mqtt_write
};

// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_mqtt_new_topic(link_t* link, int type) {
	ufr_init_link(link, &urf_gtw_mqtt_socket_api);
    link->type_started = type;
	return UFR_OK;
}

const char* urf_gtw_mqtt_list(uint8_t idx) {
    if ( idx > 1 ) {
        return NULL;
    }
    return "topic";
}
