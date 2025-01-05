/* BSD 2-Clause License
 * 
 * Copyright (c) 2024, Visao Robotica e Imagem (VRI)
 *  - Felipe Bombardelli <felipebombardelli@gmail.com>
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
 */
	
// ============================================================================
//  Header
// ============================================================================

#include <ufr.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "ufr_ssh.h"

typedef struct {
    ssh_session session;
} ssh_gtw_shr_t;

typedef struct {
    ssh_channel channel;
    sftp_session sftp;
} ssh_gtw_t;

// ============================================================================
//  Gateway
// ============================================================================

static
int    ufr_gtw_ssh_type(const link_t* link) {
    return 0;
}

static
int    ufr_gtw_ssh_state(const link_t* link) {
    return 0;
}

static
size_t ufr_gtw_ssh_size(const link_t* link, int type) {
    return 0;
}

static
int  ufr_gtw_ssh_boot(link_t* link, const ufr_args_t* args) {
    ssh_gtw_shr_t* gtw_shr = (ssh_gtw_shr_t*) malloc(sizeof(ssh_gtw_shr_t));

    gtw_shr->session = ssh_new();
    if (gtw_shr->session == NULL) {
        fprintf(stderr, "Falha ao criar a sessão SSH\n");
        exit(1);
    }

    const char* host = ufr_args_gets(args, "@host", "");
    const char* user = ufr_args_gets(args, "@user", "");

    ssh_options_set(gtw_shr->session, SSH_OPTIONS_HOST, host);
    ssh_options_set(gtw_shr->session, SSH_OPTIONS_USER, user);

    const int rc1 = ssh_connect(gtw_shr->session);
    if (rc1 != SSH_OK) {
        fprintf(stderr, "Falha ao conectar ao servidor SSH: %s\n", ssh_get_error(gtw_shr->session));
        ssh_free(gtw_shr->session);
        exit(1);
    }

    // Autenticação com senha
    const char* password = ufr_args_gets(args, "@password", "");
    const int rc2 = ssh_userauth_password(gtw_shr->session, NULL, password);
    if (rc2 != SSH_AUTH_SUCCESS) {
        fprintf(stderr, "Falha na autenticação: %s\n", ssh_get_error(gtw_shr->session));
        ssh_disconnect(gtw_shr->session);
        ssh_free(gtw_shr->session);
        exit(1);
    }

    // Criar um canal SSH
    ssh_gtw_t* gtw = (ssh_gtw_t*) malloc(sizeof(ssh_gtw_t));
    gtw->channel = ssh_channel_new(gtw_shr->session);
    if (gtw->channel == NULL) {
        fprintf(stderr, "Falha ao criar o canal SSH\n");
        ssh_disconnect(gtw_shr->session);
        ssh_free(gtw_shr->session);
        exit(1);
    }

    // Abrir o canal e executar o comando
    const int rc3 = ssh_channel_open_session(gtw->channel);
    if (rc3 != SSH_OK) {
        fprintf(stderr, "Falha ao abrir o canal SSH: %s\n", ssh_get_error(gtw_shr->session));
        ssh_channel_free(gtw->channel);
        ssh_disconnect(gtw_shr->session);
        ssh_free(gtw_shr->session);
        exit(1);
    }

    gtw->sftp = sftp_new(gtw_shr->session);
    if (gtw->sftp == NULL) {
        fprintf(stderr, "Falha ao abrir o SFTP SSH: %s\n", ssh_get_error(gtw_shr->session));
        exit(1);
    }

    link->gtw_shr = gtw_shr;
    link->gtw_obj = gtw;

    ufr_log(link, "connected on %s", host);

    return UFR_OK;
}

static
int  ufr_gtw_ssh_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_gtw_ssh_stop(link_t* link, int type) {
    if ( link->gtw_obj ) {
        ssh_gtw_t* gtw = (ssh_gtw_t*) link->gtw_obj;
        ssh_channel_send_eof(gtw->channel);
        ssh_channel_close(gtw->channel);
        ssh_channel_free(gtw->channel);
        free(gtw);
    }
    if ( link->gtw_shr ) {
        ssh_gtw_shr_t* gtw_shr = (ssh_gtw_shr_t*) link->gtw_shr;
        ssh_disconnect(gtw_shr->session);
        ssh_free(gtw_shr->session);
        free(gtw_shr);
    }

}

static
int  ufr_gtw_ssh_copy(link_t* link, link_t* out) {

}

static
size_t ufr_gtw_ssh_read(link_t* link, char* buffer, size_t length) {
    ssh_gtw_t* gtw = (ssh_gtw_t*) link->gtw_obj;
    return ssh_channel_read(gtw->channel, buffer, length, 0);
}

static
size_t ufr_gtw_ssh_write(link_t* link, const char* buffer, size_t length) {
    ssh_gtw_t* gtw = (ssh_gtw_t*) link->gtw_obj;
    int rc = ssh_channel_request_exec(gtw->channel, "ls");
    if (rc != SSH_OK) {
        ssh_gtw_shr_t* gtw_shr = (ssh_gtw_shr_t*) link->gtw_shr;
        fprintf(stderr, "Falha ao executar o comando: %s\n", ssh_get_error(gtw_shr->session));
        /*ssh_channel_close(channel);
        ssh_channel_free(channel);
        ssh_disconnect(my_ssh_session);
        ssh_free(my_ssh_session);*/
        exit(1);
    }

    return 0;
}

static
int ufr_gtw_ssh_recv(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_ssh_recv_async(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_ssh_send(link_t* link) {
    return UFR_OK;
}

static
int ufr_gtw_ssh_accept(link_t* link, link_t* out_client) {
    return UFR_OK;
}

static
const char* ufr_gtw_ssh_test_args(const link_t* link) {
    return "";
}

static
ufr_gtw_api_t ufr_gtw_ssh_api = {
    .name = "SSH",
	.type = ufr_gtw_ssh_type,
	.state = ufr_gtw_ssh_state,
	.size = ufr_gtw_ssh_size,

	.boot = ufr_gtw_ssh_boot,
	.start = ufr_gtw_ssh_start,
	.stop = ufr_gtw_ssh_stop,
	.copy = ufr_gtw_ssh_copy,

	.read = ufr_gtw_ssh_read,
	.write = ufr_gtw_ssh_write,

	.recv = ufr_gtw_ssh_recv,
	.recv_async = ufr_gtw_ssh_recv_async,

    .accept = ufr_gtw_ssh_accept,

    .test_args = ufr_gtw_ssh_test_args,
};


// ============================================================================
//  Public Function
// ============================================================================

int ufr_gtw_ssh_new(link_t* link, int type) {
    ufr_init_link(link, &ufr_gtw_ssh_api);
    return UFR_OK;
}