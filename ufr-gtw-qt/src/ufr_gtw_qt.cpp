/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Felipe Bombardelli
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

#include <stdio.h>
#include <pthread.h>
#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QMessageBox>
#include <ufr.h>

QApplication* g_app;
QWidget* g_window;

pthread_t g_thread;
volatile int g_started = 0;

link_t g_pipe;
link_t g_pipe_event;

// ============================================================================
//  Private Functions
// ============================================================================

/*
static 
void gtk_cb_onClick (GtkWidget* widget, gpointer data) {
    // g_print ("Hello World\n");
    int len = strlen(data);
    ufr_write(&g_pipe_event, data, len);
}
*/


static 
void* main_qt(void*) {
    // initialize the client/server pipe
    g_pipe = ufr_new_pipe();
    g_pipe_event = ufr_new_pipe();
    ufr_dcr_sys_new_std(&g_pipe_event, 0);
    ufr_enc_sys_new_std(&g_pipe_event, 0);
    g_pipe_event.log_level = 4;
    ufr_boot_dcr(&g_pipe_event, NULL);

    int argc = 0;
    char* argv[] = {"", NULL};
    g_app = new QApplication(argc, argv);

    g_window = new QWidget();
    g_window->setWindowTitle("Hello World Qt");
    g_window->resize(300, 200);
    g_window->show();

    g_started = 1;
    g_app->exec();

    ufr_write(&g_pipe_event, "end", 4);
    return NULL;
}


// ============================================================================
//  GTK Client 
// ============================================================================

static
int gtw_qt_type(const link_t* link) {
	return 0;
}

static
int gtw_qt_state(const link_t* link){
	return 0;
}

static
size_t gtw_qt_size(const link_t* link, int type){
	return 0;
}

static
int gtw_qt_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
int gtw_qt_start(link_t* link, int type, const ufr_args_t* args) {
    if ( g_started == 0 ) {
        pthread_create(&g_thread, NULL, main_qt, NULL);
        while ( g_started == 0 );
    }

    return UFR_OK;
}

static
void gtw_qt_stop(link_t* link, int type) {
    pthread_join(g_thread, NULL); 
}

static
int gtw_qt_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gtw_qt_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t gtw_qt_write(link_t* link, const char* buffer, size_t length) {
    printf("sent %s\n", buffer);
    ufr_write(&g_pipe, buffer, length);
    return 0;
}

static
int gtw_qt_recv(link_t* link) {
    int retval = ufr_recv(&g_pipe);

    return retval;
}

static
ufr_gtw_api_t gtw_gtk = {
    .name = "qt",
	.type = gtw_qt_type,
	.state = gtw_qt_state,
	.size = gtw_qt_size,
	.boot = gtw_qt_boot,
	.start = gtw_qt_start,
	.stop = gtw_qt_stop,
	.copy = gtw_qt_copy,
	.read = gtw_qt_read,
	.write = gtw_qt_write,
    .recv = gtw_qt_recv,
};

// ============================================================================
//  GTK Event 
// ============================================================================

static
int gtw_qt_event_type(const link_t* link) {
	return 0;
}

static
int gtw_qt_event_state(const link_t* link){
	return 0;
}

static
size_t gtw_qt_event_size(const link_t* link, int type){
	return 0;
}

static
int gtw_qt_event_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
int gtw_qt_event_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

static
void gtw_qt_event_stop(link_t* link, int type) {
    
}

static
int gtw_qt_event_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gtw_qt_event_read(link_t* link, char* buffer, size_t length) {
    return ufr_read(&g_pipe_event, buffer, length);
}

static
size_t gtw_qt_event_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int gtw_qt_event_recv(link_t* link) {
    return ufr_recv(&g_pipe_event);
}

static
ufr_gtw_api_t gtw_qt_event = {
    .name = "qt/event",
	.type = gtw_qt_event_type,
	.state = gtw_qt_event_state,
	.size = gtw_qt_event_size,
	.boot = gtw_qt_event_boot,
	.start = gtw_qt_event_start,
	.stop = gtw_qt_event_stop,
	.copy = gtw_qt_event_copy,
	.read = gtw_qt_event_read,
	.write = gtw_qt_event_write,
    .recv = gtw_qt_event_recv,
};


// ============================================================================
//  Public Functions
// ============================================================================

extern "C" {

int ufr_gtw_qt_new(link_t* link, int type) {
    ufr_init_link(link, &gtw_gtk);
	return UFR_OK;
}

int ufr_gtw_qt_new_socket(link_t* link, int type) {
	ufr_init_link(link, &gtw_gtk);
	return UFR_OK;
}

int ufr_gtw_qt_new_topic(link_t* link, int type) {
	ufr_init_link(link, &gtw_qt_event);
    link->dcr_api = g_pipe_event.dcr_api;
    link->dcr_obj = g_pipe_event.dcr_obj;
	return UFR_OK;
}

}

