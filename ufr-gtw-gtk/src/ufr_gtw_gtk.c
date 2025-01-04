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
#include <gtk/gtk.h>
#include <pthread.h>
#include <ufr.h>

GtkApplication* g_app;
pthread_t g_thread;

GtkWidget* g_box;
volatile int g_started;

link_t g_pipe;
link_t g_pipe_event;

// ============================================================================
//  Private Functions
// ============================================================================

static 
void gtk_cb_onClick (GtkWidget* widget, gpointer data) {
    // g_print ("Hello World\n");
    int len = strlen(data);
    ufr_write(&g_pipe_event, data, len);
}

static 
void gtk_cb_onActivate(GtkApplication* app, gpointer user_data) {
    GtkWidget* window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
    gtk_window_present (GTK_WINDOW (window));

    g_box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_halign (g_box, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (g_box, GTK_ALIGN_CENTER);
    gtk_window_set_child (GTK_WINDOW (window), g_box);

    // mark started
    g_started = 1;
}

static 
void* main_gtk(void*) {
    // initialize the client/server pipe
    g_pipe = ufr_new_pipe();
    g_pipe_event = ufr_new_pipe();
    ufr_dcr_sys_new_std(&g_pipe_event, 0);
    ufr_enc_sys_new_std(&g_pipe_event, 0);
    g_pipe_event.log_level = 4;
    ufr_boot_dcr(&g_pipe_event, NULL);

    g_app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
    g_signal_connect (g_app, "activate", G_CALLBACK (gtk_cb_onActivate), NULL);
    int status = g_application_run (G_APPLICATION (g_app), 0, NULL);
    g_object_unref (g_app);
    ufr_loop_set_end();

    ufr_write(&g_pipe_event, "end", 4);
}


// ============================================================================
//  GTK Client 
// ============================================================================

static
int gtw_gtk_type(const link_t* link) {
	return 0;
}

static
int gtw_gtk_state(const link_t* link){
	return 0;
}

static
size_t gtw_gtk_size(const link_t* link, int type){
	return 0;
}

static
int gtw_gtk_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
int gtw_gtk_start(link_t* link, int type, const ufr_args_t* args) {
    g_started = 0;
    pthread_create(&g_thread, NULL, main_gtk, NULL);
    
    // wait for activation function
    while ( g_started == 0 );
    return UFR_OK;
}

static
void gtw_gtk_stop(link_t* link, int type) {
    pthread_join(g_thread, NULL); 
}

static
int gtw_gtk_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gtw_gtk_read(link_t* link, char* buffer, size_t length) {
    return 0;
}

static
size_t gtw_gtk_write(link_t* link, const char* buffer, size_t length) {
    printf("sent %s\n", buffer);
    ufr_write(&g_pipe, buffer, length);
    return 0;
}

static
int gtw_gtk_recv(link_t* link) {
    int retval = ufr_recv(&g_pipe);

    char gtk_buffer[1024];
    ufr_read(&g_pipe, gtk_buffer, 1024);
    printf("recv %s\n", gtk_buffer);

    ufr_args_t args = {.text=gtk_buffer};
    char* create = ufr_args_gets(&args, "@create", NULL);
    if ( create == NULL ) {
        return -1;
    }

    if ( strcmp(create, "button") == 0 ) {
        const char* text = ufr_args_gets(&args, "@text", "[NULL]");
        GtkWidget* button = gtk_button_new_with_label(text);
        gtk_box_append(GTK_BOX(g_box), button);

        char* id = malloc (32);
        strcpy(id, "aqui");
        g_signal_connect(button, "clicked", G_CALLBACK(gtk_cb_onClick), id);
    } else if ( strcmp(create, "label") == 0 ) {
        const char* text = ufr_args_gets(&args, "@text", "[NULL]");
        GtkWidget *label = gtk_label_new (NULL);
        gtk_label_set_markup (GTK_LABEL (label), text);
        gtk_box_append(GTK_BOX(g_box), label);
    } else {
        ufr_log(link, "Invalid component name");
    }

    return retval;
}

static
ufr_gtw_api_t gtw_gtk = {
    .name = "gtk",
	.type = gtw_gtk_type,
	.state = gtw_gtk_state,
	.size = gtw_gtk_size,
	.boot = gtw_gtk_boot,
	.start = gtw_gtk_start,
	.stop = gtw_gtk_stop,
	.copy = gtw_gtk_copy,
	.read = gtw_gtk_read,
	.write = gtw_gtk_write,
    .recv = gtw_gtk_recv,
};

// ============================================================================
//  GTK Event 
// ============================================================================

static
int gtw_gtk_event_type(const link_t* link) {
	return 0;
}

static
int gtw_gtk_event_state(const link_t* link){
	return 0;
}

static
size_t gtw_gtk_event_size(const link_t* link, int type){
	return 0;
}

static
int gtw_gtk_event_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
int gtw_gtk_event_start(link_t* link, int type, const ufr_args_t* args) {
    return UFR_OK;
}

static
void gtw_gtk_event_stop(link_t* link, int type) {
    
}

static
int gtw_gtk_event_copy(link_t* link, link_t* out) {
	return 0;
}

static
size_t gtw_gtk_event_read(link_t* link, char* buffer, size_t length) {
    return ufr_read(&g_pipe_event, buffer, length);
}

static
size_t gtw_gtk_event_write(link_t* link, const char* buffer, size_t length) {
    return 0;
}

static
int gtw_gtk_event_recv(link_t* link) {
    return ufr_recv(&g_pipe_event);
}

static
ufr_gtw_api_t gtw_gtk_event = {
    .name = "gtk_event",
	.type = gtw_gtk_event_type,
	.state = gtw_gtk_event_state,
	.size = gtw_gtk_event_size,
	.boot = gtw_gtk_event_boot,
	.start = gtw_gtk_event_start,
	.stop = gtw_gtk_event_stop,
	.copy = gtw_gtk_event_copy,
	.read = gtw_gtk_event_read,
	.write = gtw_gtk_event_write,
    .recv = gtw_gtk_event_recv,
};

// ============================================================================
//  Default Decoder
// ============================================================================

static
int ufr_dcr_lnk_boot(link_t* link, const ufr_args_t* args) {
    return UFR_OK;
}

static
void ufr_dcr_lnk_close(link_t* link) {

}

static
int ufr_dcr_lnk_recv(link_t* link, char* msg_data, size_t msg_size) {
    return -1;
}

static
int ufr_dcr_lnk_get_type(link_t* link) {
    return 0;
}

static
int ufr_dcr_lnk_get_u32(link_t* link, uint32_t* val, int nitems) {
    return UFR_OK;
}

static
int ufr_dcr_lnk_get_i32(link_t* link, int32_t* val, int nitems) {
    return 0;
}

static
int ufr_dcr_lnk_get_f32(link_t* link, float* val, int nitems) {
    return 0;
}

static
int ufr_dcr_lnk_get_str(link_t* link, char* ret_val, int maxbytes) {
    return UFR_OK;
}

static
int ufr_dcr_lnk_enter(link_t* link) {
    return UFR_OK;
}

static
int ufr_dcr_lnk_leave(link_t* link) {
    return UFR_OK;
}

static
ufr_dcr_api_t dcr_lnk_api = {
    .boot = ufr_dcr_lnk_boot,
    .close = ufr_dcr_lnk_close,
	.recv_cb = ufr_dcr_lnk_recv,
    .recv_async_cb = NULL,

    .next = NULL, 

    .get_type = NULL,

	.get_u32 = ufr_dcr_lnk_get_u32,
	.get_i32 = ufr_dcr_lnk_get_i32,
	.get_f32 = ufr_dcr_lnk_get_f32,

    .get_raw = NULL,
    .get_str = ufr_dcr_lnk_get_str,

    .enter = ufr_dcr_lnk_enter,
    .leave = ufr_dcr_lnk_leave
};



// ============================================================================
//  Public Functions
// ============================================================================

int ufr_gtw_gtk_new(link_t* link, int type) {
    ufr_init_link(link, &gtw_gtk);
	return UFR_OK;
}

int ufr_gtw_gtk_new_socket(link_t* link, int type) {
	ufr_init_link(link, &gtw_gtk);
	return UFR_OK;
}

int ufr_gtw_gtk_new_topic(link_t* link, int type) {
	ufr_init_link(link, &gtw_gtk_event);
    link->dcr_api = g_pipe_event.dcr_api;
    link->dcr_obj = g_pipe_event.dcr_obj;
	return UFR_OK;
}


