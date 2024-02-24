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

static 
void activate(GtkApplication* app, gpointer user_data) {
    GtkWidget* window = gtk_application_window_new (app);
    gtk_window_set_title (GTK_WINDOW (window), "Window");
    gtk_window_set_default_size (GTK_WINDOW (window), 400, 400);
    gtk_window_present (GTK_WINDOW (window));

    g_box = gtk_box_new (GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_halign (g_box, GTK_ALIGN_CENTER);
    gtk_widget_set_valign (g_box, GTK_ALIGN_CENTER);
    gtk_window_set_child (GTK_WINDOW (window), g_box);

    g_started = 1;
}

static 
void* main_gtk(void*) {
    g_app = gtk_application_new ("org.gtk.example", G_APPLICATION_FLAGS_NONE);
    g_signal_connect (g_app, "activate", G_CALLBACK (activate), NULL);
    int status = g_application_run (G_APPLICATION (g_app), 0, NULL);
    g_object_unref (g_app);
}


static 
void print_hello (GtkWidget* widget, gpointer data) {
    g_print ("Hello World\n");
}

// ============================================================================
//  File
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
int gtw_gtk_boot(link_t* link, const lt_args_t* args) {
    g_started = 0;
    pthread_create(&g_thread, NULL, main_gtk, NULL);
    
    // wait for activation function
    while ( g_started == 0 );
	return LT_OK;
}

static
int gtw_gtk_start(link_t* link, int type, const lt_args_t* args) {

	return 0;
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
    printf("%s\n", buffer);

    GtkWidget *label = gtk_label_new (NULL);
    gtk_label_set_markup (GTK_LABEL (label), buffer);
    gtk_box_append(GTK_BOX(g_box), label);

    GtkWidget* button = gtk_button_new_with_label("Hello World");
    gtk_box_append(GTK_BOX(g_box), button);
    g_signal_connect(button, "clicked", G_CALLBACK(print_hello), NULL);

    return 0;
}

static
void gtw_gtk_recv(link_t* link) {

}

static
ufr_gtw_api_t gtw_gtk = {
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
//  Public Functions
// ============================================================================

int lt_new_gtk_socket(link_t* link, int type) {
	link->gtw_api = &gtw_gtk;
	return LT_OK;
}

const char* gtw_gtk_list() {
    return "socket";
}
