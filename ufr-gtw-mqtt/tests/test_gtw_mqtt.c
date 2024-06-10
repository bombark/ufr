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
//  HEADER
// ============================================================================

#include <assert.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ufr.h>

int ufr_gtw_mqtt_new_topic(link_t* link, int type);

// ============================================================================
//  Tests
// ============================================================================

void test_publisher() {
    link_t link;
    ufr_args_t args = {.text="@host 185.209.160.8 @topic test/topic"};
    assert( ufr_gtw_mqtt_new_topic(&link, UFR_START_PUBLISHER) == UFR_OK );
    assert( ufr_boot_gtw(&link, &args) == UFR_OK );
    assert( ufr_start_publisher(&link, &args) == UFR_OK );
    assert( ufr_write(&link, "teste", 5) == 5 );
    ufr_close(&link);
}

void test_subscriber() {
    char buffer[1024];
    link_t link;
    ufr_args_t args = {.text="@host 185.209.160.8 @topic test/topic"};
    
    assert( ufr_gtw_mqtt_new_topic(&link, UFR_START_SUBSCRIBER) == UFR_OK );
    assert( ufr_boot_gtw(&link, &args) == UFR_OK );
    assert( ufr_start_subscriber(&link, &args) == UFR_OK );
    ufr_recv(&link);
    ufr_read(&link, buffer, sizeof(buffer));
    printf("%s\n", buffer);
    ufr_close(&link);
}

void test_publisher_fmt() {
    char buffer[1024];
    link_t link = ufr_publisher("@new mqtt:topic @host 185.209.160.8 @topic test/topic @encoder msgpack:obj");
    ufr_put(&link, "iii\n", 40,50,60);
    ufr_close(&link);
}

void test_subscriber_fmt() {
    char buffer[1024];
    link_t link = ufr_subscriber("@new mqtt:topic @host 185.209.160.8 @topic test/topic @decoder msgpack:obj");
    ufr_start_subscriber(&link, NULL);
    int a,b,c;
    ufr_get(&link, "^iii", &a, &b, &c);
    printf("%d %d %d\n", a, b, c);
    ufr_close(&link);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test_publisher();
    test_subscriber();
    // test_publisher_fmt();
    // test_subscriber_fmt();
	return 0;
}