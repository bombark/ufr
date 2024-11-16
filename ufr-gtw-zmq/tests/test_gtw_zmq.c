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
#include <ufr_test.h>
#include <pthread.h>

int ufr_gtw_zmq_new_topic(link_t* link, const int type);

// ============================================================================
//  Test 1
// ============================================================================

ufr_args_t g_test1_args = {.text="@host 127.0.0.1 @port 3000 @debug 0"};

void test1_publisher() {
    printf("Starting publisher\n");
    link_t link;
    assert( ufr_gtw_zmq_new_topic(&link, UFR_START_PUBLISHER) == UFR_OK );  
    assert( ufr_boot_gtw(&link,&g_test1_args) == UFR_OK );
    assert( ufr_enc_sys_new_std(&link, 0) == UFR_OK );
    assert( ufr_boot_enc(&link,&g_test1_args) == UFR_OK );
    assert( ufr_start_publisher(&link,&g_test1_args) == UFR_OK );
    ufr_put(&link, "iii\n", 10, 20, 30);
    ufr_close(&link);
}

void* test1_subscriber(void* ptr) {
    printf("Starting subscriber\n");
    link_t link;
    assert( ufr_gtw_zmq_new_topic(&link, UFR_START_SUBSCRIBER) == UFR_OK );
    assert( ufr_boot_gtw(&link,&g_test1_args) == UFR_OK );
    assert( ufr_dcr_sys_new_std(&link, 0) == UFR_OK );
    assert( ufr_boot_dcr(&link,&g_test1_args) == UFR_OK );
    assert( ufr_start_subscriber(&link,&g_test1_args) == UFR_OK );

    int a=0,b=0,c=0;
    ufr_get(&link, "^iii", &a, &b, &c);
    printf("%d %d %d\n", a,b,c);
    assert( a==10 );
    assert( b==20 );
    assert( c==30 );

    return NULL;
}

void test1() {
    pthread_t thread_sub;
    pthread_create(&thread_sub, NULL, test1_subscriber, NULL);
    test1_publisher();
    pthread_join(thread_sub, NULL);
}

// ============================================================================
//  Main
// ============================================================================

int main() {
    test1();
	return 0;
}