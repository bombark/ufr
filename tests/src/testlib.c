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

#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ufr.h>

// ============================================================================
//  Tests
// ============================================================================

void test_topic(link_t* link, const char* class_name) {

    int state = ufr_sys_lib_call_new(link, link->slot_gtw, class_name, UFR_START_PUBLISHER);
    assert( state == UFR_OK );

    const char* test_args = ufr_test_args(link);
    assert( test_args != NULL );

    ufr_args_t args = {.text=test_args};
    assert( ufr_boot_gtw(link, &args) == UFR_OK );
    assert( ufr_start(link, &args) == UFR_OK );
    assert( ufr_write(link, "opa\n", 4) == 4 );
    ufr_close(link);
}

void test_socket(link_t* link, const char* class_name) {
}


// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    const char* lib_file = "./ufr-gtw-posix/libufr_gtw_posix.so";
    char* pack_name = "ufr_gtw_posix";

    link_t link;
    assert( ufr_load_gtw_from_lib(&link, lib_file, pack_name) == UFR_OK );

    int i=0;
    // for (int i=0; i<10; i++) {
        const char* class_name = ufr_sys_lib_call_list(link.slot_gtw, i);
        if ( class_name == NULL ) {
            // break;
        }
        
        printf("%s\n", class_name);
        test_topic(&link, class_name);
    // }

    

    return 0;
}