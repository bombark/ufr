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
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>    // read

#include "ufr_message.h"

// ============================================================================
//  Message Functions
// ============================================================================

void message_init(message_t* message) {
    message->size = 0;
    message->max = MESSAGE_ITEM_SIZE;
    message->ptr = malloc(message->max);
}

void message_clear(message_t* message) {
    message->size = 0;
}

bool message_write_from_fd(ufr_buffer_t* message, int fd) {
    bool is_ok = true;

    const uint32_t part_len = 1024;
    while(1) {
        ufr_buffer_check_size(message, part_len);

        const size_t bytes = read(fd, &message->ptr[ message->size ], part_len-1);
        if ( bytes == 0 ) {
            message->size += bytes;
            break;

        } else if ( bytes > 0 ) {
            message->size += bytes;
            break;

        } else {
            is_ok = false;
            break;
        }
    }

    message->ptr[message->size] = '\0';
    return is_ok;
}