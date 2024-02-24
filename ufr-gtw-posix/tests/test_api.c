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

int ufr_gtw_posix_new_dir(link_t* link, const ufr_args_t* args);
int ufr_new_gtw_posix_pipe(link_t* link, const ufr_args_t* args);

// ============================================================================
//  Tests
// ============================================================================

void test_pipe() {
	printf("%s\n", __func__);
	link_t pipe;
	ufr_args_t args = {.text=""};
	// assert( ufr_new_posix_pipe(&pipe, &args) == UFR_OK );
	
	char buffer[16];
	for (int i=0; i<10; i++) {
		const size_t sent = ufr_write(&pipe, "felipe", 6);
		assert(sent == 6);
		const size_t recv = ufr_read(&pipe, buffer, sizeof(buffer));
		assert(recv == 6);
		assert(strcmp(buffer, "felipe") == 0);
	}

	ufr_close(&pipe);
	printf("\tOK\n");
}

void test_dir() {
	link_t dir;
	ufr_args_t args = {.text="@path ./pasta"};
	assert( ufr_new_posix_dir(&dir, &args) == UFR_OK );

	// ufr_cd(&dir, "name");
	ufr_put(&dir, "^s\n", "felipe bombardelli");

}

// ============================================================================
//  Main
// ============================================================================

int main() {
	// test_dir();
	// test_pipe();
	return 0;
}