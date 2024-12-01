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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int ufr_gtw_posix_new_pipe(link_t* link, int type);
int ufr_dcr_sys_new_std(link_t* link, int type);
int ufr_enc_sys_new_std(link_t* link, int type);


#define UFR_TEST_EQUAL(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_I32(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_U32(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %u, but expected %u\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_F32(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %f, but expected %f\n", __FILE__, __LINE__, current, expected ); exit(1); }


#define UFR_TEST_EQUAL_I64(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %ld, but expected %ld\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_U64(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %lu, but expected %lu\n", __FILE__, __LINE__, current, expected ); exit(1); }

#define UFR_TEST_EQUAL_F64(current, expected) if ( current == expected ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %g, but expected %g\n", __FILE__, __LINE__, current, expected ); exit(1); }


#define UFR_TEST_EQUAL_STR(current, expected) if ( strcmp(current, expected) == 0 ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is \"%s\", but expected \"%s\"\n", __FILE__, __LINE__, current, expected); exit(1); }


#define UFR_TEST_TRUE(current) if ( current ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected != 0\n", __FILE__, __LINE__, current); exit(1); }

#define UFR_TEST_FALSE(current) if ( !current ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected 0\n", __FILE__, __LINE__, current); exit(1); }

#define UFR_TEST_ZERO(current) if ( current == 0 ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected 0\n", __FILE__, __LINE__, current); exit(1); }

#define UFR_TEST_NULL(current) if ( current == NULL ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %p, but expected NULL\n", __FILE__, __LINE__, current); exit(1); }

#define UFR_TEST_OK(current) if ( current == UFR_OK ) { ufr_test_inc_count(); } else { printf("Error:%s:%d: the value is %d, but expected %d\n", __FILE__, __LINE__, current, UFR_OK); exit(1); }
 
