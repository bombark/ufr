/* BSD 2-Clause License
 * 
 * Copyright (c) 2023, Visao Robotica e Imagem (VRI)
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
#include <ufr.h>
#include <unistd.h>
#include <assert.h>

// ============================================================================
//  Test 1
// ============================================================================

void publisher(const char* link_params) {
    link_t pub = ufr_publisher(link_params);

    // test 1 - just integers
    ufr_put(&pub, "ii\n", 10, 20);

    // test 2 - integers and floats
    ufr_put(&pub, "iiff\n", 10, 20, 44.5, 10.125);

    // test 3 - short strings
    ufr_put(&pub, "ss\n", "string1", "lorem ipsum");

    // test 4 - integer vector (1MB)
    {
        const int32_t vector_size = 1024*1024;
        int vector[vector_size];
        for (int i=0; i<vector_size; i++) {
            vector[i] = i;
        }
        ufr_put_ai32(&pub, vector, vector_size);
    }

    /*{
        const uint32_t vector_size = 1024*1024;
        int vector[vector_size];
        for (int i=0; i<vector_size; i++) {
            vector[i] = i;
        }
        ufr_put_au32(&pub, vector, vector_size);
    }*/

    // test 5 - float vector

    // test 6 - integer matrix

    // test 7 - color image
}

void subscriber(const char* link_params) {
    link_t sub = ufr_subscriber(link_params);

    // test 1
    {
        int a=0,b=0;
        ufr_get(&sub, "^ii", &a, &b);
        assert( a == 10 );
        assert( b == 20 );
    }

    // test 2
    {
        int a=0,b=0;
        float c=0,d=0;
        ufr_get(&sub, "^iiff", &a, &b, &c, &d);
        assert( a == 10 );
        assert( b == 20 );
        assert( c == 44.5 );
        assert( d == 10.125 );
    }

    // test 3
    {
        char text1[1024], text2[1024];
        ufr_get(&sub, "^ss", text1, text2);
        assert( strcmp(text1, "string1") == 0 );
        assert( strcmp(text2, "lorem ipsum") == 0 );
    }
}

// ============================================================================
//  ver o que fazer
// ============================================================================

#include <dirent.h>

void ufr_list_dl(char* out_list, const char* type, const char* path) {
    char prefix[32];
    const int prefix_size = snprintf(prefix, 32, "libufr_%s_", type);

    int pos = 0;
    struct dirent *ep;
    DIR *dp = opendir(path);
    if (dp != NULL) {
        while ( (ep = readdir(dp)) != NULL) {
            if ( strncmp(ep->d_name, prefix, prefix_size) == 0 ) {
                char* filename = &ep->d_name[prefix_size];
                char* name = strchr(filename, '.');
                name[0] = '\0';
                printf("%s\n", filename);

                pos += sprintf(&out_list[pos], "%s:", filename);
            }
        }
        closedir(dp);
    }
}

char* ufr_list_gtw() {
    char* list = malloc(1024);
    if ( list == NULL ) {
        return NULL;
    }

    list[0] = '\0';
    ufr_list_dl(list, "gtw", "/home/niceguy/workspace/ufr/install/lib");
    printf("%s\n",list);

    return list;
}

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    const char* test_params_default = "@new zmq:topic @port 2000 @coder msgpack";
    char const* test_params = test_params_default;
    if ( argc > 1 ) {
        test_params = argv[1];
    }
    
    int pid = fork();
    if ( pid == 0 ) {
        publisher(test_params);
    } else {
        subscriber(test_params);
    }
    return 0;
}