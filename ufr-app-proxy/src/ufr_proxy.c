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
#include <string.h>
#include <ufr.h>
	
// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    // get the arguments
    char const* msg_format = NULL;
    char const* input_args = NULL;
    char const* output_args = NULL;
    for (int i=1; i<argc; i++) {
        const char* word = argv[i];
        const char c = word[0];
        if ( c == '-' ) {
            if ( strcmp(word, "-from") == 0 ) {
                if ( i+1 < argc ) {
                    input_args = argv[i+1];
                }
            } else if ( strcmp(word, "-to") == 0 ) {
                if ( i+1 < argc ) {
                    output_args = argv[i+1];
                }
            } else if ( strcmp(word, "-msg") == 0 ) {
                if ( i+1 < argc ) {
                    msg_format = argv[i+1];
                }
            }
        }
    }

    // check all the arguments
    if ( input_args == NULL ) {
        input_args = "@new posix:stdin @coder std:csv";
    }
    if ( output_args == NULL ) {
        output_args = "@new posix:stdout @coder std:csv";
    }
    if ( msg_format == NULL ) {
        return 1;
    }

    // show the arguments    
    fprintf(stderr, "#  Input: %s\n", input_args);
    fprintf(stderr, "# Output: %s\n", output_args);
    fprintf(stderr, "# Format: %s\n", msg_format);

    // prepare the stdin link
    ufr_input_init(input_args);
    // todo: sair se ocorrer algum erro

    // open the publisher
    ufr_output_init(output_args);
    // todo: sair se ocorrer algum erro

    // read the data from stdin and send it to the link
    int num;
    char buffer[512];
    while( 1 ) {
        // wait for the input message
        if ( ufr_input_recv() == false ) {
            break;
        }

        // prepare the package to send
        for (char const* format=msg_format; *format!='\0'; format++ ) {
            char type = *format;

            // pack a integer
            if ( type == 'i' ) {
                int val;
                ufr_input("i", &val);
                ufr_output("i", val);

            // pack a float
            } else if ( type == 'f' ) {
                int val;
                ufr_input("f", &val);
                ufr_output("f", val);

            // pack a string
            } else if ( type == 's' ) {
                char val[1024];
                ufr_input("s", val);
                ufr_output("s", val);
            }
        }

        // send the package
        ufr_output("\n");
    }

    // success
    return 0;
}