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
#include <ufr.h>

// ============================================================================
//  Main
// ============================================================================

int main() {
    // open link
    link_t sub = ufr_subscriber("@new zmq:topic @coder msgpack");

    /*float lidar[5];
    ufr_recv(&sub);
    char type = ufr_get_type(&sub);
    int nbytes = ufr_get_nbytes(&sub);
    printf("%c %d\n", type, nbytes);
    ufr_get_pf32(&sub, lidar, 5);
    printf("%f %f %f %f %f\n", lidar[0], lidar[1], lidar[2], lidar[3], lidar[4]);*/

    // read 5 messages
    for (int i=0; i<5; i++) {
        int a,b,c;
        float d,e;
        char buffer[1024];
        ufr_get(&sub, "^iiiff", &a, &b, &c, &d, &e);
        printf("%d %d %d %f %f\n", a, b, c, d, e);
    }

    // end
    ufr_close(&sub);
    return 0;
}