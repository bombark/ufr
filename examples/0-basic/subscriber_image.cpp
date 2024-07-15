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

#include <iostream>
#include <stdio.h>
#include <ufr.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    // open link
    link_t link = ufr_subscriber("@new zmq:topic @coder msgpack @port 3000 @debug 0");

    // alloc space for the jpeg image
    std::vector<uint8_t> buffer;
    buffer.reserve(1024*1024);

    // main loop
    for (int i=0; i<100; i++) {
        // wait for the message
        if ( ufr_recv(&link) == false ) {
            break;
        }
        
        // copia a imagem
        const size_t size = ufr_get_size(&link);
        buffer.resize(size);
        ufr_get_raw(&link, buffer.data(), size);
        Mat image = imdecode(buffer, cv::IMREAD_COLOR);

        // mostra a imagem
        cout << image.cols << " " << image.rows << endl;
        if ( image.cols > 0 ) {
            imshow("teste", image);
            waitKey(1);
        }
    }

    // end
    ufr_close(&link);
    return 0;
}