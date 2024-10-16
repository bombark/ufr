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

#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <ufr.h>
 
using namespace std;
using namespace cv;

// ============================================================================
//  Main
// ============================================================================

int main() {
    // Begin
    // link_t topic = ufr_sys_publisher22("video", "@new zmq:topic @coder msgpack @port 4000");
    // link_t topic = ufr_publisher("@new video:topic @@new zmq:topic @@coder msgpack @@port 3000");
    link_t topic = ufr_publisher("@new zmq:topic @coder msgpack @port 3001");
    VideoCapture cap(0); 
    
    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    
    vector<uchar> buf;
    for(int i=0; i<100; i++) {
        Mat frame;
        cap >> frame;
  
        // If the frame is empty, break immediately
        if (frame.empty()) {
            break;
        }
 
        imencode(".jpg", frame, buf);
        ufr_put(&topic, "ii", frame.rows, frame.cols);
        // ufr_put_raw(&topic, (uint8_t*) frame.data, frame.total() );
        ufr_put_raw(&topic, (uint8_t*) buf.data(), buf.size() );
        ufr_put(&topic, "\n");
        
        // Display the resulting frame
        // imshow( "Frame", frame );
 
        // Press  ESC on keyboard to exit
        char c = (char) waitKey(1000);
        if( c == 27 ) {
            break;
        }
    }
  
    // When everything done, release the video capture object
    cap.release();
    destroyAllWindows();
   
    // fim
    return 0;
}

int main_origin() {
    // Begin
    link_t topic = ufr_publisher("@new zmq:topic @coder msgpack @debug 4 @port 4000");
    VideoCapture cap(0); 
    
    // Check if camera opened successfully
    if (!cap.isOpened()) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    
    vector<uchar> buf;
    for(int i=0; i<100; i++) {
        Mat frame;
        cap >> frame;
  
        // If the frame is empty, break immediately
        if (frame.empty()) {
            break;
        }
 
        imencode(".jpg", frame, buf);

        // ufr_put_raw(&topic, (uint8_t*) buf.data(), buf.size() );
        
        ufr_put(&topic, "i\n", 10+i);

        // Display the resulting frame
        // imshow( "Frame", frame );
 
        // Press  ESC on keyboard to exit
        char c = (char) waitKey(1000);
        if( c == 27 ) {
            break;
        }
    }
  
    // When everything done, release the video capture object
    cap.release();
    destroyAllWindows();
   
    // fim
    return 0;
}