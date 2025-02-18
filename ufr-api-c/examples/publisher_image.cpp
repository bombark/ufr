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
//  Private
// ============================================================================

int get_frame_and_send(VideoCapture& cap, link_t* topic, int count) {
    Mat frame;
    cap >> frame;

    // If the frame is empty, break immediately
    if (frame.empty()) {
        return 1;
    }

    // Dont send the frame, decrease the framerate
    if ( (count % 4) != 0 ) {
        return UFR_OK;
    }

    // convert from bgr to rgb
    // printf("%d\n", count);
    Mat frame_rgb;
    cvtColor(frame, frame_rgb, cv::COLOR_BGR2RGB);

    // send the image
    ufr_put(topic, "s", "rgb8");
    ufr_put(topic, "iii", frame_rgb.rows, frame_rgb.cols, frame_rgb.cols*3);
    ufr_put_raw(topic, frame_rgb.data, frame_rgb.total()*3);
    ufr_put(topic, "\n");

    return UFR_OK;
}

// ============================================================================
//  Main
// ============================================================================

int main(int argc, char** argv) {
    // Open the first camera
    VideoCapture cap_1(0); 
    if ( !cap_1.isOpened() ) {
        cout << "Error opening video stream or file" << endl;
        return -1;
    }
    // cap_1.set(CV_CAP_PROP_FPS, 24);

    link_t topic_1 = ufr_publisher("@new ros_humble:topic @msg image @topic camera1 @debug 0");
    int num_cap = 1;

    // Open the second camera
    link_t topic_2;
    VideoCapture cap_2(1);
    if ( cap_2.isOpened() == true ) {
        num_cap = 2;
        topic_2 = ufr_publisher("@new ros_humble:topic @msg image @topic camera2");
    }

    int count = 0;

    // Main loop for just one Camera
    if ( num_cap == 1 ) {
        printf("One camera\n");
        while ( ufr_loop_ok() ) {
            if ( get_frame_and_send(cap_1, &topic_1, count) != UFR_OK ) {
                break;
            }
            count += 1;
        }
        cap_1.release();

    // Main loop for two Cameras
    } else if ( num_cap == 2 ) {
        printf("Two cameras\n");
        while ( ufr_loop_ok() ) {
            if ( get_frame_and_send(cap_1, &topic_1, count) != UFR_OK ) {
                break;
            }
            if ( get_frame_and_send(cap_2, &topic_2, count) != UFR_OK ) {
                break;
            }
            count += 1;
        }
        cap_1.release();
        cap_2.release();
    }

    // fim
    return 0;
}