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

// ======================================================================================
//  Header
// ======================================================================================

#pragma once

#include <ufr.h>
#include <ros/ros.h>

struct Gateway {
    ros::NodeHandle node;
};

// Public Functions
extern "C" {
    // gateway
    int ufr_gtw_ros_melodic_new_topic(link_t* out, int type);

    // encoders
    int ufr_enc_ros_melodic_new_twist(link_t* link, int type);
    int ufr_enc_ros_melodic_new_twist(link_t* link, int type);
    int ufr_enc_ros_melodic_new_string(link_t* link, int type);
    int ufr_enc_ros_melodic_new_laserscan(link_t* link, int type);

    // decoders
    int ufr_dcr_ros_melodic_new_i16(link_t* link, int type);
    int ufr_dcr_ros_melodic_new_pose(link_t* link, int type);
    int ufr_dcr_ros_melodic_new_twist(link_t* link, int type);
    int ufr_dcr_ros_melodic_new_string(link_t* link, int type);
}
