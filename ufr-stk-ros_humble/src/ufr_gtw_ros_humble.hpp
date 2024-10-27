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
#include "rclcpp/rclcpp.hpp"

class ll_gateway_t {
public:
    std::shared_ptr<rclcpp::Node> m_node;

public:
    ll_gateway_t() {
        rclcpp::Node* node = new rclcpp::Node("teste1");
        m_node.reset(node);
    }
};

// Template for encoders

template <typename T>
class ufr_ros_encoder_t {
public:
    int index = 0;
    T message;
    typename rclcpp::Publisher<T>::SharedPtr m_publisher;

public:
    ufr_ros_encoder_t(link_t* link, std::string topic_name) {
        ll_gateway_t* gtw = (ll_gateway_t*) link->gtw_obj;
        m_publisher = gtw->m_node->create_publisher<T>(topic_name, 10);
    }
};

// Template for Decoder

template <typename T>
class ufr_ros_decoder_t {
public:
    int index = 0;
    int index2 = 0;
    volatile bool m_is_received;
    T m_message;
    typename rclcpp::Subscription<T>::SharedPtr m_subscription;

public:
    ufr_ros_decoder_t(ll_gateway_t* gtw, std::string topic_name) {
        m_subscription = gtw->m_node->create_subscription<T> (
            topic_name, 10, 
            std::bind (
                &ufr_ros_decoder_t::topic_callback, 
                this, std::placeholders::_1
            )
        );
    }

    void topic_callback(const T& msg) {
        printf("Opa\n");
        m_message = msg;
        m_is_received = true;
        index = 0;
    }
};

// Public Functions

extern "C" {
    // gateway
    int ufr_gtw_ros_humble_new_topic(link_t* out, int type);

    // encoders
    int ufr_enc_ros_humble_new_twist(link_t* link, int type);
    int ufr_enc_ros_humble_new_twist(link_t* link, int type);
    int ufr_enc_ros_humble_new_string(link_t* link, int type);

    // decoders
    int ufr_dcr_ros_humble_new_pose(link_t* link, int type);
    int ufr_dcr_ros_humble_new_twist(link_t* link, int type);
    int ufr_dcr_ros_humble_new_string(link_t* link, int type);
}