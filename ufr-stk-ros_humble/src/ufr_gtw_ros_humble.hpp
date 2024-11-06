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
        const char* sys_env_name = getenv("_");
        std::string node_name;
        for (int i=0; 1; i++) {
            const char c = sys_env_name[i];
            if ( c == '\0') {
                break;
            } else if ( 
                (c >= 'a' && c <= 'z') || ( c >= 'A' && c <= 'Z') || 
                (c >= '0' && c <= '9') || c == '_' ) {
                    node_name += c;
            }
        }

        rclcpp::Node* node = new rclcpp::Node(node_name);
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
    int count_proc = 0;
    volatile int count_recv = 0;

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
        m_message = msg;
        this->count_recv += 1;
        index = 0;
    }

    int recv_async(ll_gateway_t* gtw) {
// printf("%d %d\n", this->count_recv, this->count_proc);
        if ( this->count_proc == this->count_recv ) {
            rclcpp::spin_some( gtw->m_node );
            return -1;
        }

        const int count_proc_1 = this->count_proc+1;
        if ( count_proc_1 <= count_recv ) {
            this->count_proc = count_proc_1;
            return UFR_OK;
        }

        printf("ERROR ufr_dcr_ros_recv_async_cb\n");
        return -1;
    }

    int recv(ll_gateway_t* gtw) {
        while ( this->recv_async(gtw) != UFR_OK ) {
        }
        return UFR_OK;
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