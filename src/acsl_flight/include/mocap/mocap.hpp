/***********************************************************************************************************************
 * Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

 /**********************************************************************************************************************  
 * Part of the code in this file leverages the following material.
 *
 * Referenced:  https://github.com/ros-drivers/transport_drivers/tree/main
 *              Copyright 2021 LeoDrive.
 *            
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *               
 *                  http://www.apache.org/licenses/LICENSE-2.0
 *              
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        mocap.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for UDP socket as a lifecycle node.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef MOCAP_HPP_
#define MOCAP_HPP_

#include "udp_driver.hpp"                         // Include for setting up the udp networking.
#include "global_config.hpp"                      // Include this for the communication ip and port setup.
#include "px4_defines.hpp"                        // Include for the PX4_define terms in a central location.            
#include "global_helpers.hpp"                     // Include for the flightstack global functions

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <bit>
#include <ctime>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include "vehicle_class.hpp"
#include "flight_log.hpp"

namespace lc = rclcpp_lifecycle;
namespace fl = _flight_log_;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

// Use the namespace _flightstack_ for the global functions
using namespace _flightstack_;

namespace _drivers_
{
namespace _udp_driver_
{

// Struct for mocap states that come over the udp.
struct mocap_states
{
  double control_time;
  double x; 
  double y;
  double z;
  double q0;
  double q1;
  double q2;
  double q3;
  double vx;
  double vy;
  double vz;
  double rollspeed;
  double pitchspeed;
  double yawspeed;
};

/// \brief MocapUdpReceiverNode class which can receive UDP datagrams
class MocapUdpReceiverNode final
  : public lc::LifecycleNode, public fl::blackbox
{
public:
    /// \brief Constructor which accepts IoContext
    /// \param[in] ctx A shared IoContext
    /// \param[in] pointer to vehicle states in flight_bridge
    /// \param[in] string to the global flight run directory
    MocapUdpReceiverNode(const IoContext & ctx, vehicle_states* d, const std::string & global_log_dir);

    /// \brief Destructor - required to manage owned IoContext
    ~MocapUdpReceiverNode();

    /// \brief Callback from transition to "configuring" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_configure(const lc::State & state) override;

    /// \brief Callback from transition to "activating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_activate(const lc::State & state) override;

    /// \brief Callback from transition to "deactivating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_deactivate(const lc::State & state) override;

    /// \brief Callback from transition to "unconfigured" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_cleanup(const lc::State & state) override;

    /// \brief Callback from transition to "shutdown" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_shutdown(const lc::State & state) override;

    /// \brief Callback for receiving a UDP datagram
    void receiver_callback(const std::vector<uint8_t> & buffer);

private: 
    /// \brief Get the parameters for the ip and port to ping
    void get_params();

    /// Pointer to the asio context owned by this node for async communication
    std::unique_ptr<IoContext> m_owned_ctx{};
    
    /// String for the ip
    std::string m_ip{};
        
    /// String for the port
    uint16_t m_port{};
    
    /// Pointer for the udp driver which wraps the udp socket
    std::unique_ptr<UdpDriver> m_udp_driver;
    
    /// Publisher for publishing the mocap message
    lc::LifecyclePublisher<px4_msgs::msg::VehicleOdometry>::SharedPtr mocap_publisher_;

    /// Instance of the vehicle_ptr to point to the vehicle state object in
    /// flight_bridge.cpp
    vehicle_states* vehicle_ptr;

    /// Declare variables to store parsed data and declare it as internal members "_im"
    mocap_states mc_im;

    /// \brief Debugger function to output the mocap data.
    void debugMocapData2screen();

    /// \brief Implementing virtual functions from blackbox.
    void logInitHeaders();
    bool logInitLogging();
    void logLogData();

    /// \brief Directory for logging mocap data.
    std::string flight_run_log_directory;


};  

}   // namespace _udp_driver_
}   // namesapce _drivers_

#endif  // MOCAP_HPP_