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
 * File:        mocap.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 20, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Node definition for UDP socket as a lifecycle node.
 *              Writes messages to pixhawk mocap_odometry topic for 
 *              EFK2 fusion.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "mocap.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace _drivers_{
namespace _udp_driver_{

// Implementing virtual functions from Blackbox
void MocapUdpReceiverNode::logInitHeaders() {
  std::ostringstream oss;

  oss << ", "
      << "Controller time [s], "
      << "Mocap Position x [m], "
      << "Mocap Position y [m], "
      << "Mocap Position z [m], "
      << "Mocap Quaternion q0 [-], "
      << "Mocap Quaternion q1 [-], "
      << "Mocap Quaternion q2 [-], "
      << "Mocap Quaternion q3 [-], "
      << "Mocap Velocity x [m/s], "
      << "Mocap Velocity y [m/s], "
      << "Mocap Velocity z [m/s], "
      << "Mocap Angular velocity x [rad/s], "
      << "Mocap Angular velocity y [rad/s], "
      << "Mocap Angular velocity z [rad/s], ";

  BOOST_LOG(logger_mocapdata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool MocapUdpReceiverNode::logInitLogging() {
    try {
        // Generate the log file name
        std::stringstream log_ss;
        log_ss << flight_run_log_directory << "/mocap_log" << ".log";
        std::string mocap_log_filename = log_ss.str();

        // Add the "Tag" attribute with a constant value of "LogDataTag" to the logger
        logger_mocapdata.add_attribute("Tag", attrs::constant< std::string >("MocapTag"));

        // Define a synchronous sink with a text ostream backend
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> mocap_sink = boost::make_shared<text_sink>();

        // Add a stream to the backend (in this case, a file stream)
        mocap_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(mocap_log_filename));

        // Set the formatter for the sink
        mocap_sink->set_formatter(
          expr::stream
          << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
          << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
          << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
          << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
          << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
          << expr::smessage // Log message
        );

        // Add the sink to the logging core
        logging::core::get()->add_sink(mocap_sink);

        // Set a filter for the sink
        mocap_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "MocapTag");

        logging::add_common_attributes(); // Add attributes like timestamp

        // Initilaize the logging headers
        logInitHeaders();

        // Return true if all successful
        return true;       

    } catch (const std::filesystem::filesystem_error& e) {
        FLIGHTSTACK_ERROR("Filesystem error:", e.what());
    } catch (const std::exception& e) {
        FLIGHTSTACK_ERROR("Exception: ", e.what());
    } 

    return false; // IF there are errors
}

// Implementing virtual functions from Blackbox
void MocapUdpReceiverNode::logLogData() {
    // Log the data 
    std::ostringstream oss;

    oss << ", "
        << mc_im.control_time << ", "
        << mc_im.x << ", "
        << mc_im.y << ", "
        << mc_im.z << ", "
        << mc_im.q0 << ", "
        << mc_im.q1 << ", "
        << mc_im.q2 << ", "
        << mc_im.q3 << ", "
        << mc_im.vx << ", "
        << mc_im.vy << ", "
        << mc_im.vz << ", "
        << mc_im.rollspeed << ", "
        << mc_im.pitchspeed << ", "
        << mc_im.yawspeed << ", ";
    
    BOOST_LOG(logger_mocapdata) << oss.str();
}

/// \brief Constructor which accepts IoContext
/// \param[in] ctx A shared IoContext
/// \param[in] pointer to vehicle states in flight_bridge    
/// \param[in] string to the global flight run directory
MocapUdpReceiverNode::MocapUdpReceiverNode(
  const IoContext & ctx, vehicle_states* d, const std::string & global_log_dir)
: lc::LifecycleNode("mocap_udp_receiver_node"),
  m_udp_driver{new UdpDriver(ctx)},
  vehicle_ptr(d),
  flight_run_log_directory(global_log_dir)
{
  // Initialize the logging
  if(logInitLogging()) {FLIGHTSTACK_INFO("Mocap Logger Created");};

  // Get the parameters
  get_params();  
}

/// \brief Get the parameters for the ip and port to ping
void MocapUdpReceiverNode::get_params()
{
  m_ip = IP_IN_USE;     

  m_port = MOCAP_PORT;

  RCLCPP_INFO(get_logger(), "ip: %s", m_ip.c_str());
  RCLCPP_INFO(get_logger(), "port: %i", m_port);
}

/// \brief Destructor - required to manage owned IoContext
MocapUdpReceiverNode::~MocapUdpReceiverNode()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
    FLIGHTSTACK_INFO("Closed Mocap Thread");
  }
}

/// \brief Callback from transition to "configuring" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn MocapUdpReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  // Check if m_udp_driver is not null
  if (m_udp_driver) {
    RCLCPP_INFO(get_logger(), "UDP driver is initialized.");
  }

  // Create a Sensor Data QoS profile - 285-330Hz is data frequency of MOCAP. Message every 3.51-3.03ms. 
  // Therefore, we want to discard old messages in the queue.
  rclcpp::QoS qos_settings = rclcpp::SensorDataQoS();  // Use default Sensor Data QoS profile
	qos_settings.keep_last(1);  												 // Keep only the last message

  // Create publisher for mocap to the pixhawk visual odometry topic
  mocap_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
    "/fmu/in/vehicle_visual_odometry", qos_settings);

  try {
    
    m_udp_driver->init_receiver(m_ip, m_port);
    m_udp_driver->receiver()->open();
    m_udp_driver->receiver()->bind();
    m_udp_driver->receiver()->asyncReceive(
      std::bind(&MocapUdpReceiverNode::receiver_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP receiver: %s:%i - %s",
      m_ip.c_str(), m_port, ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(get_logger(), "UDP receiver successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "activating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn MocapUdpReceiverNode::on_activate(const lc::State & state)
{
  (void)state;
  mocap_publisher_->on_activate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "deactivating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn MocapUdpReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;
  mocap_publisher_->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "unconfigured" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn MocapUdpReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;
  m_udp_driver->receiver()->close();
  mocap_publisher_.reset();
  RCLCPP_DEBUG(get_logger(), "UDP receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "shutdown" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn MocapUdpReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Debugger function to output the mocap data.
void MocapUdpReceiverNode::debugMocapData2screen()
{
  // Output the parsed data
  FLIGHTSTACK_INFO("Time: ", mc_im.control_time);
  std::cout << "Position (x, y, z): " 
            << mc_im.x << ", " 
            << mc_im.y << ", " 
            << mc_im.z << std::endl;

  std::cout << "Quaternion (q0, q1, q2, q3): " 
            << mc_im.q0 << ", " 
            << mc_im.q1 << ", " 
            << mc_im.q2 << ", " 
            << mc_im.q3 << std::endl;

  std::cout << "Velocity (vx, vy, vz): " 
            << mc_im.vx << ", " 
            << mc_im.vy << ", " 
            << mc_im.vz << std::endl;

  std::cout << "Angular velocities (rollspeed, pitchspeed, yawspeed): " 
            << mc_im.rollspeed << ", " 
            << mc_im.pitchspeed << ", " 
            << mc_im.yawspeed << std::endl;  
}

/// \brief Callback for receiving a UDP datagram
void MocapUdpReceiverNode::receiver_callback(const std::vector<uint8_t> & buffer)
{
  // Convert the buffer to a string
  std::string buffer_str(buffer.begin(), buffer.end());

  // Check if the message format is valid
  if (buffer_str.size() < 2 || buffer_str[0] != 'R' || buffer_str[1] != ',') {
      FLIGHTSTACK_ERROR("Invalid message format!");
      return; // Exit the function or handle the error accordingly
  }


  // Parse the data from the string
  std::istringstream iss(buffer_str.substr(2)); // Skip "R," prefix
  char comma;
  iss >> mc_im.x >> comma >> mc_im.y >> comma >> mc_im.z >> comma                          // Read in the position 
      >> mc_im.q0 >> comma >> mc_im.q1 >> comma >> mc_im.q2 >> comma >> mc_im.q3 >> comma  // Read in the quaternion
      >> mc_im.vx >> comma >> mc_im.vy >> comma >> mc_im.vz >> comma                       // Read in the velocities
      >> mc_im.rollspeed >> comma >> mc_im.pitchspeed >> comma >> mc_im.yawspeed;          // Read in the angular speeds

  // Get the controller time
  mc_im.control_time = vehicle_ptr->get_controltime();

  // Debug Mocap parsed data to terminal
  // debugMocapData2screen();

  // Initilize the Mocap odometry message to publish to the pixhawk
  px4_msgs::msg::VehicleOdometry out;

  // \brief: https://github.com/hpaul360/mocap_to_px4/blob/main/mocap_to_px4/converter_member_function.py
  // Package the message   
  out.timestamp = 0;        // Timestamp is automatically set inside PX4
  out.timestamp_sample = 0; // Timestamp is automatically set inside PX4

  // Position frame is set to NED in the VICON code.
  out.position[0] = (float) mc_im.x;
  out.position[1] = (float) mc_im.y;
  out.position[2] = (float) mc_im.z;  

  // Orientation frame is set to NED in the VICON code.
  out.q[0] = (float) mc_im.q0;
  out.q[1] = (float) mc_im.q1;
  out.q[2] = (float) mc_im.q2;
  out.q[3] = (float) mc_im.q3;
  out.pose_frame = POSE_FRAME_NED;

  // Velocity frame is set to NED in the VICON code. 
  // The velocity calculations in the VICON code is not great, once updated - fuse.
  out.velocity[0] = (float) mc_im.vx;
  out.velocity[1] = (float) mc_im.vy;
  out.velocity[2] = (float) mc_im.vz;
  out.velocity_frame = VELOCITY_FRAME_NED;

  // Angular speed is set to 0. We need to pass the velocites to EKF2 in the FRD Body-Fixed
  // Frame. We do not have the data in the VICON code therefore we set it to 0.
  out.angular_velocity[0] = (float) 0.0;
  out.angular_velocity[1] = (float) 0.0;
  out.angular_velocity[2] = (float) 0.0;

  // Other Parameters
  out.position_variance = {0,0,0};
  out.orientation_variance = {0,0,0};
  out.velocity_variance = {0,0,0};

  // Publish the message
  mocap_publisher_->publish(out);

  // Log the data
  logLogData();
}


} // namespace _udp_driver_    
} // namespace _drivers_

