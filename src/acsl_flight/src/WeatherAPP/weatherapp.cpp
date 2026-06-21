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
 * File:        weatherapp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        March 2, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Node definition for UDP socket as a lifecycle node.
 *              Logs messages for weather data
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "weatherapp.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace _drivers_{
namespace _udp_driver_{

// Implementing virtual functions from Blackbox
void WeatherUdpReceiverNode::logInitHeaders() {
  std::ostringstream oss;

  oss << ", "
      << "Controller time [s], "
      << "Pressure [Pa], "
      << "Temperature [C], "
      << "Wind Speed [m/s], "
      << "10 Min Wind Gust [m/s], "
      << "Wind Vane [rad], "
      << "Wind Speed U [m/s], "
      << "Wind Speed V [m/s], ";

  BOOST_LOG(logger_weatherdata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool WeatherUdpReceiverNode::logInitLogging() {
  try {
      // Generate the log file name
      std::stringstream log_ss;
      log_ss << flight_run_log_directory << "/weather_log" << ".log";
      std::string weather_log_filename = log_ss.str();

      // Add the "Tag" attribute wiht a constant value of "LogDataTag" to the logger
      logger_weatherdata.add_attribute("Tag", attrs::constant< std::string >("WeatherTag"));

      // Define a synchronous sink with a text ostream backend
      typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
      boost::shared_ptr<text_sink> weather_sink = boost::make_shared<text_sink>();

      // Add a stream to the backend (in this case, a file stream)
      weather_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(weather_log_filename));

      // Set the formatter for the sink
      weather_sink->set_formatter(
        expr::stream
        << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
        << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
        << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
        << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
        << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
        << expr::smessage // Log message
      );

      // Add the sink to the logging core
      logging::core::get()->add_sink(weather_sink);

      // Set a filter for the sink
      weather_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "WeatherTag");

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
void WeatherUdpReceiverNode::logLogData() {
  // Log the data
  std::ostringstream oss;

  oss << ", "
      << w_im.control_time << ", "
      << w_im.pressure << ", "
      << w_im.temperature << ", "
      << w_im.windspeed << ", "
      << w_im.gustspeed << ", "
      << w_im.windvane << ", "
      << w_im.uwind << ", "
      << w_im.vwind << ", ";

  BOOST_LOG(logger_weatherdata) << oss.str();
}

/// \brief Constructor which accepts IoContext
/// \param[in] ctx A shared IoContext
/// \param[in] pointer to vehicle states in flight_bridge    
/// \param[in] string to the global flight run directory
WeatherUdpReceiverNode::WeatherUdpReceiverNode(
  const IoContext & ctx, vehicle_states* d, const std::string & global_log_dir)
: lc::LifecycleNode("weather_udp_receiver_node"),
  w_udp_driver{new UdpDriver(ctx)},
  vehicle_ptr(d),
  flight_run_log_directory(global_log_dir)
{
  // Initialize the logging
  if(logInitLogging()) {FLIGHTSTACK_INFO("Weather Logger Created");};

  // Get the parameters
  get_params();  
}

/// \brief Get the parameters for the ip and port to ping
void WeatherUdpReceiverNode::get_params()
{
  w_ip = IP_IN_USE;     

  w_port = WEATHER_PORT;

  RCLCPP_INFO(get_logger(), "ip: %s", w_ip.c_str());
  RCLCPP_INFO(get_logger(), "port: %i", w_port);
}

/// \brief Destructor - required to manage owned IoContext
WeatherUdpReceiverNode::~WeatherUdpReceiverNode()
{
  if (w_owned_ctx) {
    w_owned_ctx->waitForExit();
    FLIGHTSTACK_INFO("Closed Mocap Thread");
  }
}

/// \brief Callback from transition to "configuring" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn WeatherUdpReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  // Check if w_udp_driver is not null
  if (w_udp_driver) {
    RCLCPP_INFO(get_logger(), "UDP driver is initialized.");
  }

  try {
    
    w_udp_driver->init_receiver(w_ip, w_port);
    w_udp_driver->receiver()->open();
    w_udp_driver->receiver()->bind();
    w_udp_driver->receiver()->asyncReceive(
      std::bind(&WeatherUdpReceiverNode::receiver_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP receiver: %s:%i - %s",
      w_ip.c_str(), w_port, ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(get_logger(), "UDP receiver successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "activating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn WeatherUdpReceiverNode::on_activate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "deactivating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn WeatherUdpReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "unconfigured" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn WeatherUdpReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;
  w_udp_driver->receiver()->close();
  RCLCPP_DEBUG(get_logger(), "UDP receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "shutdown" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn WeatherUdpReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Debugger function to output the weather data.
void WeatherUdpReceiverNode::debugWeatherData2screen()
{
  // Output the parsed data
  FLIGHTSTACK_INFO("Time: ", w_im.control_time);
  std::cout <<  "Pressure: " << w_im.pressure << std::endl;
  std::cout << "Temperature: " << w_im.temperature << std::endl;
  std::cout << "Wind Speed: " << w_im.windspeed << std::endl;
  std::cout << "10 Min Gust Speed: " << w_im.gustspeed << std::endl;
  std::cout << "Wind Vane: " << w_im.windvane << std::endl;
  std::cout << "U Com Speed: " << w_im.uwind << std::endl;
  std::cout << "V Com Speed: " << w_im.vwind << std::endl;
}

/// \brief Callback for receiving a UDP datagram
void WeatherUdpReceiverNode::receiver_callback(const std::vector<uint8_t> & buffer)
{
  // Convert the buffer to a string
  std::string buffer_str(buffer.begin(), buffer.end());

  // Check if the message format is valid and pick accordingly
  if (buffer_str.size() < 2 || buffer_str[0] != 'W' || buffer_str[1] != ',') {
      FLIGHTSTACK_ERROR("Invalid message format!");
      return; // Exit the function or handle the error accordingly
  }


  // Parse the data from the string
  std::istringstream iss(buffer_str.substr(2)); // Skip "W," prefix
  char comma;
  iss >> w_im.pressure >> comma >> w_im.temperature >> comma            // Read in the pressure and temperature.
      >> w_im.windspeed >> comma >> w_im.gustspeed >> comma             // Read in the windspeed and the 10 minture gust speed.
      >> w_im.windvane >> comma >> w_im.uwind >> comma >> w_im.vwind;   // Read in the wind vane and the components of windspeed in x-y inertial plane.
    
  // Get the controller time
  w_im.control_time = vehicle_ptr->get_controltime();

  // Debug weather parsed data to terminal
  // debugWeatherData2screen();

  // Log the data
  logLogData();

}

} // namespace _udp_driver_
} // namespace _drivers_