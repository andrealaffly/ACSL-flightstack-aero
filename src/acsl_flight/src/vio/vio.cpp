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
 * Referenced:  https://github.com/IntelRealSense/librealsense/blob/v2.53.1
 *              Copyright 2017 Intel Corporation.
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
 * File:        vio.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        September 05, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Node definition for Intel Realsense T265 tracking camera as a lifecycle node.
 *              Writes messages to pixhawk mocap_odometry topic for EKF2 fusion.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "vio.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

// Namespace for the Intel Realsense T265 Cameras
namespace _t265_
{

// Written for tracking with stereo camera images and feature recognition
namespace _vio_
{
// Implementing virtual functions from Blackbox
void VioT265Node::logInitHeaders() {
	std::ostringstream oss;

	oss << ", "
			<< "Controller time [s], "
			<< "VIO Position x [m], "
			<< "VIO Position y [m], "
			<< "VIO Position z [m], "
			<< "VIO velocity x [m/s], "
			<< "VIO velocity y [m/s], "
			<< "VIO velocity z [m/s], "
			<< "VIO acceleration x [m/s^2], "
			<< "VIO acceleration y [m/s^2], "
			<< "VIO acceleration z [m/s^2], "
			<< "VIO Quaternion q0 [-], "
			<< "VIO Quaternion q1 [-], "
			<< "VIO Quaternion q2 [-], "
			<< "VIO Quaternion q3 [-], "
			<< "VIO Angular velocity x [rad/s], "
			<< "VIO Angular velocity y [rad/s], "
			<< "VIO Angular velocity z [rad/s], "
			<< "VIO Angular acceleration x [rad/s], "
			<< "VIO Angular acceleration y [rad/s], "
			<< "VIO Angular acceleration z [rad/s], "
			<< "VIO pose confidence level [-], "
			<< "VIO map confidence level [-], ";

	BOOST_LOG(logger_viodata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool VioT265Node::logInitLogging() {
	try {
		// Generate the log file name		
		std::stringstream log_ss;
		log_ss << flight_run_log_directory << "/vio_log" << ".log";
		std::string vio_log_filename = log_ss.str();

		// Add the "Tag" attribute with a constant value of "LogDataTag" to the logger
		logger_viodata.add_attribute("Tag", attrs::constant< std::string >("VioTag"));

		// Define a synchronous sink wiht a text ostream backend
		typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
		boost::shared_ptr<text_sink> vio_sink = boost::make_shared<text_sink>();

		// Add a stream to the backend (in this case, a file stream)
		vio_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(vio_log_filename));

		// Set the formatter for the sink
		vio_sink->set_formatter(
			expr::stream
			<< "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
          << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
          << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
          << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
          << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
          << expr::smessage // Log message
        );

		// Add the sink to the logging core
		logging::core::get()->add_sink(vio_sink);

		// Set a filter for the sink
		vio_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "VioTag");

		logging::add_common_attributes(); // Add attributes like timestamp

		// Initilaize the logging headers
		logInitHeaders();

		// Return true if all successful
		return true;
	} catch (const std::filesystem::filesystem_error& e) {
			FLIGHTSTACK_ERROR("Filesystem error:", e.what());
	} catch (const std::exception& e) {
			FLIGHTSTACK_ERROR("Exeption: ", e.what());
	}

	return false; // IF there are errors
}

// Implementing virtual function from Blackbox
void VioT265Node::logLogData() {
	// Log the data
	std::ostringstream oss;

	oss << ", "
			<< vio_im.control_time << ", "
			<< vio_im.x << ", "
			<< vio_im.y << ", "
			<< vio_im.z << ", "
			<< vio_im.vx << ", "
			<< vio_im.vy << ", "
			<< vio_im.vz << ", "
			<< vio_im.ax << ", "
			<< vio_im.ay << ", "
			<< vio_im.az << ", "
			<< vio_im.q0 << ", "
			<< vio_im.q1 << ", "
			<< vio_im.q2 << ", "
			<< vio_im.q3 << ", "
			<< vio_im.rollspeed << ", "
			<< vio_im.pitchspeed << ", "
			<< vio_im.yawspeed << ", "
			<< vio_im.rollacceleration << ", "
			<< vio_im.pitchacceleration << ", "
			<< vio_im.yawacceleration << ", "
			<< vio_im.tracker_conf << ", "
			<< vio_im.mapper_conf << ", ";

	BOOST_LOG(logger_viodata) << oss.str();
}

/// \brief Constructor which accepts pointers for the vio node
/// \param[in] pointer to vehicle states in flight_bridge
/// \param[in] string to the global flight run directory
VioT265Node::VioT265Node(vehicle_states* d, const std::string & global_log_dir)
: lc::LifecycleNode("vio_node"),
	vehicle_ptr(d),
	vio_im{},			// Aggregate initialization sets all members to zero
	flight_run_log_directory(global_log_dir)
{
	// Initialize the logging
	if(logInitLogging()) {RCLCPP_INFO(this->get_logger(), "VIO LOGGER CREATED");};	
}

/// \brief Get the parameters for the serial number of the T265 in use
char* VioT265Node::return_serial()
{

	// Assign the serial number to an internal member
	t265_serial = SERIAL_NUMBER;

	// Reutrn the serial number
	return const_cast<char*>(t265_serial.c_str());
}

/// \brief Destructor - management at quit time
VioT265Node::~VioT265Node()
{
	// Try shutting down the stream
	try 
	{
			pipeline.stop();				// Stop the pipline and release the resources

			// Inform that the device piepline is setup
			RCLCPP_INFO(this->get_logger(), "STOPPED T265 PIPELINE - %s", return_serial());
	}
	catch (const rs2::error& e)
	{
			// Catch RealSense-specific errors
			RCLCPP_ERROR(this->get_logger(), "Failed to configure T265: %s", e.what());
			RCLCPP_ERROR(this->get_logger(), "Error occurred in function: %s", e.get_failed_function().c_str());
			RCLCPP_ERROR(this->get_logger(), "Error occurred with arguments: %s", e.get_failed_args().c_str());
	}
	catch (const std::exception& e)
	{
			// Catch other exceptions
			RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", e.what());
	}
}

/// \brief Callback from transition to "configuring" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn VioT265Node::on_configure(const lc::State & state)
{
	(void)state;

	// Create a Sensor Data QoS profile - 200Hz is the data frequency of VIO. Message every 5ms. 
	// Therefore, we want to discard old messages in the queue.
	rclcpp::QoS qos_settings = rclcpp::SensorDataQoS();  // Use default Sensor Data QoS profile
	qos_settings.keep_last(1);  												 // Keep only the last message

	// Create publisher for vio to the pixhawk visual odometry topic
	vio_publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
				"/fmu/in/vehicle_visual_odometry", qos_settings);
	
	// Setup the pipeline for the T265 ----------------------------------
	rs2::config cfg_t265;																				// Setup the configuration for the camera

	// Try configuring the camera
	try
	{
		cfg_t265.enable_device(return_serial());										// Enable the device
		cfg_t265.enable_stream(RS2_STREAM_POSE,RS2_FORMAT_6DOF);		// Enable the streaming of data
		cfg_t265.disable_stream(RS2_STREAM_FISHEYE, CAMERA_ONE);		// Disable the fisheye stream	1
		cfg_t265.disable_stream(RS2_STREAM_FISHEYE, CAMERA_TWO);		// Disable the fisheye stream 2

		RCLCPP_INFO(this->get_logger(), "CONFIGURED DEVICE - %s", return_serial());
	}
	catch (const rs2::error& e)
	{
			// Catch RealSense-specific errors
			RCLCPP_ERROR(this->get_logger(), "Failed to configure T265: %s", e.what());
			RCLCPP_ERROR(this->get_logger(), "Error occurred in function: %s", e.get_failed_function().c_str());
			RCLCPP_ERROR(this->get_logger(), "Error occurred with arguments: %s", e.get_failed_args().c_str());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}
	catch (const std::exception& e)
	{
			// Catch other exceptions
			RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", e.what());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}

	// Try setting up the callback lamba
	try
	{
		auto callback_set = set_callback();

		if (callback_set) {
    // If the callback was successfully set, log the success message
				RCLCPP_INFO(this->get_logger(), "CALLBACK FOR T265 SETUP");
		} else {
				// If the callback setup failed, log an error and throw an exception
				RCLCPP_ERROR(this->get_logger(), "CALLBACK FOR T265 FAILED");
				throw std::runtime_error("Callback setup failed.");
		}
	}
	catch (const rs2::error& e)
	{
			// Catch RealSense-specific errors
			RCLCPP_ERROR(this->get_logger(), "Failed to configure T265: %s", e.what());
			RCLCPP_ERROR(this->get_logger(), "Error occurred in function: %s", e.get_failed_function().c_str());
			RCLCPP_ERROR(this->get_logger(), "Error occurred with arguments: %s", e.get_failed_args().c_str());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}
	catch (const std::exception& e)
	{
			// Catch other exceptions
			RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", e.what());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}
	
	
	// Try strating the pipeline
	try
	{
		profiles = pipeline.start(cfg_t265, callback);														// Start the pipeline

		// Inform that the device piepline is setup
		RCLCPP_INFO(this->get_logger(), "STARTED T265 PIPELINE - %s", return_serial());

		// Get the streams
		for (auto p : profiles.get_streams()) 
		{
			// Inform the streams that are available in the pipeline
			RCLCPP_INFO(this->get_logger(), "STREAMING T265 - [%s]", p.stream_name().c_str());
		}

		// Return success for the node
		return LNI::CallbackReturn::SUCCESS;
	}
	catch (const rs2::error& e)
	{
			// Catch RealSense-specific errors
			RCLCPP_ERROR(this->get_logger(), "Failed to configure T265: %s", e.what());
			RCLCPP_ERROR(this->get_logger(), "Error occurred in function: %s", e.get_failed_function().c_str());
			RCLCPP_ERROR(this->get_logger(), "Error occurred with arguments: %s", e.get_failed_args().c_str());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}
	catch (const std::exception& e)
	{
			// Catch other exceptions
			RCLCPP_ERROR(this->get_logger(), "An unexpected error occurred: %s", e.what());

			// Return error
			return LNI::CallbackReturn::ERROR;
	}
}

/// \brief Callback from transition to "activating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn VioT265Node::on_activate(const lc::State & state)
{
  (void)state;
  vio_publisher_->on_activate();
  RCLCPP_DEBUG(this->get_logger(), "VIO node activated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "deactivating" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn VioT265Node::on_deactivate(const lc::State & state)
{
  (void)state;
  vio_publisher_->on_deactivate();
  RCLCPP_DEBUG(this->get_logger(), "VIO node deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}


/// \brief Callback from transition to "unconfigured" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn VioT265Node::on_cleanup(const lc::State & state)
{
  (void)state;	
  vio_publisher_.reset();
  RCLCPP_DEBUG(this->get_logger(), "VIO node cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

/// \brief Callback from transition to "shutdown" state.
/// \param[in] state The current state that the node is in.
LNI::CallbackReturn VioT265Node::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "VIO node shutting down.");
	return LNI::CallbackReturn::SUCCESS;
}

/// \brief Debugger function to output the mocap data.
void VioT265Node::debugVioData2screen()
{
	// Output any parsed data
}

/// \brief Function to setup the callback for the frames from the T265
bool VioT265Node::set_callback()
{
    // Try setting up the callback for the frames
    try
    {
			// Lambda Function implementation
			callback = [&](const rs2::frame& frame)
			{   
					// Check if the frame is a pose frame
					if (frame.is<rs2::pose_frame>()) {
							// Pass it onto the process and publish function
							process_and_publish(frame.as<rs2::pose_frame>());
					}
			};

			return true;  // Successful callback setup
    }
    catch(const std::exception& e)
    {
        FLIGHTSTACK_ERROR("Error settup up callback:", e.what());
        return false;  // Return false if an exception occurs
    }
}

/// \brief Funtion to process and publish the pose data for fustion to the Pixhawk
void VioT265Node::process_and_publish(const rs2::pose_frame & pose)
{
	rs2_pose data = pose.get_pose_data();

	// Process the data
	vio_im.control_time = vehicle_ptr->get_controltime();
	
	// ------------------------  Translational
	// Translational data from the T265 is not rotated accourding to our mounting orientation. Therefore go from
	// T265 frame to C, the frame analogous to the body frame (NED)
	position_C << -data.translation.z, 
								 data.translation.x,
								-data.translation.y;
	
	// Translational data from the T265 is not rotated accourding to our mounting orientation. Therefore go from
	// T265 frame to C, the frame analogous to the body frame (NED)
	velocity_C << -data.velocity.z, 
								 data.velocity.x,
								-data.velocity.y;

	// Translational data from the T265 is not rotated accourding to our mounting orientation. Therefore go from
	// T265 frame to C, the frame analogous to the body frame (NED)
	acceleration_C << -data.acceleration.z, 
								 		 data.acceleration.x,
										-data.acceleration.y;
	
	// Assign the translational states.
	vio_im.x = position_C(0);
	vio_im.y = position_C(1);
	vio_im.z = position_C(2);

	vio_im.vx = velocity_C(0);
	vio_im.vy = velocity_C(1);
	vio_im.vz = velocity_C(2);

	vio_im.ax = acceleration_C(0);
	vio_im.ay = acceleration_C(1);
	vio_im.az = acceleration_C(2);
	
	// ------------------------  Rotational
	// Transform the quaternion from the camera frame (VR convention/pose_frame) to a frame c which is NED but at 
	// a specified orientation w.r.t the body frame (NED).
	t265_quaternion  = Quaternionf( data.rotation.w,
														 		 -data.rotation.z,
									 							  data.rotation.x,
																 -data.rotation.y);
	
	// Transform the orientation from the c frame to the frame C 
	// ---C frame is the analogous to the body frame (NED) and while the body frame is centered at the center of mass,
	// 		this frame is shifted in x,y,z and positioned at center of the pose frame (VR frame) of the camera. 
	transformed_quaternion = (t265_quaternion*q_J_c_inv);

	// Final quaternion data in frame C w.r.t inertial frame I.
	vio_im.q0 = transformed_quaternion.w();
	vio_im.q1 = transformed_quaternion.x();
	vio_im.q2 = transformed_quaternion.y();
	vio_im.q3 = transformed_quaternion.z();
	

	// Cache the angular velocity data with respect to the c frame that is mounted at a specified orientation w.r.t 
	// the body frame (NED)
	angular_velocity_c << -data.angular_velocity.z,
								 				 data.angular_velocity.x,
												-data.angular_velocity.y;

	// Cache the angular acceleration data with respect to the c frame that is mounted at a specified orientation w.r.t 
	// the body frame (NED)
	angular_acceleration_c << -data.angular_acceleration.z, 
								 					   data.angular_acceleration.x,
														-data.angular_acceleration.y;
	
	// Rotate the angular velocity in frame c to frame C
	// ---C frame is the analogous to the body frame (NED) and while the body frame is centered at the center of mass,
	// 		this frame is shifted in x,y,z and positioned at center of the pose frame (VR frame) of the camera. 
	angular_velocity_C = R_c_C*angular_velocity_c;

	// Rotate the angular acceleration in frame c to frame C
	// ---C frame is the analogous to the body frame (NED) and while the body frame is centered at the center of mass,
	// 		this frame is shifted in x,y,z and positioned at center of the pose frame (VR frame) of the camera. 
	angular_acceleration_C = R_c_C*angular_acceleration_c;

	// Assign the rotational dynamics
	vio_im.rollspeed = angular_velocity_C(0);
	vio_im.pitchspeed = angular_velocity_C(1);
	vio_im.yawspeed = angular_velocity_C(2);

	vio_im.rollacceleration = angular_velocity_C(0);
	vio_im.pitchacceleration = angular_velocity_C(1);
	vio_im.yawacceleration = angular_velocity_C(2);

	// Confidence Data
	vio_im.tracker_conf = data.tracker_confidence;
	vio_im.mapper_conf = data.mapper_confidence;

	// -----------------------> Publishing data to pixhawk
	// Initialize the VIO odometry message to publish to the pixhawk
	px4_msgs::msg::VehicleOdometry out;

	// \brief: https://github.com/hpaul360/mocap_to_px4/blob/main/mocap_to_px4/converter_member_function.py
	// Pacakge the data to be sent out 
	out.timestamp = 0;        // Timestamp is automatically set inside PX4
  out.timestamp_sample = 0; // Timestamp is automatically set inside PX4

	// Position frame is set to NED in the code block above 
	out.position[0] = (float) vio_im.x;
	out.position[1] = (float) vio_im.y;
	out.position[2] = (float) vio_im.z;

	// Orientation frame is set to NED in the code block above
	out.q[0] = (float) vio_im.q0;
	out.q[1] = (float) vio_im.q1;
	out.q[2] = (float) vio_im.q2;
	out.q[3] = (float) vio_im.q3;
	out.pose_frame = POSE_FRAME_NED;

	// Velocity frtame is set to NED in the code block above
	// The velocity calculation in VIO is pretty good, so it will be fused.
	out.velocity[0] = (float) vio_im.vx;
	out.velocity[1] = (float) vio_im.vy;
	out.velocity[2] = (float) vio_im.vz;
	out.velocity_frame = VELOCITY_FRAME_NED;

	// Angular speed is passed on in VIO. We need to pass the velocites to EKF2 in the FRD Body-Fixed
  // Frame. We have the code with VIO unlike vicon so we fuse the data.
  out.angular_velocity[0] = (float) vio_im.rollspeed;
  out.angular_velocity[1] = (float) vio_im.pitchspeed;
  out.angular_velocity[2] = (float) vio_im.yawspeed;

	// Other Parameters
	out.position_variance = {0,0,0};
  out.orientation_variance = {0,0,0};
  out.velocity_variance = {0,0,0};

	// Publish the message
	vio_publisher_->publish(out);

	// Log the data
  logLogData();
}

}	// namespace _vio_
}	// namespace _t265_