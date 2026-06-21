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
 * File:        vio.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        September 05, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class declaration for Intel Realsense T265 tracking camera as a lifecycle node.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef VIO_HPP_
#define VIO_HPP_

#include "global_config.hpp"                      // Include this for creating the logging file.
#include "librealsense2/rs.hpp"										// Include for the librealsense2 sdk.
#include "px4_defines.hpp"                        // Include for the PX4_define terms in a central location.
#include "global_helpers.hpp" 										// Include for the flightstack global functions
#include "helper_functions.hpp" 									// Include for rotation matrix support

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
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

// Convert Degrees to Radians
inline constexpr double DEG2RAD = (M_PI/180);

// -> Serial Number for the Intel Realsense T265 Camera
inline constexpr const char* SERIAL_NUMBER = "201222113079";

// -> Camera defines
inline constexpr int CAMERA_ONE = 1;				// Define for the 1st fisheye lens
inline constexpr int CAMERA_TWO = 2;				// Define for the 2nd fisheye lens

//-> How is the camera mounted w.r.t the body frame? - Please specify the angles in RADIANS.
// Keep in mind that the rotation sequence is 3-2-1 and the body frame is NED!!.
inline constexpr float ROLL = 0.0;
inline constexpr float PITCH = 30.0*DEG2RAD;
inline constexpr float YAW = 0.0;

// Quaternion that describes the orientation of the camera wrt to the body frame (NED)
inline const Eigen::Quaternionf q_J_c = Eigen::Quaternionf(  cos(YAW/2.0),           0.0,            0.0, sin(YAW/2.0))*
																				Eigen::Quaternionf(cos(PITCH/2.0),           0.0, sin(PITCH/2.0),          0.0)*
																				Eigen::Quaternionf( cos(ROLL/2.0), sin(ROLL/2.0),           0.0 ,          0.0);

// Cache the inverse of the quaternion that descrives how the camera is mounted w.r.t to the body frame (NED)
inline const Eigen::Quaternionf q_J_c_inv = q_J_c.inverse();

// Cache the rotation matrix that rotates the translational data in the camera frame to the NED inertial frame
inline const Eigen::Matrix3f R_c_C = rotationMatrix321GlobalToLocal(ROLL, PITCH, YAW);

// Namespace for the Intel Realsense T265 Cameras
namespace _t265_
{

// Struct for vio states that come over the USB 3.0 Link
struct vio_states
{
	double control_time;						// Common synced controller time from control thread
	double x;												// VIO position in x
	double y;												// VIO position in y
	double z;												// VIO position in z
	double vx;											// VIO velocity in x
	double vy;											// VIO velocity in y
	double vz;											// VIO velocity in z
	double ax;											// VIO acceleration in x
	double ay;											// VIO acceleration in y
	double az;											// VIO acceleration in z
	double q0;											// VIO rotation quaternion in qw
	double q1;											// VIO rotation quaternion in qx
	double q2;											// VIO rotation quaternion in qy
	double q3;											// VIO rotation quaternion in qz
	double rollspeed;								// VIO angular velocity about body x
	double pitchspeed;							// VIO angular velocity about body y
	double yawspeed;								// VIO angular velocity about body z
	double rollacceleration;				// VIO angular acceleration about body x
	double pitchacceleration;				// VIO angular acceleration about body y
	double yawacceleration;					// VIO angular acceleration about body z
	unsigned int tracker_conf; 			// Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                          
  unsigned int mapper_conf;  			// Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High
};

// Written for tracking with stereo camera images and feature recognition
namespace _vio_
{

/// \brief VioT265Node class which can recieve position tracking from the Realsense T265
class VioT265Node final
	:	public lc::LifecycleNode, public fl::blackbox
{
	public:
		/// \brief Constructor 
		/// \param[in] pointer to vehicle states in flight_bridge
		/// \param[in] string to the global flight run directory
		VioT265Node(vehicle_states* d, const std::string & global_log_dir);

		/// \brief Destructor - management at quit time
		~VioT265Node();

		/// \brief Callback from transition to "configuring" state.
		/// \param[in] state The current state that the node is in.
		LNI::CallbackReturn on_configure(const lc::State & state) override;

		/// \brief Callback from transition to "activating" state.
		/// \param[in] state The current state that the node is in.
		LNI::CallbackReturn on_activate(const lc::State & state) override;

		/// \brief Callback from transistion to "deactivating" state.
		/// \param[in] state The current state that the node is in.
		LNI::CallbackReturn on_deactivate(const lc::State & state) override;

    /// \brief Callback from transition to "unconfigured" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_cleanup(const lc::State & state) override;

    /// \brief Callback from transition to "shutdown" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_shutdown(const lc::State & state) override;

		/// \brief Returns the serial number of the Intel Realsense T265 camera in use
		char* return_serial();

	private:
		/// Serial number of the Intel Realsens t265 camera
		std::string t265_serial{};

		/// Pipelines for the Intel Realsense T265 camera
		rs2::pipeline pipeline;

		/// Obeject for the profiles
		rs2::pipeline_profile profiles;

		/// Instance of the vehicle_ptr to point to the vehicle state object in 
		/// flight_bridge.cpp
		vehicle_states* vehicle_ptr;

		/// Publisher for publishing the vio message
		lc::LifecyclePublisher<px4_msgs::msg::VehicleOdometry>::SharedPtr vio_publisher_;

		/// Declare variables to store parsed data and declare it as internal members "_im"
		vio_states vio_im;

		/// \brief Function to setup the callback for the frames from the T265
		bool set_callback();

		/// \brief Callback for receiving a state
		std::function<void(const rs2::frame&)> callback;

		/// \brief Funtion to process and publish the pose data for fustion to the Pixhawk
		void process_and_publish(const rs2::pose_frame & pose);

		/// \brief Debugger function to output the vio data.
		void debugVioData2screen();

		/// \brief Implementing virtual functions from blackbox
		void logInitHeaders();
		bool logInitLogging();
		void logLogData();

		// Quaternions to transform rotation
		Vector3f position_C;
		Vector3f velocity_C;
		Vector3f acceleration_C;
		Vector3f angular_velocity_c;
		Vector3f angular_velocity_C;
		Vector3f angular_acceleration_c;
		Vector3f angular_acceleration_C;
		Quaternionf t265_quaternion;
		Quaternionf transformed_quaternion;

		/// \brief Directory for logging mocap data
    std::string flight_run_log_directory;

};

}	// namespace _vio_
}	// namespace _t265_

#endif // VIO_HPP_