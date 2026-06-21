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

/***********************************************************************************************************************
 * File:        read_pix.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class definition for reading pixhawk data.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "read_pix.hpp"

namespace _read_pix_
{
    /***********************************************************************************************/
    /*                                  NODE CONSTRUCTOR                                           */
    /***********************************************************************************************/
    read_odometryNode::read_odometryNode(vehicle_states* d, flight_params* p)
    : rclcpp::Node("read_odometry_node"), vehicle_ptr(d), flight_params_ptr(p)
    {
        using std::placeholders::_1;

        // Quality of Service initialization for subscription of odometry data.
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        /// Create the odometry subscription
        odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, 
            std::bind(&read_odometryNode::odometry_callback_,this, 
            _1));

        // Check if we are running RTK GPS. IF so do not want to zero the yaw data.
        if (!flight_params_ptr->mocap && !flight_params_ptr->vio) 
        { 
            stateMachine.zeroAcq = false;       // Correction data has not been acquired yet
            stateMachine.zeroPos = true;        // Zero the position
            stateMachine.zeroYaw = false;       // Do not zero the yaw
        }
        else
        {
            stateMachine.zeroAcq = false;       // Correction data has not been acquired yet
            stateMachine.zeroPos = true;        // Zero the position
            stateMachine.zeroYaw = true;        // Zero the yaw
        }

    }


    /***********************************************************************************************/
    /*                                  HELPER FUNCTIONS                                           */
    /***********************************************************************************************/
    // Function for calculating the euler angles
    // Computing Euler Angles from the unit quaternion following a 3-2-1 rotation sequence
    //   The Euler Angles are ordered as: roll, pitch, yaw
    //   For reference:
    //     - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Euler.hpp
    //     - https://github.com/PX4/PX4-Matrix/blob/master/matrix/Dcm.hpp
    std::array<double, 3> read_odometryNode::quaternionToEulerAnglesRPY(double q0, double q1, double q2, double q3)
    {
        std::array<double, 3> output;
        
        // Compute elements of Direction Cosine Matrix (DCM) from quaternion
        double a = q0;
        double b = q1;
        double c = q2;
        double d = q3;
        double aa = a * a;
        double ab = a * b;
        double ac = a * c;
        double ad = a * d;
        double bb = b * b;
        double bc = b * c;
        double bd = b * d;
        double cc = c * c;
        double cd = c * d;
        double dd = d * d;

        // Compute DCM elements
        double dcm00 = aa + bb - cc - dd;
        // double dcm01 = 2 * (bc - ad); // not needed by the algorithm
        double dcm02 = 2 * (ac + bd);
        double dcm10 = 2 * (bc + ad);
        // double dcm11 = aa - bb + cc - dd; // not needed by the algorithm
        double dcm12 = 2 * (cd - ab);
        double dcm20 = 2 * (bd - ac);
        double dcm21 = 2 * (ab + cd);
        double dcm22 = aa - bb - cc + dd;

        // Clamp the value of dcm20 to the range [-1, 1] to avoid domain errors with asinf
        if (dcm20 < -1.0) dcm20 = -1.0;
        if (dcm20 > 1.0) dcm20 = 1.0;

        // Compute the Euler angles from DCM
        output[1] = asin(-dcm20); // Pitch

        // Handle gimbal lock cases
        if (fabs(output[1] - M_PI / 2) < 1.0e-3) {
            output[0] = 0.0; // Roll
            output[2] = atan2(dcm12, dcm02); // Yaw
        } else if (fabs(output[1] + M_PI / 2) < 1.0e-3) {
            output[0] = 0.0; // Roll
            output[2] = atan2(-dcm12, -dcm02); // Yaw
        } else {
            output[0] = atan2(dcm21, dcm22); // Roll
            output[2] = atan2(dcm10, dcm00); // Yaw
        }


        return output;
        
    }

    /***********************************************************************************************/
    /*                                  CALLBACK FUNCTION                                          */
    /***********************************************************************************************/
    void read_odometryNode::odometry_callback_(const px4_msgs::msg::VehicleOdometry::UniquePtr msg)
    {
        // NOTE:
        // If using mocap or VIO, we perform zeroing of the yaw (heading).
        // However, if the system is using RTK for inertial positioning, 
        // heading and x-y position are not decoupled. Zeroing the heading
        // in the flight stack in this case can result in crashes.

        // Cache bitmask for the different state machine configurations | bit 1 - Correction data acquired | bit 2 - Zero Pos | bit 3 - Zero Yaw
        ReadMachineState bitmask = stateMachine.getState();

        // STATE 1 : Collecting correction data for both position and yaw -------------------------------------------------------------------------
        if ( bitmask == ReadMachineState::ACQUIRE_POS_YAW_CORRECTION )
        {
            // Store the intial position values
            stateMachine.corrections.x = msg->position[0];
            stateMachine.corrections.y = msg->position[1];
            stateMachine.corrections.z = msg->position[2];

            // Compute the initial orientation
            stateMachine.corrections.EulerRPY = quaternionToEulerAnglesRPY(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

            // Cache the intial orientation
            stateMachine.corrections.roll = stateMachine.corrections.EulerRPY[0];
            stateMachine.corrections.pitch = stateMachine.corrections.EulerRPY[1];
            stateMachine.corrections.yaw = stateMachine.corrections.EulerRPY[2];

            // Cache the quaternion offset in yaw
            stateMachine.corrections.yaw_quaternion = Quaterniond(cos(stateMachine.corrections.yaw/2.0),
                                                                  0.0,
                                                                  0.0,
                                                                  sin(stateMachine.corrections.yaw/2.0));

            // Load in the quaternion reading from the pixhawk
            quaternion_pix = Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

            // Zero the quaternion in yaw by post-multiplying with inverse of yaw quaternion
            zeroed_quaternion = quaternion_pix*stateMachine.corrections.yaw_quaternion.inverse();

            // Convert the zerod quaternion to euler
            EulerRPY = quaternionToEulerAnglesRPY(zeroed_quaternion.w(),
                                                  zeroed_quaternion.x(),
                                                  zeroed_quaternion.y(),
                                                  zeroed_quaternion.z());

            // ------------------------------------------------------------- > Set Values < --
            // Set the initial values as zero in the vehicle class where needed.
            vehicle_ptr->set_pixhawktime(msg->timestamp);
            vehicle_ptr->set_x(0.0);                                      // We zero in the position in x 
            vehicle_ptr->set_y(0.0);                                      // We zero in the position in y
            vehicle_ptr->set_z(0.0);                                      // We zero in the position in z
            vehicle_ptr->set_vx(0.0);                                     // We zero in the velocity in x
            vehicle_ptr->set_vy(0.0);                                     // We zero in the velocity in y
            vehicle_ptr->set_vz(0.0);                                     // We zero in the velocity in z
            vehicle_ptr->set_q0(zeroed_quaternion.w());                   // We zero in the yaw of the quaternion
            vehicle_ptr->set_q1(zeroed_quaternion.x());                   // We zero in the yaw of the quaternion
            vehicle_ptr->set_q2(zeroed_quaternion.y());                   // We zero in the yaw of the quaternion
            vehicle_ptr->set_q3(zeroed_quaternion.z());                   // We zero in the yaw of the quaternion
            vehicle_ptr->set_rollspeed(0.0);                              // We zero in the rollspeed
            vehicle_ptr->set_pitchspeed(0.0);                             // We zero in the pitchspeed
            vehicle_ptr->set_yawspeed(0.0);                               // We zero in the yawspeed
            vehicle_ptr->set_roll(EulerRPY[0]);                           // We still pass the roll onto the state
            vehicle_ptr->set_pitch(EulerRPY[1]);                          // We still pass the pitch onto the state
            vehicle_ptr->set_yaw(0.0);                                    // We zero in the heading. 
        }

        // STATE 2 : Collecting correction data for only position and not yaw -------------------------------------------------------------------------
        else if ( bitmask == ReadMachineState::ACQUIRE_POS_CORRECTION )
        {
            // Store the initial position values
            stateMachine.corrections.x = msg->position[0];
            stateMachine.corrections.y = msg->position[1];
            stateMachine.corrections.z = msg->position[2];

            // Load in the quaternion from the pixhawk
            quaternion_pix = Quaterniond(msg->q[0],
                                         msg->q[1],
                                         msg->q[2],
                                         msg->q[3]);

            // Convert the quaternion to Euler angles
            EulerRPY = quaternionToEulerAnglesRPY(quaternion_pix.w(),
                                                  quaternion_pix.x(),
                                                  quaternion_pix.y(),
                                                  quaternion_pix.z());

            // ------------------------------------------------------------- > Set Values < --
            // Set the initial values as zero in the vehicle class where needed.
            vehicle_ptr->set_pixhawktime(msg->timestamp);
            vehicle_ptr->set_x(0.0);                                      // We zero in the position in x 
            vehicle_ptr->set_y(0.0);                                      // We zero in the position in y
            vehicle_ptr->set_z(0.0);                                      // We zero in the position in z
            vehicle_ptr->set_vx(0.0);                                     // We zero in the velocity in x
            vehicle_ptr->set_vy(0.0);                                     // We zero in the velocity in y
            vehicle_ptr->set_vz(0.0);                                     // We zero in the velocity in z
            vehicle_ptr->set_q0(quaternion_pix.w());                      // We pass on the orientation data
            vehicle_ptr->set_q1(quaternion_pix.x());                      // We pass on the orientation data
            vehicle_ptr->set_q2(quaternion_pix.y());                      // We pass on the orientation data
            vehicle_ptr->set_q3(quaternion_pix.z());                      // We pass on the orientation data
            vehicle_ptr->set_rollspeed(0.0);                              // We zero in the rollspeed
            vehicle_ptr->set_pitchspeed(0.0);                             // We zero in the pitchspeed
            vehicle_ptr->set_yawspeed(0.0);                               // We zero in the yawspeed
            vehicle_ptr->set_roll(EulerRPY[0]);                           // We still pass the roll onto the state
            vehicle_ptr->set_pitch(EulerRPY[1]);                          // We still pass the pitch onto the state
            vehicle_ptr->set_yaw(EulerRPY[2]);                            // We still pass the yaw onto the state 

        }

        // STATE 3 : Correcting both position and yaw data --------------------------------------------------------------------------------------------
        else if ( bitmask == ReadMachineState::ZERO_POS_YAW_CORRECTION)
        {
            // ------------------------------------------------------------- > Calculations < --
            // Load in the quaternion from the pixhawk
            quaternion_pix = Quaterniond(msg->q[0],
                                         msg->q[1],
                                         msg->q[2],
                                         msg->q[3]);

            // Zero the quaternion in yaw by post-multiplying with inverse of yaw quaternion
            zeroed_quaternion = quaternion_pix*stateMachine.corrections.yaw_quaternion.inverse();
            
            // Convert quaternions to euler
            EulerRPY = quaternionToEulerAnglesRPY(zeroed_quaternion.w(),
                                                  zeroed_quaternion.x(),
                                                  zeroed_quaternion.y(),
                                                  zeroed_quaternion.z());

            // ------------------------------------------------------------- > Set Values < --
            // Modify the data in the vehicle object in flight_bride atomically
            vehicle_ptr->set_pixhawktime(msg->timestamp);
            vehicle_ptr->set_x(msg->position[0]  - stateMachine.corrections.x); // We zero in the position in x 
            vehicle_ptr->set_y(msg->position[1]  - stateMachine.corrections.y); // We zero in the position in y
            vehicle_ptr->set_z(msg->position[2]  - stateMachine.corrections.z); // We zero in the position in z
            vehicle_ptr->set_vx(msg->velocity[0]);
            vehicle_ptr->set_vy(msg->velocity[1]);
            vehicle_ptr->set_vz(msg->velocity[2]);
            vehicle_ptr->set_q0(zeroed_quaternion.w());                         // We zero in the yaw of the quaternion
            vehicle_ptr->set_q1(zeroed_quaternion.x());                         // We zero in the yaw of the quaternion
            vehicle_ptr->set_q2(zeroed_quaternion.y());                         // We zero in the yaw of the quaternion
            vehicle_ptr->set_q3(zeroed_quaternion.z());                         // We zero in the yaw of the quaternion
            vehicle_ptr->set_rollspeed(msg->angular_velocity[0]);
            vehicle_ptr->set_pitchspeed(msg->angular_velocity[1]);
            vehicle_ptr->set_yawspeed(msg->angular_velocity[2]);
            vehicle_ptr->set_roll(EulerRPY[0]);
            vehicle_ptr->set_pitch(EulerRPY[1]);
            vehicle_ptr->set_yaw(EulerRPY[2]);                                  // We zero in the heading using the quaternion. 
        }

        // STATE 4: Correcting only position data and not yaw data ------------------------------------------------------------------------------------
        else if ( bitmask == ReadMachineState::ZERO_POS_CORRECTION )
        {
            // ------------------------------------------------------------- > Calculations < --
            // Load in the quaternion from the pixhawk
            quaternion_pix = Quaterniond(msg->q[0],
                msg->q[1],
                msg->q[2],
                msg->q[3]);

            // Convert the quaternion to Euler angles
            EulerRPY = quaternionToEulerAnglesRPY(quaternion_pix.w(),
                                                  quaternion_pix.x(),
                                                  quaternion_pix.y(),
                                                  quaternion_pix.z());

            // ------------------------------------------------------------- > Set Values < --
            // Modify the data in the vehicle object in flight_bride atomically
            vehicle_ptr->set_pixhawktime(msg->timestamp);
            vehicle_ptr->set_x(msg->position[0]  - stateMachine.corrections.x); // We zero in the position in x 
            vehicle_ptr->set_y(msg->position[1]  - stateMachine.corrections.y); // We zero in the position in y
            vehicle_ptr->set_z(msg->position[2]  - stateMachine.corrections.z); // We zero in the position in z
            vehicle_ptr->set_vx(msg->velocity[0]);
            vehicle_ptr->set_vy(msg->velocity[1]);
            vehicle_ptr->set_vz(msg->velocity[2]);
            vehicle_ptr->set_q0(quaternion_pix.w());                         
            vehicle_ptr->set_q1(quaternion_pix.x());                         
            vehicle_ptr->set_q2(quaternion_pix.y());                         
            vehicle_ptr->set_q3(quaternion_pix.z());                         
            vehicle_ptr->set_rollspeed(msg->angular_velocity[0]);
            vehicle_ptr->set_pitchspeed(msg->angular_velocity[1]);
            vehicle_ptr->set_yawspeed(msg->angular_velocity[2]);
            vehicle_ptr->set_roll(EulerRPY[0]);
            vehicle_ptr->set_pitch(EulerRPY[1]);
            vehicle_ptr->set_yaw(EulerRPY[2]);                                  
        }

        // --------------------------------------------------------------------------------------------------------------------------------------------
        // If greater than the controller start time. Set the correction data Acquisition bit to true 
        // to signify that the correction data has been acquired.
        if (!stateMachine.zeroAcq && vehicle_ptr->get_controltime() > flight_params_ptr->start_time) {stateMachine.zeroAcq = true;}
        
    }

}  // namespace read_pix