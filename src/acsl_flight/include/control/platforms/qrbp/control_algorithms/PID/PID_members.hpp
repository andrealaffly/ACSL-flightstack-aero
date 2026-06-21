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
 * File:        PID_members.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 26, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID controller class members.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*
________/\\\__________/\\\\\\\\\______/\\\\\\\\\\\\\____/\\\\\\\\\\\\\___        
 _____/\\\\/\\\\_____/\\\///////\\\___\/\\\/////////\\\_\/\\\/////////\\\_       
  ___/\\\//\////\\\__\/\\\_____\/\\\___\/\\\_______\/\\\_\/\\\_______\/\\\_      
   __/\\\______\//\\\_\/\\\\\\\\\\\/____\/\\\\\\\\\\\\\\__\/\\\\\\\\\\\\\/__     
    _\//\\\______/\\\__\/\\\//////\\\____\/\\\/////////\\\_\/\\\/////////____    
     __\///\\\\/\\\\/___\/\\\____\//\\\___\/\\\_______\/\\\_\/\\\_____________   
      ____\////\\\//_____\/\\\_____\//\\\__\/\\\_______\/\\\_\/\\\_____________  
       _______\///\\\\\\__\/\\\______\//\\\_\/\\\\\\\\\\\\\/__\/\\\_____________ 
        _________\//////___\///________\///__\/////////////____\///______________
*/

#ifndef CONTROLLERS_PID_MEMBERS_HPP_
#define CONTROLLERS_PID_MEMBERS_HPP_

#include "Eigen/Dense"
#include <chrono>
#include "qrbp.hpp"                   // Header file for vehicle specific information and some other functions
#include "aero_qrbp.hpp"              // Header file for aerodynamic specific information and some other functions

using namespace Eigen;
namespace _qrbp_{
namespace _pid_{

// Structure for all parameter members of the controller
struct controller_internal_parameters
{
  // Proportional Gains for translational
  Matrix<double, 3, 3> Kp_tran;

  // Integral Gains for translational
  Matrix<double, 3, 3> Ki_tran;

  // Differential Gains for translational
  Matrix<double, 3, 3> Kd_tran;

  // Proportional Gains for rotational
  Matrix<double, 3, 3> Kp_rot;

  // Integral Gains for rotational
  Matrix<double, 3, 3> Ki_rot;

  // Differential Gains for rotational
  Matrix<double, 3, 3> Kd_rot;

};

// Structure for all the aerodynamic members of the controller
struct controller_internal_members_aero
{
  AeroStates states;
  AeroCoeff coeff;
  AeroDynamicMembers dyn;
};

// Structure for the members that are mapped to the rk4 vector after integration
struct controller_integrated_state_members
{
  // Translational Integral error
  Matrix<double, 3, 1> e_tran_pos_I;

  // States for filter
  Matrix<double, 2, 1> state_roll_d_filter;
  Matrix<double, 2, 1> state_pitch_d_filter;

  Matrix<double, 2, 1> state_roll_dot_d_filter;
  Matrix<double, 2, 1> state_pitch_dot_d_filter;

  // Integral error in Euler angles
  Matrix<double, 3, 1> e_eta_I;

};

// Structure for the internal members of the controller
struct controller_internal_members
{
  // Time
  double t; 

  // Control execution duration
  double alg_duration;
  std::chrono::high_resolution_clock::time_point alg_start_time;
  std::chrono::high_resolution_clock::time_point alg_end_time;

  controller_internal_members_aero aero;

  // Rotation matrices
  Matrix<double, 3,3> R_J_I;
  Matrix<double, 3,3> R_I_J;
  Matrix<double, 3,3> R_W_J;

  // Jacobian matrices
  Matrix<double, 3,3> Jacobian;
  Matrix<double, 3,3> Jacobian_inv;
  
  // Translational User Commands
  Matrix<double, 3, 1> r_user;
  Matrix<double, 3, 1> r_dot_user;
  Matrix<double, 3, 1> r_ddot_user;

  // Quaternion
  Quaterniond q;

  // Position in I
  Matrix<double, 3, 1> x_tran_pos;

  // Velocity in I
  Matrix<double, 3, 1> x_tran_vel;

  // Translational velocities in J
  Matrix<double, 3,1> x_tran_vel_J;

  // Translational error
  Matrix<double, 3, 1> e_tran_pos;
  Matrix<double, 3, 1> e_tran_vel;

  // Translational baseline control input 
  Matrix<double, 3, 1> mu_tran_baseline;

  // Translational control input in the inertial frame
  Matrix<double, 3, 1> mu_tran_I;

  // Translational control input in the body frame
  Matrix<double, 3, 1> mu_tran_J;

  // Desired Euler Angles
  Matrix<double, 3, 1> eta_rot_d;

  // Desired Euler Rates
  Matrix<double, 3, 1> eta_rot_rate_d;

  // Desired Euler Acceleration
  Matrix<double, 3, 1> eta_rot_acceleration_d;

  // Internal States for filter
  Matrix<double, 2, 1> internal_state_roll_d_filter;
  Matrix<double, 2, 1> internal_state_pitch_d_filter;

  Matrix<double, 2, 1> internal_state_roll_dot_d_filter;
  Matrix<double, 2, 1> internal_state_pitch_dot_d_filter;
  
  // Euler Angles
  Matrix<double, 3, 1> eta_rot;

  // Angular rates
  Matrix<double, 3, 1> eta_rot_rate;

  // Angular velocities
  Matrix<double, 3, 1> omega_rot;

  // Error in Euler angles and rates
  Matrix<double, 3, 1> e_eta;
  Matrix<double, 3, 1> e_eta_rate;

  // Rotational Baseline control input
  Matrix<double, 3, 1> tau_rot_baseline;

  // Rotational control input
  Matrix<double, 3, 1> tau_rot;
  
  // Final Control Input
  Matrix<double, 4, 1> u;

  // Thrust 
  Matrix<double, 4,1> Thrust;

  // Thrust after Saturation
  Matrix<double, 4,1> Sat_Thrust;

};

} // namespace _pid_
} // namespace _qrbp_

#endif  // CONTROLLERS_PID_MEMBERS_HPP_