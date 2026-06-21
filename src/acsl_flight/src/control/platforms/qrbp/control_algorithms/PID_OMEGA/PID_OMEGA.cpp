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
 * File:        PID_OMEGA.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 25, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID  with angular velocities for the QRBP. Inherts the
 *              class controller_base for the basic functionality that 
 *              is to be used for all control algorithms.
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

#include "PID_OMEGA.hpp"

namespace _qrbp_{
namespace _pid_omega_{

// Constructor - Take care to initialize the logger
pid_omega::pid_omega(flight_params* p, const std::string & controller_log_dir_) :
           controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_) {

    // Reading in the parameters
    read_params("./src/acsl_flight/params/control_algorithms/qrbp/PID_OMEGA/gains_PID_OMEGA.json");

    // Initial Conditions
    init();
}

// Destructor
pid_omega::~pid_omega(){
    // Destructor implementation
}

// Implementing fucntions from controller_base
void pid_omega::read_params(const std::string& jsonFile) {

    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Translational parameters
    cip.Kp_tran = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KP_translational"]);
    cip.Ki_tran = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KI_translational"]);
    cip.Kd_tran = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KD_translational"]);

    // Rotational parameters
    cip.Kp_rot = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KP_rotational"]);
    cip.Ki_rot = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KI_rotational"]);
    cip.Kd_rot = jsonToScaledMatrix<double, 3, 3>(j["BASELINE"]["KD_rotational"]);
    
}

// Implementing virutal functions from controller_base
void pid_omega::init(){
    // Fill up the Initial Conditions
    y.fill(0.0);
    dy.fill(0.0);
}

void pid_omega::update(double time, 
                       double x,
                       double y,
                       double z,
                       double vx,
                       double vy,
                       double vz,
                       double q0,
                       double q1,
                       double q2,
                       double q3,
                       double roll,
                       double pitch,
                       double yaw,
                       double w_x,
                       double w_y,
                       double w_z)     
{
    // 1. Assign all the states ----------------------------------------------------
    cim.t = time;

    // Assign the Translational States
    cim.x_tran_pos << x,y,z;
    cim.x_tran_vel << vx, vy, vz;

    // Assign the Rotational States
    cim.eta_rot << roll, pitch, yaw;
    cim.omega_rot << w_x, w_y, w_z;

    // Assign the quaternions
    cim.q.w() = q0;
    cim.q.z() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // 2. Get the reference trajectory ---------------------------------------------
    ud.updateUserDefinedTrajectory(time);
    cim.r_user = ud.getUserDefinedPosition();
    cim.r_dot_user = ud.getUserDefinedVelocity();
    cim.r_ddot_user = ud.getUserDefinedAcceleration();   

    // Set the desired yaw and yaw_rate; we don't need the acceleration as 
    // we are going to compute it from the filter.
    cim.eta_rot_d(2) = ud.getUserDefinedYaw();
    cim.eta_rot_rate_d(2) = ud.getUserDefinedYawRate();

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    cim.R_I_J = rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.R_J_I = rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));

    // 5. Compute the body velocities in the quadcopter frame from the inertial velocities
    cim.x_tran_vel_J = cim.R_I_J * cim.x_tran_vel;

    // 6. Compute the square of the norm of velocity in the body frame - it is the same for both quadcopter and biplane
    //    amd compute the aerodynamic angles
    cim.aero.states = computeAeroStatesNACA0012Quad(cim.x_tran_vel_J(0), cim.x_tran_vel_J(1), cim.x_tran_vel_J(2),
                                                    cim.omega_rot(0),  cim.omega_rot(1),  cim.omega_rot(2));

    // 7. Compute the Direction Cosine Matrix in the quadcopter frame at the center of gravity
    cim.R_W_J = aeroDCMQuadNED(cim.aero.states.alpha, cim.aero.states.beta);
    
    // 8.  Compute the aerodynamic coefficeints    
    cim.aero.coeff = compute_aero_coefficients(cim.aero.states);

    // 10. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_omega::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    assignElementsToMembers(csm.state_roll_d_filter, y, index);
    assignElementsToMembers(csm.state_pitch_d_filter, y, index);
    assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
    assignElementsToMembers(csm.e_tran_pos_I, y, index);
    assignElementsToMembers(csm.e_rot_eta_I, y, index);
}

// Model function for integration
void pid_omega::model([[maybe_unused]] const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, [[maybe_unused]] const double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    assignElementsToDxdt(cim.internal_state_roll_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
    assignElementsToDxdt(cim.e_tran_pos, dy, index);
    assignElementsToDxdt(cim.e_rot_eta, dy, index);
}

// Function to compute the outerloop control in I
void pid_omega::compute_translational_control_in_I()
{
    // Compute the error in the states
    cim.e_tran_pos << cim.x_tran_pos - cim.r_user;
    cim.e_tran_vel << cim.x_tran_vel - cim.r_dot_user;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos 
                                   - cip.Kd_tran * cim.e_tran_vel
                                   - cip.Ki_tran * csm.e_tran_pos_I
                                   + cim.r_ddot_user );

    // Compute with the dynamic inversion without aerodynamics
    cim.mu_tran_I << cim.mu_tran_baseline - MASS * G * e3_basis;
}

// Compute the orientation commands and the desired total thrust
void pid_omega::compute_u1_eta_d()
{
	// Compute the virtual forces in J
	cim.mu_tran_J << cim.R_I_J * cim.mu_tran_I;

	// Compute u1 - Total Thrust
	cim.u(0) = sqrt(  pow(cim.mu_tran_J(0), 2)
							 		+ pow(cim.mu_tran_J(1), 2)
								 	+ pow(cim.mu_tran_J(2), 2));

	// Retrieve the desired yaw and yaw rate
	cim.eta_rot_d(2) = ud.getUserDefinedYaw();
	cim.eta_rot_rate_d(2) = ud.getUserDefinedYawRate();									

	// Compute the desired roll
	double r1 = cim.mu_tran_J(1) / cim.u(0);
	cim.eta_rot_d(0) = atan2(r1, sqrt(1 - pow(r1,2)));

	// Compute the desired pitch
	cim.eta_rot_d(1) = atan2( -cim.mu_tran_J(0), -cim.mu_tran_J(2) );

	// Compute the internal state for angular rates
	cim.internal_state_roll_d_filter << A_filter_roll_ref * csm.state_roll_d_filter
                                      + B_filter_roll_ref * cim.eta_rot_d(0);

	cim.internal_state_pitch_d_filter << A_filter_pitch_ref * csm.state_pitch_d_filter
                                       + B_filter_pitch_ref * cim.eta_rot_d(1);


	// Compute the desired roll and pitch rates
	cim.eta_rot_rate_d(0) = C_filter_roll_ref * csm.state_roll_d_filter;    
	cim.eta_rot_rate_d(1) = C_filter_pitch_ref * csm.state_pitch_d_filter;	

	// Compute the Jacobian with the current orientation
	cim.Jacobian = jacobianMatrix(cim.eta_rot(0), cim.eta_rot(1));

	// Compute the desired angular velocities
	cim.omega_rot_d << cim.Jacobian * cim.eta_rot_rate_d;
	
	// Compute the internal state for angular accelration
	cim.internal_state_omega_x_d_filter << A_filter_roll_dot_ref * csm.state_omega_x_d_filter
																				+ B_filter_roll_dot_ref * cim.omega_rot_d(0);

	cim.internal_state_omega_y_d_filter << A_filter_pitch_dot_ref * csm.state_omega_y_d_filter
																				+ B_filter_pitch_dot_ref * cim.omega_rot_d(1);

	cim.internal_state_omega_z_d_filter << A_filter_yaw_dot_ref * csm.state_omega_z_d_filter
																				+ B_filter_yaw_dot_ref * cim.omega_rot_d(2);

	// Compute the desired angular acceleration
	cim.alpha_rot_d(0) = C_filter_roll_dot_ref * csm.state_omega_x_d_filter;
	cim.alpha_rot_d(1) = C_filter_pitch_dot_ref * csm.state_omega_y_d_filter;
	cim.alpha_rot_d(2) = C_filter_yaw_dot_ref * csm.state_omega_z_d_filter;	
}


// Compute the rotational control
void pid_omega::compute_rotational_control()
{
    // Compute the error in the rotational states
    cim.e_rot_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
    cim.e_rot_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
    cim.e_rot_eta(2) = makeYawAngularErrorContinuous<double>(cim.eta_rot(2), cim.eta_rot_d(2)); 

    cim.e_rot_omega << cim.omega_rot - cim.omega_rot_d;

    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_rot * cim.e_rot_eta 
                                                 - cip.Kd_rot * cim.e_rot_omega
                                                 - cip.Ki_rot * csm.e_rot_eta_I
                                                 + cim.alpha_rot_d);

    // Compute with dynamic inversion without aerodynamics
    cim.tau_rot << cim.tau_rot_baseline 
                   + cim.omega_rot.cross(inertia_matrix_q * cim.omega_rot);

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);                                             
}

// Function to compute the normalized thrusts
void pid_omega::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_qrbp * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = (float) evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
    control_input(1) = (float) evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
    control_input(2) = (float) evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
    control_input(3) = (float) evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));
}

// Function to print to terminal during runtime
void pid_omega::debug2terminal()
{

}

// Function that is called in control.cpp
void pid_omega::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics 
    cim.aero.dyn = compute_aero_forces_moments(cim.aero.states, cim.aero.coeff, cim.R_J_I, cim.R_W_J);
    
    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the thrust needed and the desired angles
    compute_u1_eta_d();

    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid_omega::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 7. Log the Data after all the calculataions
    logger.logLogData();

    // optional debug
    debug2terminal();
}

// Implementing getter functions from controller_base
float pid_omega::get_t1() const {
    return control_input(0);
}

float pid_omega::get_t2() const {
    return control_input(1);
}

float pid_omega::get_t3() const {
    return control_input(2);
}

float pid_omega::get_t4() const {
    return control_input(3);
}

float pid_omega::get_t5() const {
    return control_input(4);
}

float pid_omega::get_t6() const {
    return control_input(5);
}

float pid_omega::get_t7() const {
    return control_input(6);
}

float pid_omega::get_t8() const {
    return control_input(7);
}

} // namespace _pid_omega_
} // namespace _qrbp_
