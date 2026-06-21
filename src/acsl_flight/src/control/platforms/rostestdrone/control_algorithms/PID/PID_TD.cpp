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
 * File:        PID_TD.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 10, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID for the rostestdrone. Inherts the class controller_base
 *              for the basic functionality that is to be used for all 
 *              control algorithms.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

/*               _           _      _                      
 _ __ ___  ___| |_ ___ ___| |_ __| |_ __ ___  _ __   ___ 
| '__/ _ \/ __| __/ _ / __| __/ _` | '__/ _ \| '_ \ / _ \
| | | (_) \__ | ||  __\__ | || (_| | | | (_) | | | |  __/
|_|  \___/|___/\__\___|___/\__\__,_|_|  \___/|_| |_|\___|
                                                         
*/

#include "PID_TD.hpp"

namespace _rostestdrone_{
namespace _pid_{

// Constructor - Take care to initialize the logger
pid::pid(flight_params* p, const std::string & controller_log_dir_) : 
     controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_) {
    
    // Reading in the parameters
    read_params("./src/acsl_flight/params/control_algorithms/rostestdrone/PID/gains_PID.json");

    // Initial Conditions
    init();    
}

// Destructor
pid::~pid() {
    // Destructor implementation
}

// Implementing functions from controller_base
void pid::read_params(const std::string& jsonFile) {
    // Implementation here
    
    std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;

    cip.Kp_tran = jsonToMatrix<double, 3, 3>(j["KP_translational"]);
    cip.Ki_tran = jsonToMatrix<double, 3, 3>(j["KI_translational"]);
    cip.Kd_tran = jsonToMatrix<double, 3, 3>(j["KD_translational"]);
    
    cip.Kp_rot = jsonToMatrix<double, 3, 3>(j["KP_rotational"]);
    cip.Ki_rot = jsonToMatrix<double, 3, 3>(j["KI_rotational"]);
    cip.Kd_rot = jsonToMatrix<double, 3, 3>(j["KD_rotational"]);

}

// Implementing functions from controller_base
void pid::init(){
    // Fill up the Initial Condition
    y.fill(0.0);
    dy.fill(0.0);
    
    // Fill up the zeros Vector for resetting events in the Integrator
    zeros.fill(0.0);
}


void pid::update(double time, 
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
    cim.x_tran_vel << vx,vy,vz;

    // Assign the Rotational States
    cim.eta_rot << roll,pitch,yaw;
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

    // Set the desired yaw, yaw rate and yaw acceleration
    cim.eta_rot_d(2) = ud.getUserDefinedYaw();
    cim.eta_rot_rate_d(2) = ud.getUserDefinedYawRate();
    cim.eta_rot_acceleration_d(2) = ud.getUserDefinedYawRateDot();

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices and the jacobian Inverse -------------------
    cim.R_J_I = rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.R_I_J = rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.Jacobian_inv = jacobianMatrixInverse(cim.eta_rot(0), cim.eta_rot(1));

    // 6. Compute the angular rates from the angular velocities --------------------
    cim.eta_rot_rate = cim.Jacobian_inv*cim.omega_rot;
    
    // 7. Assign the values from the integrator -----------------------------------
    assign_from_rk4();
}

void pid::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    assignElementsToMembers(csm.state_roll_d_filter, y, index);
    assignElementsToMembers(csm.state_pitch_d_filter, y, index);
    assignElementsToMembers(csm.state_roll_dot_d_filter, y, index);
    assignElementsToMembers(csm.state_pitch_dot_d_filter, y, index);
    assignElementsToMembers(csm.e_tran_pos_I, y, index);
    assignElementsToMembers(csm.e_eta_I, y, index);
}

void pid::model([[maybe_unused]] const rk4_array<double, 14> &y, rk4_array<double, 14> &dy, [[maybe_unused]] const double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    assignElementsToDxdt(cim.internal_state_roll_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_roll_dot_d_filter, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_dot_d_filter, dy, index);
    assignElementsToDxdt(cim.e_tran_pos, dy, index);
    assignElementsToDxdt(cim.e_eta, dy, index);
}


void pid::compute_translational_control_in_I()
{ 
    // Compute the error in the states
    cim.e_tran_pos << cim.x_tran_pos - cim.r_user;
    cim.e_tran_vel << cim.x_tran_vel - cim.r_dot_user;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos 
                                   - cip.Kd_tran * cim.e_tran_vel
                                   - cip.Ki_tran * csm.e_tran_pos_I
                                   + cim.r_ddot_user );

    // Compute with the dynamic inversion
    cim.mu_tran_I << cim.mu_tran_baseline - MASS * G * e3_basis;
}

void pid::compute_u1_eta_d()
{
    // Compute the virtual forces in J
    cim.mu_tran_J << cim.R_I_J * cim.mu_tran_I;

    // Compute u1 - Total Thrust
    cim.u(0) = sqrt(  pow(cim.mu_tran_J(0), 2)
                    + pow(cim.mu_tran_J(1), 2)
                    + pow(cim.mu_tran_J(2), 2)
                    );

    // Compute the desired roll
    double r1 = cim.mu_tran_J(1) / cim.u(0);
    cim.eta_rot_d(0) = atan2(r1, sqrt(1 - pow(r1,2)));

    // Compute the desired pitch
    cim.eta_rot_d(1) = atan2( -cim.mu_tran_J(0), -cim.mu_tran_J(2) );

    // Compute the internal state
    cim.internal_state_roll_d_filter << A_filter_roll_ref * csm.state_roll_d_filter
                                      + B_filter_roll_ref * cim.eta_rot_d(0);

    cim.internal_state_pitch_d_filter << A_filter_pitch_ref * csm.state_pitch_d_filter
                                       + B_filter_pitch_ref * cim.eta_rot_d(1);

    cim.internal_state_roll_dot_d_filter << A_filter_roll_dot_ref * csm.state_roll_dot_d_filter
                                          + B_filter_roll_dot_ref * cim.eta_rot_rate_d(0);

    cim.internal_state_pitch_dot_d_filter << A_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter
                                           + B_filter_pitch_dot_ref * cim.eta_rot_rate_d(1); 

    // Compute the desired pitch and roll rates
    cim.eta_rot_rate_d(0) = C_filter_roll_ref * csm.state_roll_d_filter;    
    cim.eta_rot_rate_d(1) = C_filter_pitch_ref * csm.state_pitch_d_filter;

    // Compute the desired pitch and roll acceleration
    cim.eta_rot_acceleration_d(0) = C_filter_roll_dot_ref * csm.state_roll_dot_d_filter;
    cim.eta_rot_acceleration_d(1) = C_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter;


}

void pid::compute_rotational_control()
{
    // Compute the error in the rotational states
    cim.e_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
    cim.e_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
    cim.e_eta(2) = makeYawAngularErrorContinuous<double>(cim.eta_rot(2), cim.eta_rot_d(2)); 

    cim.e_eta_rate << cim.eta_rot_rate - cim.eta_rot_rate_d;

    // Compute the baseline control input
    cim.tau_rot_baseline << inertia_matrix_q * ( - cip.Kp_rot * cim.e_eta 
                                                 - cip.Kd_rot * cim.e_eta_rate
                                                 - cip.Ki_rot * csm.e_eta_I
                                                 + cim.eta_rot_acceleration_d   
                                             );

    // Compute with dynamic inversion
    cim.tau_rot << cim.tau_rot_baseline + cim.omega_rot.cross(inertia_matrix_q * cim.omega_rot);

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);

}

void pid::compute_normalized_thrusts()
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

void pid::debug2terminal()
{
    // FLIGHTSTACK_INFO_STREAM_NO_TAG("Execution Time:", cim.alg_duration);
}

void pid::run(const double time_step_rk4_) {
    
    // Process the dynamics --------------------------------------------------------
    // 1. Compute the translational control input
    compute_translational_control_in_I();

    // 2. Compute the thrust needed and the desired angles
    compute_u1_eta_d();
   
    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid::model, this, bph::_1, bph::_2, bph::_3),
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
float pid::get_t1() const {
    return control_input(0);
}

float pid::get_t2() const {
    return control_input(1);
}

float pid::get_t3() const {
    return control_input(2);
}

float pid::get_t4() const {
    return control_input(3);
}

float pid::get_t5() const {
    return control_input(4);
}

float pid::get_t6() const {
    return control_input(5);
}

float pid::get_t7() const {
    return control_input(6);
}

float pid::get_t8() const {
    return control_input(7);
}



} // namespace _pid_
} // namespace _rostestdrone_