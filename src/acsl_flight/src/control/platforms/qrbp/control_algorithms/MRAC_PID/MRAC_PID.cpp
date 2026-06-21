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
 * File:        MRAC_PID.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        May 14, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC_PID for the QRBP. Inherts the class controller_base 
 *              for the basic functionality that is to be used for all 
 *              control algorithms.
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

#include "MRAC_PID.hpp"

namespace _qrbp_{
namespace _mrac_pid_{

// Constructor - Take care to initialize the logger
mrac_pid::mrac_pid(flight_params* p, const std::string & controller_log_dir_) :
          controller_base(p), ud(p), logger(&cim, &csm, &control_input, controller_log_dir_) {
    
    // Reading in the parameters
    read_params("./src/acsl_flight/params/control_algorithms/qrbp/MRAC_PID/gains_MRAC_PID.json");

    // Initial Conditions
    init();    
}

// Destructor
mrac_pid::~mrac_pid() {
    // Destructor implementation
}

// Implementing virtual functions from controller_base
void mrac_pid::read_params(const std::string& jsonFile) {
    
    // Create the file stream
    std::ifstream file(jsonFile);
	nlohmann::json j;
	file >> j;

    // Translational Parameters
    cip.Kp_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KP_refmod_translational"]);
    cip.Ki_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KI_refmod_translational"]);
    cip.Kd_refmod_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KD_refmod_translational"]);
    cip.Kp_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KP_translational"]);
    cip.Ki_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KI_translational"]);
    cip.Kd_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["KD_translational"]);
    cip.Gamma_x_tran = jsonToScaledMatrix<double, 6, 6>(j["TRANSLATIONAL"]["Gamma_x_translational"]);
    cip.Gamma_r_tran = jsonToScaledMatrix<double, 3, 3>(j["TRANSLATIONAL"]["Gamma_r_translational"]);
    cip.Gamma_Theta_tran = jsonToScaledMatrix<double, 30, 30>(j["TRANSLATIONAL"]["Gamma_Theta_translational"]);
    cip.Q_tran = jsonToScaledMatrix<double, 6, 6>(j["TRANSLATIONAL"]["Q_translational"]);

    // Quadcopter Parameters
    cip.Kp_omega_cmd_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KP_omega_cmd_rotational_q"]);
    cip.Ki_omega_cmd_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KI_omega_cmd_rotational_q"]);
    cip.Kp_omega_ref_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KP_omega_ref_rotational_q"]);
    cip.Ki_omega_ref_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KI_omega_ref_rotational_q"]);
    cip.Kp_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KP_rotational_q"]);
    cip.Ki_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KI_rotational_q"]);
    cip.Kd_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["KD_rotational_q"]);
    cip.Gamma_x_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["Gamma_x_rotational_q"]);
    cip.Gamma_r_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["Gamma_r_rotational_q"]);
    cip.Gamma_Theta_rot_q = jsonToScaledMatrix<double, 12, 12>(j["QUADCOPTER"]["Gamma_Theta_rotational_q"]);
    cip.Q_rot_q = jsonToScaledMatrix<double, 3, 3>(j["QUADCOPTER"]["Q_rotational_q"]);

    // Biplane Parameters
    cip.Kp_omega_cmd_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KP_omega_cmd_rotational_b"]);
    cip.Ki_omega_cmd_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KI_omega_cmd_rotational_b"]);
    cip.Kp_omega_ref_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KP_omega_ref_rotational_b"]);
    cip.Ki_omega_ref_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KI_omega_ref_rotational_b"]);
    cip.Kp_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KP_rotational_b"]);
    cip.Ki_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KI_rotational_b"]);
    cip.Kd_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["KD_rotational_b"]);
    cip.Gamma_x_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["Gamma_x_rotational_b"]);
    cip.Gamma_r_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["Gamma_r_rotational_b"]);
    cip.Gamma_Theta_rot_b = jsonToScaledMatrix<double, 12, 12>(j["BIPLANE"]["Gamma_Theta_rotational_b"]);
    cip.Q_rot_b = jsonToScaledMatrix<double, 3, 3>(j["BIPLANE"]["Q_rotational_b"]);
}

// Implementing virtual functions from controller_base
void mrac_pid::init() {
    // Initial conditions for rk4 integrator
    y.fill(0.0);
    dy.fill(0.0);

    // Fill up the zeros Vector for resetting events in the Integrator
    zeros.fill(0.0);

    /// Translational - Initialize matrices
    // Initilaize to zero the 6x6 matrix and set the top-right 3x3 block as follows
    initMat(cip.A_tran);
    cip.A_tran.block<3, 3>(0, 3) = Matrix3d::Identity();

    // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
    initMat(cip.B_tran);
    cip.B_tran.block<3, 3>(3, 0) = Matrix3d::Identity();

    // Initialize to zero the 6x6 matrix
    initMat(cip.A_ref_tran);
    // Set the top-right 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(0, 3) = Matrix3d::Identity();
    // Set the bottom-left 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(3, 0) = -cip.Kp_refmod_tran;
    // Set the bottom-right 3x3 block as follows
    cip.A_ref_tran.block<3, 3>(3, 3) = -cip.Kd_refmod_tran;

    // Initialize to zero the 6x3 matrix and set the bottom 3x3 block as follows
    initMat(cip.B_ref_tran);
    cip.B_ref_tran.block<3, 3>(3, 0) = (1 / MASS) * Matrix3d::Identity();

    // Solve the continuous Lyapunov equation to compute P_translational
    cip.P_tran = RealContinuousLyapunovEquation(cip.A_ref_tran, cip.Q_tran);

    /// Quadcopter - Initialize matrices
    // Initialize to zero the 3x3 matrix
    initMat(cip.A_rot_q);

    // Initlize to identity the 3x3 matrix 
	cip.B_rot_q = Matrix3d::Identity();

    // Set the 3x3 matrix as follows
    cip.A_ref_rot_q = -cip.Kp_omega_ref_rot_q;

    // Set the 3x3 matrix as follows
    cip.B_ref_rot_q = Matrix3d::Identity();

    // Solve the continuous Lyapunov equation to compute P_rotational quadcopter
    cip.P_rot_q = RealContinuousLyapunovEquation(cip.A_ref_rot_q, cip.Q_rot_q);

    /// Biplane - Initialize matrices
    // Initialize to zero the 3x3 matrix
    initMat(cip.A_rot_b);

    // Initlize to identity the 3x3 matrix 
	cip.B_rot_b = Matrix3d::Identity();

    // Set the 3x3 matrix as follows
    cip.A_ref_rot_b = -cip.Kp_omega_ref_rot_b;

    // Set the 3x3 matrix as follows
    cip.B_ref_rot_b = Matrix3d::Identity();

    // Solve the continuous Lyapunov equation to compute P_rotational biplane
    cip.P_rot_b = RealContinuousLyapunovEquation(cip.A_ref_rot_b, cip.Q_rot_b);

    // Set the momentary button to false as the inital condition
    momentary_button = false;

    // Set the vehicle to quadcopter mode as the initial condition
    is_biplane = false;

    // Set the inertia matrix to the quadcopter mode as the initial condition
    cip.inertia_matrix = inertia_matrix_q;
}

void mrac_pid::update(double time, 
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
    cim.x_tran.head<3>() << x,y,z;
    cim.x_tran.tail<3>() << vx,vy,vz;

    // Assign the Rotational States
    cim.eta_rot_q << roll,pitch,yaw;
    cim.omega_rot_q << w_x, w_y, w_z;

    // Update the states to biplane mode. This function always measures the smallest angle.
    // update_states_to_biplane_mode(cim.eta_rot_q, cim.omega_rot_q, cim.eta_rot_b, cim.omega_rot_b);

    // Assign the quaternions
    cim.q.w() = q0;
    cim.q.z() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // Update the states to biplane mode. This function uses quaternions and tested code for gimbal locks
    // Look at qrbp.hpp for more methods
    // update_states_to_biplane_quaternions(cim.q, cim.eta_rot_q, cim.eta_rot_b, cim.omega_rot_b);
    update_states_to_biplane_quaternion_gmk(cim.q, cim.omega_rot_q, cim.eta_rot_b, cim.omega_rot_b);

    // 2. Check if you are in biplane or Quadcopter mode ---------------------------
    QRBPswitchFun(pitch, is_biplane, momentary_button);

    // 3. Get the reference trajectory ---------------------------------------------
    ud.updateUserDefinedTrajectory(time);
    cim.r_user = ud.getUserDefinedPosition();
    cim.r_dot_user = ud.getUserDefinedVelocity();
    cim.r_ddot_user = ud.getUserDefinedAcceleration();    

    // 4. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 5. Compute the rotation matrices
    cim.R_Jq_I = rotationMatrix321LocalToGlobal(cim.eta_rot_q(0), cim.eta_rot_q(1), cim.eta_rot_q(2));
    cim.R_I_Jq = rotationMatrix321GlobalToLocal(cim.eta_rot_q(0), cim.eta_rot_q(1), cim.eta_rot_q(2));
    cim.R_Jb_I = rotationMatrix321LocalToGlobal(cim.eta_rot_b(0), cim.eta_rot_b(1), cim.eta_rot_b(2));
    cim.R_I_Jb = rotationMatrix321GlobalToLocal(cim.eta_rot_b(0), cim.eta_rot_b(1), cim.eta_rot_b(2));
    
    // 6. Assign the rotation and the angular velocity for the inner loop here according to the mode
    //    and compute the Jacobin inverse and the angular rates. Also assign the inertia matrix
    if (is_biplane)
    {
        cim.eta_rot = cim.eta_rot_b;
        cim.omega_rot = cim.omega_rot_b;
        cim.Jacobian = jacobianMatrix(cim.eta_rot_b(0), cim.eta_rot_b(1));
        cim.Jacobian_inv = jacobianMatrixInverse(cim.eta_rot_b(0), cim.eta_rot_b(1));
        cim.eta_rot_rate = cim.Jacobian_inv*cim.omega_rot_b;
        cim.Jacobian_dot = jacobianMatrixDerivative(cim.eta_rot(0), cim.eta_rot(1), 
                                                    cim.eta_rot_rate(0), cim.eta_rot_rate(1));
        cip.inertia_matrix = inertia_matrix_b;
    }
    else
    {
        cim.eta_rot = cim.eta_rot_q;
        cim.omega_rot = cim.omega_rot_q;
        cim.Jacobian = jacobianMatrix(cim.eta_rot_q(0), cim.eta_rot_q(1));
        cim.Jacobian_inv = jacobianMatrixInverse(cim.eta_rot_q(0), cim.eta_rot_q(1));
        cim.eta_rot_rate = cim.Jacobian_inv*cim.omega_rot_q;
        cim.Jacobian_dot = jacobianMatrixDerivative(cim.eta_rot(0), cim.eta_rot(1), 
                                                    cim.eta_rot_rate(0), cim.eta_rot_rate(1));
        cip.inertia_matrix = inertia_matrix_q;
    }        
    
    // 7. Compute the body velocities in the quadcopter frame from the inertial velocities
    cim.x_tran_vel_Jq = cim.R_I_Jq * cim.x_tran.tail<3>();

    // 8. Compute the square of the norm of velocity in the body frame - it is the same for both quadcopter and biplane
    //    amd compute the aerodynamic angles
    cim.aero.states = computeAeroStatesNACA0012Quad(cim.x_tran_vel_Jq(0), cim.x_tran_vel_Jq(1), cim.x_tran_vel_Jq(2),
                                                    cim.omega_rot_q(0),  cim.omega_rot_q(1),  cim.omega_rot_q(2));
    
    // 9. Compute the Direction Cosine Matrix in the quadcopter frame at the center of gravity
    cim.R_W_Jq = aeroDCMQuadNED(cim.aero.states.alpha, cim.aero.states.beta);

    // 10. Compute the aerodynamic coefficeints    
    cim.aero.coeff = compute_aero_coefficients(cim.aero.states);

    // 11. Assign the values from the integrator -------------------------------------
    // If the momentary button to indicate a switch is hit reset the integrator to
    // emulate a start of a new mission in the inner loop or go ahead assigning the states as usual.
    if (momentary_button){ reset_integrator_states(); } else{ assign_from_rk4(); }
    
}

// Function to reset the states of the integrator
void mrac_pid::reset_integrator_states()
{
    int index = 0;
    assignElementsToMembers(csm.state_roll_d_filter_q, y, index);              // no action
    assignElementsToMembers(csm.state_pitch_d_filter_q, y, index);             // no action
    assignElementsToMembers(csm.state_roll_dot_d_filter_q, y, index);          // no action
    assignElementsToMembers(csm.state_pitch_dot_d_filter_q, y, index);         // no action
    assignElementsToMembers(csm.state_pitch_d_filter_b, y , index);            // no action
    assignElementsToMembers(csm.state_yaw_d_filter_b, y, index);               // no action
    assignElementsToMembers(csm.state_pitch_dot_d_filter_b, y, index);         // no action
    assignElementsToMembers(csm.state_yaw_dot_d_filter_b, y, index);           // no action
    assignElementsToMembers(csm.e_tran_pos_I, y, index);                       // no action
    resetElementsToMembers(csm.e_eta_I, y, index, zeros);                      // reset
    assignElementsToMembers(csm.x_tran_ref, y, index);                         // no action 
    assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);                   // no action
    assignElementsToMembers(csm.K_hat_x_tran, y, index);                       // no action
    assignElementsToMembers(csm.K_hat_r_tran, y, index);                       // no action
    assignElementsToMembers(csm.Theta_hat_tran, y, index);                     // no action
    resetElementsToMembers(csm.e_omega_ref_I, y, index, zeros);                // reset
    resetElementsToMembers(csm.omega_ref_rot, y, index, zeros);                // reset
    resetElementsToMembers(csm.K_hat_x_rot, y, index, zeros);                  // reset
    resetElementsToMembers(csm.K_hat_r_rot, y, index, zeros);                  // reset
    resetElementsToMembers(csm.Theta_hat_rot, y, index, zeros);                // reset

}

// Function to assign elements from the rk4 integrator
void mrac_pid::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    assignElementsToMembers(csm.state_roll_d_filter_q, y, index);              
    assignElementsToMembers(csm.state_pitch_d_filter_q, y, index);             
    assignElementsToMembers(csm.state_roll_dot_d_filter_q, y, index);          
    assignElementsToMembers(csm.state_pitch_dot_d_filter_q, y, index);         
    assignElementsToMembers(csm.state_pitch_d_filter_b, y , index);            
    assignElementsToMembers(csm.state_yaw_d_filter_b, y, index);               
    assignElementsToMembers(csm.state_pitch_dot_d_filter_b, y, index);         
    assignElementsToMembers(csm.state_yaw_dot_d_filter_b, y, index);           
    assignElementsToMembers(csm.e_tran_pos_I, y, index);
    assignElementsToMembers(csm.e_eta_I, y, index);
    assignElementsToMembers(csm.x_tran_ref, y, index);
    assignElementsToMembers(csm.e_tran_pos_ref_I, y, index);
    assignElementsToMembers(csm.K_hat_x_tran, y, index);
    assignElementsToMembers(csm.K_hat_r_tran, y, index);
    assignElementsToMembers(csm.Theta_hat_tran, y, index);
    assignElementsToMembers(csm.e_omega_ref_I, y, index);
    assignElementsToMembers(csm.omega_ref_rot, y, index);
    assignElementsToMembers(csm.K_hat_x_rot, y, index);
    assignElementsToMembers(csm.K_hat_r_rot, y, index);
    assignElementsToMembers(csm.Theta_hat_rot, y, index);
}


// Model function for integration
void mrac_pid::model([[maybe_unused]] const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, [[maybe_unused]] const double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    assignElementsToDxdt(cim.internal_state_roll_d_filter_q, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_d_filter_q, dy, index);
    assignElementsToDxdt(cim.internal_state_roll_dot_d_filter_q, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_dot_d_filter_q, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_d_filter_b, dy, index);
    assignElementsToDxdt(cim.internal_state_yaw_d_filter_b, dy, index);
    assignElementsToDxdt(cim.internal_state_pitch_dot_d_filter_b, dy, index);
    assignElementsToDxdt(cim.internal_state_yaw_dot_d_filter_q, dy, index);
    assignElementsToDxdt(cim.e_tran_pos, dy, index);
    assignElementsToDxdt(cim.e_eta, dy, index);
    assignElementsToDxdt(cim.x_tran_ref_dot, dy, index);
    assignElementsToDxdt(cim.e_tran_pos_ref, dy, index);
    assignElementsToDxdt(cim.K_hat_x_tran_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_r_tran_dot, dy, index);
    assignElementsToDxdt(cim.Theta_hat_tran_dot, dy, index);
    assignElementsToDxdt(cim.e_omega_ref, dy, index);
    assignElementsToDxdt(cim.omega_ref_dot_rot, dy, index);
    assignElementsToDxdt(cim.K_hat_x_rot_dot, dy, index);
    assignElementsToDxdt(cim.K_hat_r_rot_dot, dy, index);
    assignElementsToDxdt(cim.Theta_hat_rot_dot, dy, index);
}


// Function to compute the outerloop regressor vector
void mrac_pid::compute_outer_loop_regressor()
{
    // Cache the variables
    double sph = sin(cim.eta_rot_q(0));
    double cph = cos(cim.eta_rot_q(0));
    double sth = sin(cim.eta_rot_q(1));
    double cth = cos(cim.eta_rot_q(1));
    double sps = sin(cim.eta_rot_q(2));
    double cps = cos(cim.eta_rot_q(2));
    double sa = sin(cim.aero.states.alpha);
    double ca = cos(cim.aero.states.alpha);
    double sb = sin(cim.aero.states.beta);
    double cb = cos(cim.aero.states.beta);
    double v_norm_sq = cim.aero.states.v_norm_sq;

    cim.outer_loop_regressor << -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth),
                                -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth),
                                -v_norm_sq*(sb*(cph*sps - cps*sph*sth) - ca*cb*(sph*sps + cph*cps*sth) + cb*cps*sa*cth),
                                -v_norm_sq*(ca*cb*(cps*sph - cph*sps*sth) - sb*(cph*cps + sph*sps*sth) + cb*sa*cth*sps),
                                                                     v_norm_sq*(cb*sa*sth + sb*cth*sph + ca*cb*cph*cth),
                                -v_norm_sq*(cb*(cph*sps - cps*sph*sth) + ca*sb*(sph*sps + cph*cps*sth) - cb*cps*sa*cth),
                                 v_norm_sq*(cb*(cph*cps + sph*sps*sth) + ca*sb*(cps*sph - cph*sps*sth) + cb*sa*cth*sps),
                                                                    -v_norm_sq*(cb*sa*sth - cb*cth*sph + ca*cph*sb*cth),
                                                                   -v_norm_sq*(sa*(sph*sps + cph*cps*sth) + ca*cps*cth),
                                                                    v_norm_sq*(sa*(cps*sph - cph*sps*sth) - ca*cth*sps),
                                                                                        v_norm_sq*(ca*sth - cph*sa*cth);
}

// Function to compute the inner loop regressor vector
void mrac_pid::compute_innner_loop_regressor()
{
    // Cache the variables
    double sa = sin(cim.aero.states.alpha);
    double ca = cos(cim.aero.states.alpha);
    double sb = sin(cim.aero.states.beta);
    double cb = cos(cim.aero.states.beta);
    double wx = 0.0;
    double wy = 0.0;
    double wz = 0.0;

    if (is_biplane)
    {
        wx = cim.omega_rot_b(0);
        wy = cim.omega_rot_b(1);
        wz = cim.omega_rot_b(2);
    }
    else
    {
        wx = cim.omega_rot_q(0);
        wy = cim.omega_rot_q(1);
        wz = cim.omega_rot_q(2);
    }

    cim.inner_loop_regressor << wx*wy,
                                wz*wx,
                                wy*wx,
                                cim.aero.states.v_norm_sq,
                                cim.aero.states.v_norm_sq*ca,
                                cim.aero.states.v_norm_sq*sb,
                                cim.aero.states.v_norm_sq*cb,
                                cim.aero.states.v_norm_sq*sa*sb,
                                cim.aero.states.v_norm_sq*sa*cb;
}

// Function to compute the outerloop control in I 
void mrac_pid::compute_translational_control_in_I()
{ 
    // Compute the error in the states
    cim.e_tran << cim.x_tran - csm.x_tran_ref;
    cim.e_tran_pos << cim.e_tran.head<3>();
    cim.e_tran_vel << cim.e_tran.tail<3>();

    // Compute the translational position error between the reference model and the user defined trajectory
    cim.e_tran_pos_ref << csm.x_tran_ref.head<3>() - cim.r_user;
    
    // Compute the reference command input [reference model - user_defined_trajectory]
    cim.r_cmd_tran << MASS * (- cip.Ki_refmod_tran * csm.e_tran_pos_ref_I      // Integral term
                              + cip.Kp_refmod_tran * cim.r_user                // Proportional term
                              + cip.Kd_refmod_tran * cim.r_dot_user            // Derivative term
                              + cim.r_ddot_user);                              // Feedforward term

    // Reference model
    cim.x_tran_ref_dot << cip.A_ref_tran * csm.x_tran_ref
                        + cip.B_ref_tran * cim.r_cmd_tran;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS * (- cip.Kp_tran * cim.e_tran_pos                         // Proportional term
                                    - cip.Kd_tran * cim.e_tran_vel                         // Derivative term
                                    - cip.Ki_tran * csm.e_tran_pos_I                       // Integral term
                                    + cim.x_tran_ref_dot.tail<3>()                         // Feedforward term
                                    - G * e3_basis)                                        // Weight inversion term
                                    - cim.aero.dyn.outer_loop_dynamic_inv;                 // Aerodynamic inversion term 

    // Compute the augmented regressor vector
    cim.augmented_outer_loop_regressor << cim.mu_tran_baseline, 
                                          cim.outer_loop_regressor;

    // Cache the transpose of the tracking error * P * B
    Matrix<double, 1, 3> e_transpose_p_b = cim.e_tran.transpose() * cip.P_tran * cip.B_tran;

    // Adaptive laws
    cim.K_hat_x_tran_dot << -cip.Gamma_x_tran * 
                             cim.x_tran * 
                             e_transpose_p_b;

    cim.K_hat_r_tran_dot << -cip.Gamma_r_tran *
                             cim.r_cmd_tran *
                             e_transpose_p_b;

    cim.Theta_hat_tran_dot << cip.Gamma_Theta_tran * 
                              cim.augmented_outer_loop_regressor * 
                              e_transpose_p_b;

    // Adaptive control law
    cim.mu_tran_adaptive << csm.K_hat_x_tran.transpose() * cim.x_tran
                            + csm.K_hat_r_tran.transpose() * cim.r_cmd_tran
                            - csm.Theta_hat_tran.transpose() * cim.augmented_outer_loop_regressor;

    // Compute the total baseline + adaptive control
    cim.mu_tran_I << cim.mu_tran_baseline + cim.mu_tran_adaptive;
}

// Compute the orientation commands and the desired total thrust
void mrac_pid::compute_u1_eta_d()
{
    // Compute the virtual forces in the biplane frame
    cim.mu_tran_Jb << cim.R_I_Jb * cim.mu_tran_I;

    // Compute the virutal forces in the quadcopter frame
    cim.mu_tran_Jq << cim.R_I_Jq * cim.mu_tran_I;

    // compute u1 - Total Thrust in Biplane frame
    double u1_b = sqrt(  pow(cim.mu_tran_Jb(0), 2)
                    + pow(cim.mu_tran_Jb(1), 2)
                    + pow(cim.mu_tran_Jb(2), 2)
                    );

    // Compute u1 - Total Thrust in quadcopter frame
    double u1_q  = sqrt(  pow(cim.mu_tran_Jq(0), 2)
                    + pow(cim.mu_tran_Jq(1), 2)
                    + pow(cim.mu_tran_Jq(2), 2)
                    );
                    
    // Compute the desired roll in biplane frame
    cim.eta_rot_d_b(0) = 0.0;

    // Compute the desired roll in the quadcopter frame
    double r1 = cim.mu_tran_Jq(1) / u1_q;
    cim.eta_rot_d_q(0) = atan2(r1, sqrt(1 - pow(r1,2)));

    // Compute the desired pitch in the biplane frame
    cim.eta_rot_d_b(1) = atan2( cim.mu_tran_Jb(0), cim.mu_tran_Jb(2) );

    // Compute the desired pitch in the quadcopter frame
    cim.eta_rot_d_q(1) = atan2( -cim.mu_tran_Jq(0), -cim.mu_tran_Jq(2) );

    // Compute the desired yaw in the biplane frame 
    double y1 = cim.mu_tran_Jb(1) / u1_b;
    cim.eta_rot_d_b(2) = atan2(y1, sqrt(1 - pow(y1,2)));

    // Compute the desired yaw in quadcopter frame
    cim.eta_rot_d_q(2) = ud.getUserDefinedYaw();

    // Compute the internal states - Roll - Quadcopter only
    cim.internal_state_roll_d_filter_q << A_filter_roll_ref * csm.state_roll_d_filter_q
                                        + B_filter_roll_ref * cim.eta_rot_d_q(0);

    cim.internal_state_roll_dot_d_filter_q << A_filter_roll_dot_ref * csm.state_roll_dot_d_filter_q
                                            + B_filter_roll_dot_ref * cim.eta_rot_rate_d_q(0);

    // Compute the internal states - Pitch
    cim.internal_state_pitch_d_filter_b << A_filter_pitch_ref * csm.state_pitch_d_filter_b
                                        + B_filter_pitch_ref * cim.eta_rot_d_b(1);
    cim.internal_state_pitch_d_filter_q << A_filter_pitch_ref * csm.state_pitch_d_filter_q
                                        + B_filter_pitch_ref * cim.eta_rot_d_q(1);

    cim.internal_state_pitch_dot_d_filter_b << A_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter_b
                                            + B_filter_pitch_dot_ref * cim.eta_rot_rate_d_b(1); 
    cim.internal_state_pitch_dot_d_filter_q << A_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter_q
                                            + B_filter_pitch_dot_ref * cim.eta_rot_rate_d_q(1); 

    // Compute the internal states - Yaw - Biplane only
    cim.internal_state_yaw_d_filter_b << A_filter_yaw_ref * csm.state_yaw_d_filter_b
                                        + B_filter_yaw_ref * cim.eta_rot_d_b(2);

    cim.internal_state_yaw_dot_d_filter_b << A_filter_yaw_dot_ref * csm.state_yaw_dot_d_filter_b
                                            + B_filter_yaw_dot_ref * cim.eta_rot_rate_d_b(2); 

    // Compute the desired Euler rates - Biplane
    cim.eta_rot_rate_d_b(0) = 0.0;
    cim.eta_rot_rate_d_b(1) = C_filter_pitch_ref * csm.state_pitch_d_filter_b;
    cim.eta_rot_rate_d_b(2) = C_filter_yaw_ref * csm.state_yaw_d_filter_b;

    // Compute the desired Euler rates - Quadcopter
    cim.eta_rot_rate_d_q(0) = C_filter_roll_ref * csm.state_roll_d_filter_q;
    cim.eta_rot_rate_d_q(1) = C_filter_pitch_ref * csm.state_pitch_d_filter_q;
    cim.eta_rot_rate_d_q(2) = ud.getUserDefinedYawRate();

    // Compute the desired Euler Acceleration - Biplane
    cim.eta_rot_acceleration_d_b(0) = 0.0;
    cim.eta_rot_acceleration_d_b(1) = C_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter_b;
    cim.eta_rot_acceleration_d_b(2) = C_filter_yaw_dot_ref * csm.state_yaw_dot_d_filter_b;

    // Compute the desired Euler Acceleration - Quadcopter
    cim.eta_rot_acceleration_d_q(0) = C_filter_roll_dot_ref * csm.state_roll_dot_d_filter_q;
    cim.eta_rot_acceleration_d_q(1) = C_filter_pitch_dot_ref * csm.state_pitch_dot_d_filter_q;
    cim.eta_rot_acceleration_d_q(2) = ud.getUserDefinedYawRateDot();

    // Assign according to the mode
    if (is_biplane)
    {
        cim.u(0) = u1_b;
        cim.mu_tran_J = cim.mu_tran_Jb;
        cim.eta_rot_d = cim.eta_rot_d_b;
        cim.eta_rot_rate_d = cim.eta_rot_rate_d_b;
        cim.eta_rot_acceleration_d = cim.eta_rot_acceleration_d_b;
    }
    else
    {
        cim.u(0) = u1_q;
        cim.mu_tran_J = cim.mu_tran_Jq;
        cim.eta_rot_d = cim.eta_rot_d_q;
        cim.eta_rot_rate_d = cim.eta_rot_rate_d_q;
        cim.eta_rot_acceleration_d = cim.eta_rot_acceleration_d_q;
    }
    
}

// Compute the rotational control
void mrac_pid::compute_rotational_control()
{   
    // If you are in biplane mode
    if (is_biplane)
    {   
        // Compute the error in the orientation [actual - desired]
        cim.e_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
        cim.e_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
        cim.e_eta(2) = makeYawAngularErrorContinuous<double>(cim.eta_rot(2), cim.eta_rot_d(2));        

        // Compute the error in the angular velocity
        cim.e_omega << cim.omega_rot - csm.omega_ref_rot;

        // Compute the error in the angular rates
        cim.e_eta_rate << cim.eta_rot_rate - cim.eta_rot_rate_d;

        // Compute command angular velocity
        cim.omega_cmd_rot << cim.Jacobian * (-cip.Kp_omega_cmd_rot_b * cim.e_eta
                                             -cip.Ki_omega_cmd_rot_b * csm.e_eta_I
                                             + cim.eta_rot_rate_d);

        // Compute the derivative of command angular velocity
        cim.omega_cmd_dot_rot << cim.Jacobian_dot * (-cip.Kp_omega_cmd_rot_b * cim.e_eta
                                                     -cip.Ki_omega_cmd_rot_b * csm.e_eta_I
                                                     + cim.eta_rot_rate_d)
                                   + cim.Jacobian * (-cip.Kp_omega_cmd_rot_b * cim.e_eta_rate
                                                     -cip.Ki_omega_cmd_rot_b * cim.e_eta
                                                     + cim.eta_rot_acceleration_d);

        // Compute the angular_velocity_error_ref [refmod - cmd]
        cim.e_omega_ref << csm.omega_ref_rot - cim.omega_cmd_rot;

        // Reference command input
        cim.r_cmd_rot << cip.Kp_omega_ref_rot_b * cim.omega_cmd_rot 
                         -cip.Ki_omega_ref_rot_b * csm.e_omega_ref_I
                         +cim.omega_cmd_dot_rot;

        // Reference model
        cim.omega_ref_dot_rot << -cip.Kp_omega_ref_rot_b * csm.omega_ref_rot + cim.r_cmd_rot;

        // Basline PID controller 
        cim.tau_rot_baseline << cip.inertia_matrix * ( -cip.Kp_rot_b * cim.e_eta               // Proportional term
                                                       -cip.Kd_rot_b * cim.e_omega             // Derivative term
                                                       -cip.Ki_rot_b * csm.e_eta_I             // Integral term
                                                       + cim.omega_ref_dot_rot)                // Feedforward term
                                + cim.omega_rot.cross(cip.inertia_matrix * cim.omega_rot)      // Dynamic inversion term
                                - R_Jq_Jb*cim.aero.dyn.inner_loop_dynamic_inv;                 // Aero inversion term

        // Compute the augmented regressor vector
        cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

        // Cache the transpose of the tracking error * P * B
        Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_omega.transpose() * cip.P_rot_b * cip.B_rot_b; 

        // Adaptive laws
        cim.K_hat_x_rot_dot << -cip.Gamma_x_rot_b *
                                cim.omega_rot * 
                                e_transpose_p_b;

        cim.K_hat_r_rot_dot << -cip.Gamma_r_rot_b *
                                cim.r_cmd_rot * 
                                e_transpose_p_b;

        cim.Theta_hat_rot_dot << cip.Gamma_Theta_rot_b *
                                 cim.augmented_inner_loop_regressor *
                                 e_transpose_p_b;

        // Adaptive control law
        cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega_rot
                                +csm.K_hat_r_rot.transpose() * cim.r_cmd_rot
                                -csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;

        // Total rotational control input
        cim.tau_rot << R_Jb_Jq * (cim.tau_rot_baseline + cim.tau_rot_adaptive);
                            

    }
    // If you are in quadcopter mode
    else
    {
        // Compute the error in the orientation [actual - desired]
        cim.e_eta(0) = cim.eta_rot(0) - cim.eta_rot_d(0);
        cim.e_eta(1) = cim.eta_rot(1) - cim.eta_rot_d(1);
        cim.e_eta(2) = makeYawAngularErrorContinuous<double>(cim.eta_rot(2), cim.eta_rot_d(2));        

        // Compute the error in the angular velocity
        cim.e_omega << cim.omega_rot - csm.omega_ref_rot;

        // Compute the error in the angular rates
        cim.e_eta_rate << cim.eta_rot_rate - cim.eta_rot_rate_d;

        // Compute command angular velocity
        cim.omega_cmd_rot << cim.Jacobian * (-cip.Kp_omega_cmd_rot_q * cim.e_eta
                                             -cip.Ki_omega_cmd_rot_q * csm.e_eta_I
                                             + cim.eta_rot_rate_d);

        // Compute the derivative of command angular velocity
        cim.omega_cmd_dot_rot << cim.Jacobian_dot * (-cip.Kp_omega_cmd_rot_q * cim.e_eta
                                                     -cip.Ki_omega_cmd_rot_q * csm.e_eta_I
                                                     + cim.eta_rot_rate_d)
                                   + cim.Jacobian * (-cip.Kp_omega_cmd_rot_q * cim.e_eta_rate
                                                     -cip.Ki_omega_cmd_rot_q * cim.e_eta
                                                     + cim.eta_rot_acceleration_d);

        // Compute the angular_velocity_error_ref [refmod - cmd]
        cim.e_omega_ref << csm.omega_ref_rot - cim.omega_cmd_rot;

        // Reference command input
        cim.r_cmd_rot << cip.Kp_omega_ref_rot_q * cim.omega_cmd_rot
                         -cip.Ki_omega_ref_rot_q * csm.e_omega_ref_I
                         +cim.omega_cmd_dot_rot;

        // Reference model
        cim.omega_ref_dot_rot << -cip.Kp_omega_ref_rot_q * csm.omega_ref_rot + cim.r_cmd_rot;

        // Basline PID controller 
        cim.tau_rot_baseline << cip.inertia_matrix * ( -cip.Kp_rot_q * cim.e_eta               // Proportional term
                                                       -cip.Kd_rot_q * cim.e_omega             // Derivative term
                                                       -cip.Ki_rot_q * csm.e_eta_I             // Integral term
                                                       + cim.omega_ref_dot_rot)                // Feedforward term
                                + cim.omega_rot.cross(cip.inertia_matrix * cim.omega_rot)      // Dynamic inversion term
                                - cim.aero.dyn.inner_loop_dynamic_inv;                         // Aero inversion term

        // Compute the augmented regressor vector
        cim.augmented_inner_loop_regressor << cim.tau_rot_baseline, cim.inner_loop_regressor;

        // Cache the transpose of the tracking error * P * B
        Eigen::Matrix<double, 1, 3> e_transpose_p_b = cim.e_omega.transpose() * cip.P_rot_q * cip.B_rot_q; 

        // Adaptive laws
        cim.K_hat_x_rot_dot << -cip.Gamma_x_rot_q *
                                cim.omega_rot * 
                                e_transpose_p_b;

        cim.K_hat_r_rot_dot << -cip.Gamma_r_rot_q *
                                cim.r_cmd_rot * 
                                e_transpose_p_b;

        cim.Theta_hat_rot_dot << cip.Gamma_Theta_rot_q *
                                 cim.augmented_inner_loop_regressor *
                                 e_transpose_p_b;

        // Adaptive control law
        cim.tau_rot_adaptive << csm.K_hat_x_rot.transpose() * cim.omega_rot
                                +csm.K_hat_r_rot.transpose() * cim.r_cmd_rot
                                -csm.Theta_hat_rot.transpose() * cim.augmented_inner_loop_regressor;

         // Total rotational control input
        cim.tau_rot << cim.tau_rot_baseline + cim.tau_rot_adaptive;                                

    }

    // Assign the control inputs
    cim.u(1) = cim.tau_rot(0);
    cim.u(2) = cim.tau_rot(1);
    cim.u(3) = cim.tau_rot(2);

}

// Function to compute the normalized thrusts
void mrac_pid::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_qrbp * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
    control_input(1) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
    control_input(2) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
    control_input(3) = evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));
}

// Function to print to terminal during runtime
void mrac_pid::debug2terminal()
{
    // FLIGHTSTACK_INFO_STREAM_NO_TAG("Execution Time:", cim.alg_duration);  
}

// Function that is called in control.cpp
void mrac_pid::run(const double time_step_rk4_) {
    
    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics 
    cim.aero.dyn = compute_aero_forces_moments(cim.aero.states, cim.aero.coeff, cim.R_Jq_I, cim.R_W_Jq);

    // 2. Compute the regressor vector for the outer loop
    compute_outer_loop_regressor();

    // 3. Compute the translational control input
    compute_translational_control_in_I();

    // 4. Compute the thrust needed and the desired angles
    compute_u1_eta_d();
   
    // 5. Compute the regressor vector for the inner loop
    compute_innner_loop_regressor();
    
    // 6. Compute the rotational control input
    compute_rotational_control();

    // 7. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 8. Do the integration
    rk4.do_step(boost::bind(&mrac_pid::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 9. Log the Data after all the calculataions
    logger.logLogData();

    // optional debug
    debug2terminal();
}

// Implementing getter functions from controller_base
float mrac_pid::get_t1() const {
    return control_input(0);
}

float mrac_pid::get_t2() const {
    return control_input(1);
}

float mrac_pid::get_t3() const {
    return control_input(2);
}

float mrac_pid::get_t4() const {
    return control_input(3);
}

float mrac_pid::get_t5() const {
    return control_input(4);
}

float mrac_pid::get_t6() const {
    return control_input(5);
}

float mrac_pid::get_t7() const {
    return control_input(6);
}

float mrac_pid::get_t8() const {
    return control_input(7);
}



} // namespace _mrac_pid_
} // namespace _qrbp_