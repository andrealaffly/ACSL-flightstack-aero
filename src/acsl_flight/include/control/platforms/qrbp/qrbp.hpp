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
 * File:        qrbp.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        May 29, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vehicle Information of the QRBP with the NACA 0012 wings 
 *              and the Velox V2808 kv1300 motors used in all the control
 *              algorithms
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

/* NOTE: Removed physical mounting functionality for T265 on vehicle. Therefore, those parameters are not provided in 
         the Mass and inertia properties. Can be added later on if needed with changes to the compile time defines. */

#ifndef QRBP_HPP_
#define QRBP_HPP_

#include "helper_functions.hpp"     // Include the helper functions for using the rotation matrices


/*********************************************************************************************************************
  PLATFORM component defines
**********************************************************************************************************************
*/
#define ZEDF9P 1  // Set to 0 if ZED-F9P is not attached


/*********************************************************************************************************************
  PLATFORM property defines
**********************************************************************************************************************
*/
namespace _qrbp_{
    // ZED-F9P attached -------------------------------------------------------------------- Mass and inertia properties
    #if ZEDF9P
    
    inline constexpr double MASS = 1.68817748;  // mass with ZED-F9P    [Kg]

    // Matrix of inertia of the quadcopter frame - With ZED-F9P attached
    // [kg*m^2] inertia matrix of the vehicle system expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
                                                 0.02353227, -0.00000011,  0.00047910,
                                                -0.00000011,  0.01623841, -0.00000128,
                                                 0.00047910, -0.00000128,  0.02753728).finished();

    // Matrix of inertia of the biplane frame - With ZED-F9P attached
    // [kg*m^2] inertia matrix of the vehicle system expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_b = (Eigen::Matrix3d() << 
                                                 0.02753728,  0.00000128, -0.00047910,
                                                 0.00000128,  0.01623841, -0.00000011,
                                                -0.00047910, -0.00000011,  0.02353227).finished();       
    
    #else
    // ZED-F9P not attached ---------------------------------------------------------------- Mass and inertia properties
    
    inline constexpr double MASS = 1.66037326;   // mass without ZED-F9P [Kg]

    // Matrix of inertia of the quadcopter frame - With no T265 and ZED-F9P attached
    // [kg*m^2] inertia matrix of the vehicle system expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
                                                 0.02322746, -0.00000025,  0.00065038,
                                                -0.00000025,  0.01583236, -0.00000103,
                                                 0.00065038, -0.00000103,  0.02741269).finished();

    // Matrix of inertia of the biplane frame - With no T265 and ZED-F9P attached
    // [kg*m^2] inertia matrix of the vehicle system expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_b = (Eigen::Matrix3d() << 
                                                 0.02741269,  0.00000104, -0.00065038
                                                 0.00000104,  0.01583236, -0.00000025
                                                -0.00065038, -0.00000025,  0.02322746).finished();

    #endif
    
    // Constants -------------------------------------------------------------------------------------------------------
    inline constexpr double G = 9.81;
    inline constexpr double RHO_HAT = 1.225;
    inline constexpr double PI = 3.1415;
    inline constexpr double PIHALF = (PI/2);
    inline constexpr double DEG2RAD = (PI/180);
    inline constexpr double RAD2DEG = (180/PI);
    inline constexpr double SQRT2_OVER_2 = 0.7071067;

    // Vehicle and Environment Defines
    inline constexpr double LX = 0.097509;                                        // dist to motor along x^J         [m]
    inline constexpr double LY = 0.110688;                                        // dist to motor along y^J         [m]
    inline constexpr double LZ_S = 0.038779;                                      // dist to aero center of stabs    [m]
    inline constexpr double SPAN_W = 0.50;                                        // span of wings                   [m]
    inline constexpr double CHORD_W = 0.12;                                       // chord of wings                  [m]
    inline constexpr double SPAN_S = 0.175;                                       // span of stabs                   [m]
    inline constexpr double CHORD_S = 0.02;                                       // chord of stabs                  [m]
    inline constexpr double PLANFORM_AREA_W = (2.0 * SPAN_W * CHORD_S);           // planform area of wings        [m^2]
    inline constexpr double PLANFORM_AREA_S = (2.0 * SPAN_S * CHORD_S);           // planform area of stabs        [m^2]
    
    // Aerodynamic Dynamic Coefficient Defines
    inline constexpr double DYN_PRESS_COEFF_W = (0.5 * RHO_HAT * PLANFORM_AREA_W);     
    inline constexpr double DYN_PRESS_COEFF_S = (0.5 * RHO_HAT * PLANFORM_AREA_S);

    // Weight vector of the qrbp
    static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

    // Thrust Normalizeing polynomials are hardcoded!
    // Take care coding this from matlab. check the documentation for the polyval function.
    // The polynomails are reversed!.
    inline constexpr double CT_MOTOR =  0.1017;                                   // motor torque coeff              [-]
    inline constexpr double MAX_THRUST = 18;                                      // max allowed thrust per motor    [N]
    inline constexpr double MIN_THRUST = 0.3;                                     // min allowed thrust per motor    [N]

    // Switch over angle of QRBP
    inline constexpr double AoS = 70*DEG2RAD;

    // vairbales for switching mechanism
    inline bool is_biplane;        // Boolean to tell the controller if it is in biplane mode
    inline bool momentary_button;  // Boolean to tell the controller to reset conditions

    // Matrix that rotates from quadcopter to biplane mode
    const Eigen::Matrix3d R_Jq_Jb = (Eigen::Matrix3d() << 
                                        0.0, 0.0, 1.0,
                                        0.0, 1.0, 0.0,
                                       -1.0, 0.0, 0.0).finished();

    // Matrix that rotates from biplane to quadcopter mode
    const Eigen::Matrix3d R_Jb_Jq = (Eigen::Matrix3d() <<
                                        0.0, 0.0, -1.0,
                                        0.0, 1.0,  0.0,
                                        1.0, 0.0,  0.0).finished();

    // Quaternion that denotes a pitch of 90 degress about an axis and it's inverse
    const Eigen::Quaterniond PITCH_90 = Eigen::Quaterniond(cos(90*DEG2RAD/2.0), 0.0, sin(90*DEG2RAD/2.0), 0.0);
    const Eigen::Quaterniond PITCH_90_INV = PITCH_90.inverse();

    // Roll Rate Filter ------------------------------------------------------------------------------------------------
    // A matrix of the roll_ref filter
    const Eigen::Matrix2d A_filter_roll_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0 
                                                ).finished(); 

    // B matrix of the roll_ref filter
    const Eigen::Vector2d B_filter_roll_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the roll_ref filter
    const Eigen::RowVector2d C_filter_roll_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the roll_ref filter
    const double D_filter_roll_ref = 0.0; 

    // Pitch Rate Filter -----------------------------------------------------------------------------------------------
    // A matrix of the pitch_ref filter
    const Eigen::Matrix2d A_filter_pitch_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the pitch_ref filter
    const Eigen::Vector2d B_filter_pitch_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the pitch_ref filter
    const Eigen::RowVector2d C_filter_pitch_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the pitch_ref filter
    const double D_filter_pitch_ref = 0.0; 
    
    // Yaw Rate Filter -------------------------------------------------------------------------------------------------
    // A matrix of the yaw_ref filter
    const Eigen::Matrix2d A_filter_yaw_ref = (Eigen::Matrix2d() << 
                                                    -72.0, -1600.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the yaw_ref filter
    const Eigen::Vector2d B_filter_yaw_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the yaw_ref filter
    const Eigen::RowVector2d C_filter_yaw_ref = Eigen::RowVector2d(1600.0, 0); 

    // D matrix of the yaw_ref filter
    const double D_filter_yaw_ref = 0.0; 

    // Roll Acceleration Filter ----------------------------------------------------------------------------------------
    // A matrix of the roll_dot_ref filter
    const Eigen::Matrix2d A_filter_roll_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0 
                                                ).finished(); 

    // B matrix of the roll_dot_ref filter
    const Eigen::Vector2d B_filter_roll_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the roll_dot_ref filter
    const Eigen::RowVector2d C_filter_roll_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the roll_dot_ref filter
    const double D_filter_roll_dot_ref = 0.0; 

    // Pitch Acceleration Filter ---------------------------------------------------------------------------------------
    // A matrix of the pitch_ref filter
    const Eigen::Matrix2d A_filter_pitch_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the pitch_ref filter
    const Eigen::Vector2d B_filter_pitch_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the pitch_ref filter
    const Eigen::RowVector2d C_filter_pitch_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the pitch_ref filter
    const double D_filter_pitch_dot_ref = 0.0; 

    // Yaw Acceleration Filter -----------------------------------------------------------------------------------------
    // A matrix of the yaw_ref filter
    const Eigen::Matrix2d A_filter_yaw_dot_ref = (Eigen::Matrix2d() << 
                                                    -50.0, -2500.0, 
                                                      1.0,    0.0
                                                ).finished(); 

    // B matrix of the yaw_ref filter
    const Eigen::Vector2d B_filter_yaw_dot_ref = Eigen::Vector2d(1.0, 0); 

    // C matrix of the yaw_ref filter
    const Eigen::RowVector2d C_filter_yaw_dot_ref = Eigen::RowVector2d(0.15*2500.0, 0); 

    // D matrix of the yaw_ref filter
    const double D_filter_yaw_dot_ref = 0.0; 
    
    // Thrust Polynomial -----------------------------------------------------------------------------------------------
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    const Eigen::VectorXd thrust_polynomial_coeff_qrbp = (Eigen::VectorXd(8) << 
                                                            0.000000030316084940690649384320061064321,
                                                           -0.000001814468657920738862350290739045,
                                                            0.000042434512948142268497307011410058,
                                                           -0.00050487431926448940282259325584846,
                                                            0.0035249290604740459061094970394379,
                                                           -0.017552333963129614080589391278409,
                                                            0.1149654034760465154407782506496,
                                                            0.011001286689392602777259888569006
                                                        ).finished();
    
    // Mixer Matrix ----------------------------------------------------------------------------------------------------
    // This captures the necessary constants within the lambda function and initializes the mixer_matrix_quadcopter with 
    // the resulting matrix.
    // [1/4, -1/(4*l_y),  1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y), -1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y),  1/(4*l_x), -1/(4*c_t)]
    // [1/4, -1/(4*l_y), -1/(4*l_x), -1/(4*c_t)]
    const Eigen::Matrix4d mixer_matrix_qrbp = []() {
        Eigen::Matrix4d mat; // 4 rows, 4 columns
            
        // Assign values element-by-element
        mat(0, 0) =  1.0 / 4.0;
        mat(0, 1) = -1.0 / (4.0 * LY);
        mat(0, 2) =  1.0 / (4.0 * LX);
        mat(0, 3) =  1.0 / (4.0 * CT_MOTOR); 

        mat(1, 0) =  1.0 / 4.0;
        mat(1, 1) =  1.0 / (4.0 * LY);
        mat(1, 2) = -1.0 / (4.0 * LX);
        mat(1, 3) =  1.0 / (4.0 * CT_MOTOR); 

        mat(2, 0) =  1.0 / 4.0;
        mat(2, 1) =  1.0 / (4.0 * LY);
        mat(2, 2) =  1.0 / (4.0 * LX);
        mat(2, 3) = -1.0 / (4.0 * CT_MOTOR); 

        mat(3, 0) =  1.0 / 4.0;
        mat(3, 1) = -1.0 / (4.0 * LY);
        mat(3, 2) = -1.0 / (4.0 * LX);
        mat(3, 3) = -1.0 / (4.0 * CT_MOTOR); 

        return mat; 
    }();

    /********************************************************************************************************************/
    /*                                         HELPER FUNCTIONS FOR BIPLANE MODE                                        */
    /********************************************************************************************************************/

    // Inline function to compute the switch from Quadcopter mode to Biplane Mode and vice versa
    inline void QRBPswitchFun(double pitch, bool& is_biplane, bool& mnt_btn)
    {
        // Uncomment based on your requirement

        // Code to switch in both directions of flight.
        // double abs_pitch = abs(pitch);

        // if (abs_pitch >= AoS && !is_biplane)
        // {
        //     is_biplane = true;
        //     mnt_btn = true;
        // }
        // else if (abs_pitch < AoS && is_biplane)
        // {
        //     is_biplane = false;
        //     mnt_btn = true;
        // }
        // else
        // {
        //     mnt_btn = false;
        // }

        // Code to switch in forward direction of flight.
        if (pitch <= -AoS && !is_biplane)
        {
            is_biplane = true;
            mnt_btn = true;
        }
        else if (pitch > -AoS && is_biplane)
        {
            is_biplane = false;
            mnt_btn = true;
        }
        else
        {
            mnt_btn = false;
        }
    }

    // Inline Function to update the biplane mode states - Goes in the inner loop bridge.
    // Need to modify. This function is written assuming that we are only going in forward flight in a straight line!!!
    inline void update_states_to_biplane_mode(const Vector3d eta,const Vector3d omega, 
                                              Vector3d& eta_b, Vector3d& omega_b) 
    {

        // Compute the right hand side of the equation
        Eigen::Matrix<double, 3, 3>  RHS = _utilities_::rotationMatrix321LocalToGlobal(eta(0), eta(1), eta(2))*R_Jq_Jb;

        // Matlab code to follow
        // if RHS(3,1) == 1
        //     theta_B = -pi/2;
        //     psi_B = atan2(-RHS(2,3),-RHS(1,3));
        //     phi_B = 0;
        // elseif RHS(3,1) == -1
        //     theta_B = pi/2;
        //     psi_B = atan2(RHS(1,2),RHS(2,2));
        //     phi_B = 0;
        // else
        //     theta_B(2,1) = atan2(-RHS(3,1),sqrt(1-RHS(3,1)^2));
        //     theta_B(1,1) = atan2(-RHS(3,1),-sqrt(1-RHS(3,1)^2));
            
        //     phi_B(2,1) = atan2(RHS(3,2),RHS(3,3));
        //     phi_B(1,1) = atan2(-RHS(3,2),-RHS(3,3));
            
        //     psi_B(2,1) = atan2(RHS(2,1),RHS(1,1));
        //     psi_B(1,1) = atan2(-RHS(2,1),-RHS(1,1));
        // end

        // Some vectors and temprory variables
        Vector2d phi_b, theta_b, psi_b;             // Resultant angle pair for biplane
        
        double abs0, abs1;                          // Temp variables for caluclating the smallest rotation
        double rhs20 = RHS(2, 0);                   // caching variable
        double rhs20_sqrt = sqrt(1 - rhs20*rhs20);  // caching variable
        double rhs21 = RHS(2,1);                    // caching variable
        double rhs22 = RHS(2,2);                    // caching variable
        double rhs10 = RHS(1,0);                    // caching variable
        double rhs00 = RHS(0,0);                    // caching variable

        if (abs(rhs20) != 1)
        {
            /// The state we usually will be in
            // Find the pitch for biplane
            theta_b(0)  = atan2( -rhs20,  rhs20_sqrt );
            theta_b(1)  = atan2( -rhs20, -rhs20_sqrt );
            abs0 = std::fabs(theta_b(0));
            abs1 = std::fabs(theta_b(1));
            eta_b(1) = (abs0 < abs1) ? theta_b(0) : theta_b(1);

            // Find the roll for biplane
            phi_b(0) = atan2(  rhs21,  rhs22 );
            phi_b(1) = atan2( -rhs21, -rhs22 );
            abs0 = std::fabs(phi_b(0));
            abs1 = std::fabs(phi_b(1));
            eta_b(0) = (abs0 < abs1) ? phi_b(0) : phi_b(1);

            // Find the yaw for biplane
            psi_b(0) = atan2(  rhs10,  rhs00 );
            psi_b(1) = atan2( -rhs10, -rhs00 );

            abs0 = std::fabs(psi_b(0));
            abs1 = std::fabs(psi_b(1));

            eta_b(2) = (abs0 < abs1) ? psi_b(0) : psi_b(1);
        }
        // Conditions for if we are hovering and if we are pointing the propeller's axis to the ground. 
        // This is not going to happen and therfore the if statements are put under another if statement 
        // To save compute
        else
        {
            eta_b(1) = (rhs20 == 1) ? -PIHALF : PIHALF;
            eta_b(2) = (rhs20 == 1) ? atan2(-RHS(1,2), -RHS(0,2)) : atan2(RHS(0,1), RHS(1,1));
            eta_b(0) = 0.0;
        }

        // Compute the angular velocities in the biplane frame
        omega_b = R_Jq_Jb * omega;
    }

    // Code written by Giri Mugundan Kumar to switch states - yet to be debugged
    inline void update_states_to_biplane_quaternion_gmk(const Eigen::Quaterniond q_q, const Vector3d omega,
                                                        Vector3d& eta_b, Vector3d& omega_b)
    {
        // Pitch 90 degrees about the y axis to get the new quaternion in biplane frame
        Eigen::Quaterniond q_b = q_q*PITCH_90_INV;

        // Compute elements of Direction Cosine Matrix (DCM) from quaternion
        double a = q_b.w();
        double b = q_b.x();
        double c = q_b.y();
        double d = q_b.z();
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
        eta_b(1) = asin(-dcm20); // Pitch

        // Handle gimbal lock cases
        if (fabs(eta_b(1) - M_PI / 2) < 1.0e-3) {
            eta_b(0) = 0.0; // Roll
            eta_b(2) = atan2(dcm12, dcm02); // Yaw
        } else if (fabs(eta_b(1) + M_PI / 2) < 1.0e-3) {
            eta_b(0) = 0.0; // Roll
            eta_b(2) = atan2(-dcm12, -dcm02); // Yaw
        } else {
            eta_b(0) = atan2(dcm21, dcm22); // Roll
            eta_b(2) = atan2(dcm10, dcm00); // Yaw
        }

        // Compute the angular velocities in the biplane frame
        omega_b = R_Jq_Jb * omega;

    }

    // Code from the PX4 Repository to switch states - might need debugging
    inline void update_states_to_biplane_quaternions(const Eigen::Quaterniond q_q, const Vector3d omega, 
                                                     Vector3d& eta_b, Vector3d& omega_b)
    {
        // Modified from pixhawk code that constructs a dcm matrix from quaternion     
        const double a = q_q.w();
        const double b = q_q.x();
        const double c = q_q.y();
        const double d = q_q.z();
        const double aa = a * a;
        const double ab = a * b;
        const double ac = a * c;
        const double ad = a * d;
        const double bb = b * b;
        const double bc = b * c;
        const double bd = b * d;
        const double cc = c * c;
        const double cd = c * d;
        const double dd = d * d;

        // Construct the dcm rotation matrix according to the pixhawk code.
        Eigen::Matrix<double, 3, 3> dcm;
        dcm(0, 0) = aa + bb - cc - dd;
        dcm(0, 1) = 2 * (bc - ad);
        dcm(0, 2) = 2 * (ac + bd);
        dcm(1, 0) = 2 * (bc + ad);
        dcm(1, 1) = aa - bb + cc - dd;
        dcm(1, 2) = 2 * (cd - ab);
        dcm(2, 0) = 2 * (bd - ac);
        dcm(2, 1) = 2 * (ab + cd);
        dcm(2, 2) = aa - bb - cc + dd;

        // Vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
        // Since the VTOL airframe is initialized as a multicopter we need to modify the 
        // estimated attitude fro the fixed wing operation. Since the neutral position of
        // the vehicle in fixed wing mode is -90 degrees rotated around the pitch axis 
        // compared to the neutral position of the vecicle in multicopter mode we need to 
        // swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
        // Additionally, in order to get the correct sign of the pitch, we need to multiply
        // the new x axis of the rotation matrix with -1. 
        
        /*
        * original:			modified:
        *
        * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
        * Rxy  Ryy  Rzy	    -Rzy  Ryy  Rxy
        * Rxz  Ryz  Rzz	    -Rzz  Ryz  Rxz
        * */

        Eigen::Matrix<double, 3,3> dcm_adapted = dcm;        // modified rotation matrix
        /* move z to x */
        dcm_adapted(0, 0) = dcm(0, 2);
        dcm_adapted(1, 0) = dcm(1, 2);
        dcm_adapted(2, 0) = dcm(2, 2);

        /* move x to z */
        dcm_adapted(0, 2) = dcm(0, 0);
        dcm_adapted(1, 2) = dcm(1, 0);
        dcm_adapted(2, 2) = dcm(2, 0);

        /* change direction of pitch (convert to right handed system) */
        dcm_adapted(0, 0) = -dcm_adapted(0, 0);
        dcm_adapted(1, 0) = -dcm_adapted(1, 0);
        dcm_adapted(2, 0) = -dcm_adapted(2, 0);

        // Now reconstruct the euler angles from the dcm_adapted matrix.
        
        // Theta
        double theta = std::asin(-dcm_adapted(2,0));
        double roll;
        double yaw;

        if ((std::fabs(theta - M_PI_2)) < 1.0e-3)
        {
            roll = 0.0;
            yaw = std::atan2(dcm_adapted(1,2), dcm_adapted(0,2));
        }
        else if ((std::fabs(theta + M_PI_2)) < 1.0e-3)
        {
            roll = 0.0;
            yaw = std::atan2(-dcm_adapted(1,2), -dcm_adapted(0,2));
        } 
        else
        {
            roll = std::atan2(dcm_adapted(2,1), dcm_adapted(2,2));
            yaw = std::atan2(dcm_adapted(1,0), dcm_adapted(0,0));
        }
        

        // Assign pitch
        eta_b(1) = theta;

        // Assign roll
        if (roll > M_PI_2) { eta_b(0) = roll - M_PI; }
        else if (roll < -M_PI_2) { eta_b(0) = roll + M_PI; }
        else { eta_b(0) = roll; }
        
        // Assign yaw
        if (yaw > M_PI_2) { eta_b(2) = yaw - M_PI; }
        else if (yaw < -M_PI_2) { eta_b(2) = yaw + M_PI; }
        else { eta_b(2) = yaw; }

        // Compute the angular velocities in the biplane frame
        omega_b = R_Jq_Jb * omega;

    }                                                     


} // namespace _qrbp_

#endif  // QRBP_HPP_
