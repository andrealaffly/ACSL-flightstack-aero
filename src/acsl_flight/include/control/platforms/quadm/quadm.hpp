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
 * File:        quadm.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 10, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vehicle Information of the quadm with the T-Motor
 *              P2305 kv2550 motors used in all the control algorithms
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-flight-stack.git
 **********************************************************************************************************************/

/*               _           _      _                      
 $$$$$$\  $$\   $$\  $$$$$$\  $$$$$$$\  $$\      $$\ 
$$  __$$\ $$ |  $$ |$$  __$$\ $$  __$$\ $$$\    $$$ |
$$ /  $$ |$$ |  $$ |$$ /  $$ |$$ |  $$ |$$$$\  $$$$ |
$$ |  $$ |$$ |  $$ |$$$$$$$$ |$$ |  $$ |$$\$$\$$ $$ |
$$ |  $$ |$$ |  $$ |$$  __$$ |$$ |  $$ |$$ \$$$  $$ |
$$ $$\$$ |$$ |  $$ |$$ |  $$ |$$ |  $$ |$$ |\$  /$$ |
\$$$$$$ / \$$$$$$  |$$ |  $$ |$$$$$$$  |$$ | \_/ $$ |
 \___$$$\  \______/ \__|  \__|\_______/ \__|     \__|
     \___|                                           
*/

#ifndef QUADM_HPP_
#define QUADM_HPP_

#include <Eigen/Dense>           // Include the Eigen library
#include <cmath>
#include <math.h>

namespace _quadm_{
        
    // Constants
    inline constexpr double G = 9.81;
    inline constexpr double RHO_HAT = 1.225;
    inline constexpr double PI = 3.1415;
    inline constexpr double PIHALF = (PI/2);
    inline constexpr double DEG2RAD = (PI/180);
    inline constexpr double RAD2DEG = (180/PI);

    // Vehicle and Environment Defines
    inline constexpr double MASS = 1.28941154;  // mass of vehicle                      [Kg]                                      
    inline constexpr double LX = 0.09667602;    // dist to motor along x^J              [m]
    inline constexpr double LY = 0.11074982;    // dist to motor along y^J              [m]

    // Thrust Normalizeing polynomials are hardcoded!
    // Take care coding this from matlab. check the documentation for the polyval function.
    // The polynomails are reversed!.
    inline constexpr double CT_MOTOR =  0.1017;                                    // motor torque coeff              [-]
    inline constexpr double MAX_THRUST = 18;                                       // max allowed thrust per motor    [N]
    inline constexpr double MIN_THRUST = 0.3;                                      // min allowed thrust per motor    [N]

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
    const Eigen::VectorXd thrust_polynomial_coeff_quadm = (Eigen::VectorXd(8) << 
                                                            0.000000030316084940690649384320061064321,
                                                           -0.000001814468657920738862350290739045,
                                                            0.000042434512948142268497307011410058,
                                                           -0.00050487431926448940282259325584846,
                                                            0.0035249290604740459061094970394379,
                                                           -0.017552333963129614080589391278409,
                                                            0.1149654034760465154407782506496,
                                                            0.011001286689392602777259888569006
                                                        ).finished();

    // Weight vector of the qrbp
    static inline const Eigen::Vector3d e3_basis = Eigen::Vector3d(0.0, 0.0, 1.0);

    // Mixer Matrix ----------------------------------------------------------------------------------------------------
    // This captures the necessary constants within the lambda function and initializes the
    // mixer_matrix_quadcopter with the resulting matrix.
    // [1/4, -1/(4*l_y),  1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y), -1/(4*l_x),  1/(4*c_t)]
    // [1/4,  1/(4*l_y),  1/(4*l_x), -1/(4*c_t)]
    // [1/4, -1/(4*l_y), -1/(4*l_x), -1/(4*c_t)]
    const Eigen::Matrix4d mixer_matrix_quadm = []() {
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



    // Matrix of inertia of the quadcopter frame -----------------------------------------------------------------------
    // [kg*m^2] inertia matrix of the vehicle system (drone frame + box + propellers) expressed in
    // Pixhawk coordinate system (FRD - x-Front, y-Right, z-Down), computed at the vehicle center of mass
    const Eigen::Matrix3d inertia_matrix_q = (Eigen::Matrix3d() << 
                                                0.00677586,	0.00000121, 0.00000914,
                                                0.00000121,	0.00639639,	0.00000215,
                                                0.00000914,	0.00000215,	0.01061757).finished();                                             

} // namespace _quadm_

#endif  // QUADM_HPP_