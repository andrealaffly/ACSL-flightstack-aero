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
 * File:        aero_qrbp.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 8, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Aerodynamics class for all the algorithms to use with the qrbp platform 
 *              of ACSL.
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

#ifndef AERO_QRBP_
#define AERO_QRBP_

#include "qrbp.hpp"

namespace _qrbp_{

// Angles for the wings and stabilizers
struct AeroStates {
    double v_norm_sq; // Square of the norm of velocity
    double alpha;     // Angle of attack at the center of mass
    double alpha_up;  // Angle of attack of the upper wing
    double alpha_lw;  // Angle of attack of the lower wing
    double beta;      // Side slip angle at the center of mass
    double beta_lt;   // Side slip angle for the left stabilizer
    double beta_rt;   // Side slip angle for the right stabilizer
};

// Aerodynamic coefficients for the QRBP
struct AeroCoeff {
  // Coefficients for lift of the wings and stabilizers
  double Cl_up;     // Aerodynamic lift coefficient for upper wing
  double Cl_lw;     // Aerodynamic lift coefficient for lower wing
  double Cl_lt;     // Aerodynamic lift coefficient for left stabilizer
  double Cl_rt;     // Aerodynamic lift coefficient for right stabilizer

// Coeffficient for drag of the wings and stabilizers
  double Cd_up;     // Aerodynamic drag coefficient for upper wing
  double Cd_lw;     // Aerodynamic drag coefficient for lower wing
  double Cd_lt;     // Aerodynamic drag coefficient for left stabilizer
  double Cd_rt;     // Aerodynamic drag coefficient for right stabilizer

// Coefficient for moment of the wings and stabilizers
  double Cm_up;     // Aerodynamic moment coefficient for upper wing
  double Cm_lw;     // Aerodynamic moment coefficient for lower wing
  double Cm_lt;     // Aerodynamic moment coefficient for left stabilizer
  double Cm_rt;     // Aerodynamic moment coefficient for right stabilizer
};

// Structure for all the aerodynamic members of the controller
struct AeroDynamicMembers
{
  Eigen::Matrix<double, 3,1> F_aero;                   // Aerodynamic Forces
  Eigen::Matrix<double, 3,1> outer_loop_dynamic_inv;   // Aero Baseline dynamic inversion - outer loop
  Eigen::Matrix<double, 3,1> M_aero;                   // Aerodynamic Moments
  Eigen::Matrix<double, 3,1> inner_loop_dynamic_inv;   // Aero Baseline dynamic inversion - inner loop
};


// Inline function to compute the Aerodynamic angles for the NACA-0012 airfoil
inline AeroStates computeAeroStatesNACA0012Quad(const double u, const double v, const double w, 
                                                const double w_x, const double w_y, const double w_z)
{
    AeroStates states;
    
    // Precompute common expressions
    double u_sq = u * u;
    double v_sq = v * v;
    double w_sq = w * w;
    
    double LY_wz = LY * w_z;
    double LY_wx = LY * w_x;

    // Compute intermediate values
    double c2 = -(w - LX * w_y);
    double c3 = -(w + LX * w_y);
    double c4 = sqrt(u_sq + w_sq);
    double c5 = sqrt((u + LY_wz) * (u + LY_wz) + (w - LY_wx) * (w - LY_wx));
    double c6 = sqrt((u - LY_wz) * (u - LY_wz) + (w + LY_wx) * (w + LY_wx));

    double n1 = c4;
    double n2 = sqrt(u_sq + c2 * c2);
    double n3 = sqrt(u_sq + c3 * c3);
    double n4 = sqrt(v * v + c4 * c4);
    double n5 = sqrt(v * v + c5 * c5);
    double n6 = sqrt(v * v + c6 * c6);

    // Compute the angle of attack at the center of mass
    states.alpha = (n1 <= 0.01f) ? 0.0f : atan2(u, -w);

    // Compute the angle of attack of the upper wing
    states.alpha_up = (n2 <= 0.01f) ? 0.0f : atan2(u, c2);

    // Compute the angle of attack of the lower wing
    states.alpha_lw = (n3 <= 0.01f) ? 0.0f : atan2(u, c3);

    // Compute the side slip angle at the center of mass
    states.beta = (n4 <= 0.01f) ? 0.0f : atan2(v, c4);

    // Compute the side slip angle for the left stabilizer
    states.beta_lt = (n5 <= 0.01f) ? 0.0f : atan2(v, c5);

    // Compute the side slip angle for the right stabilizer
    states.beta_rt = (n6 <= 0.01f) ? 0.0f : atan2(v, c6);

    // Compute the square of the norm of the velocity
    states.v_norm_sq = u_sq + v_sq + w_sq;

    return states;
}

// Inline function to compute the coefficient of lift for the NACA-0012 airfoil using polynomial approximations
inline double compute_cl(double angle)
{
    using Eigen::Matrix;

    static const struct {
        double min_angle;
        double max_angle;
        Matrix<double, 3, 1> coeffs;
    } angle_coeff_map[] = {
        {-18 * DEG2RAD, -15 * DEG2RAD, Matrix<double, 3, 1>(-2.016727473, -0.040687912, 0.0)},
        {-15 * DEG2RAD,  -7 * DEG2RAD, Matrix<double, 3, 1>(-0.339043494,  0.073185027, 0.0)},
        { -7 * DEG2RAD,   7 * DEG2RAD, Matrix<double, 3, 1>( 0.0,          0.113305266, 0.0)},
        {  7 * DEG2RAD,  15 * DEG2RAD, Matrix<double, 3, 1>( 0.344930022,  0.073338904, 0.0)},
        { 15 * DEG2RAD,  17 * DEG2RAD, Matrix<double, 3, 1>( 2.007250549, -0.39995604,  0.0)},
        { 17 * DEG2RAD,  27 * DEG2RAD, Matrix<double, 3, 1>( 4.98,        -0.378,       0.0087)},
        { 27 * DEG2RAD,  95 * DEG2RAD, Matrix<double, 3, 1>( 0.43,         0.0303,     -0.000382)},
        { 95 * DEG2RAD, 155 * DEG2RAD, Matrix<double, 3, 1>( 8.91,        -0.143,       0.000519)},
        {155 * DEG2RAD, 180 * DEG2RAD, Matrix<double, 3, 1>(139,          -1.67,        0.00499)},
        {-27 * DEG2RAD, -18 * DEG2RAD, Matrix<double, 3, 1>(-4.98,        -0.378,      -0.0087)},
        {-95 * DEG2RAD, -27 * DEG2RAD, Matrix<double, 3, 1>(-0.43,         0.0303,      0.000382)},
        {-155* DEG2RAD, -95 * DEG2RAD, Matrix<double, 3, 1>(-8.91,        -0.143,      -0.000519)},
        {-180* DEG2RAD,-155* DEG2RAD, Matrix<double, 3, 1>(-139,         -1.67,       -0.00499)},
    };

    for (const auto& entry : angle_coeff_map) {
        if (angle >= entry.min_angle && angle < entry.max_angle) {
            return entry.coeffs(0) + entry.coeffs(1) * angle + entry.coeffs(2) * angle * angle;
        }
    }

    return 0.0; // Default return if no range matches
}

// Inline function to compute the coefficient of drag for the NACA-0012 airfoil using polynomial approximations
inline double compute_cd(double angle)
{
    using Eigen::Matrix;

    static const struct {
        double min_angle;
        double max_angle;
        Matrix<double, 3, 1> coeffs;
    } angle_coeff_map[] = {
        {-18 * DEG2RAD,  18 * DEG2RAD, Matrix<double, 3, 1>( 0.004705434,  0.0,        0.000199101)},
        { 18 * DEG2RAD, 180 * DEG2RAD, Matrix<double, 3, 1>(-0.82,         0.0574,    -0.000319)},
        {-180* DEG2RAD, -18 * DEG2RAD, Matrix<double, 3, 1>(-0.82,        -0.0574,    -0.000319)},
    };

    for (const auto& entry : angle_coeff_map) {
        if (angle >= entry.min_angle && angle < entry.max_angle) {
            return entry.coeffs(0) + entry.coeffs(1) * angle + entry.coeffs(2) * angle * angle;
        }
    }

    return 0.0; // Default return if no range matches
}

// Inline function to compute the coefficient of aerodynamic moment for the NACA-0012 airfoil using polynomial 
// approximations
inline double compute_cm(double angle)
{
    using Eigen::Matrix;

    static const struct {
        double min_angle;
        double max_angle;
        Matrix<double, 2, 1> coeffs;
    } angle_coeff_map[] = {
        {-18 * DEG2RAD, -15 * DEG2RAD,  Matrix<double, 2, 1>(-0.154623077, -0.007923077)},
        {-15 * DEG2RAD,  -6.75*DEG2RAD, Matrix<double, 2, 1>( 0.043910573,  0.004934576)},
        { -6.75*DEG2RAD,  -4 * DEG2RAD, Matrix<double, 2, 1>(-0.029656624, -0.005647643)},
        { -4 * DEG2RAD,   4 * DEG2RAD, Matrix<double, 2, 1>( 0.0,          0.001556935)},
        {  4 * DEG2RAD,   7 * DEG2RAD, Matrix<double, 2, 1>( 0.027566825, -0.005293839)},
        {  7 * DEG2RAD,  15 * DEG2RAD, Matrix<double, 2, 1>(-0.044215508,  0.004953476)},
        { 15 * DEG2RAD,  18 * DEG2RAD, Matrix<double, 2, 1>( 0.168665618, -0.008760839)},
        {-180* DEG2RAD, -18 * DEG2RAD, Matrix<double, 2, 1>(-0.3,          0.0)}, // Default case
        { 18 * DEG2RAD,  180* DEG2RAD, Matrix<double, 2, 1>(-0.3,          0.0)}, // Default case
    };

    for (const auto& entry : angle_coeff_map) {
        if (angle >= entry.min_angle && angle < entry.max_angle) {
            return entry.coeffs(0) + entry.coeffs(1) * angle;
        }
    }

    return 0.0; // Default return if no range matches
}

// Inline function to compute the aerodynamic coeffiecients for the QRBP
inline AeroCoeff compute_aero_coefficients(AeroStates states)
{
  AeroCoeff coeff;

  coeff.Cl_up = compute_cl(states.alpha_up);
  coeff.Cl_lw = compute_cl(states.alpha_lw);
  coeff.Cl_lt = compute_cl(states.beta_lt);
  coeff.Cl_rt = compute_cl(states.beta_rt);

  coeff.Cd_up = compute_cd(states.alpha_up);
  coeff.Cd_lw = compute_cd(states.alpha_lw);
  coeff.Cd_lt = compute_cd(states.beta_lt);
  coeff.Cd_rt = compute_cd(states.beta_rt);

  coeff.Cm_up = compute_cm(states.alpha_up);
  coeff.Cm_lw = compute_cm(states.alpha_lw);
  coeff.Cm_lt = compute_cm(states.beta_lt);
  coeff.Cm_rt = compute_cm(states.beta_rt);

  return coeff;
}

// Incine function to comput the aerodynamic and moments for the QRBP
inline AeroDynamicMembers compute_aero_forces_moments(AeroStates states, AeroCoeff coeff, Eigen::Matrix3d R_J_I, Eigen::Matrix3d R_W_J)
{
  AeroDynamicMembers output;

  // Cache the dynamic pressure of the wings and stabilizers
  double dynamic_pressure_wings = DYN_PRESS_COEFF_W*states.v_norm_sq;
  double dynamic_pressure_stabs = DYN_PRESS_COEFF_S*states.v_norm_sq;

  // Compute the Aerodynamic forces
  output.F_aero(0) =  -1*(dynamic_pressure_wings*(coeff.Cl_up + coeff.Cl_lw));
  output.F_aero(1) = -1*(dynamic_pressure_wings*(coeff.Cd_up + coeff.Cd_lw)
                      + dynamic_pressure_stabs*(coeff.Cd_lt  + coeff.Cd_rt));
  output.F_aero(2) = -1*(dynamic_pressure_stabs*(coeff.Cl_lt + coeff.Cl_rt));

  // Assign it to dynamic inversion
  output.outer_loop_dynamic_inv << R_J_I * R_W_J * output.F_aero;

  // cache the cosine and the sine values of alpha and beta at C.O.M
  double calpha = cos(states.alpha);
  double salpha = sin(states.alpha);
  double cbeta = cos(states.beta);
  double sbeta = sin(states.beta);

  // Compute the Aerodynamic moments
  output.M_aero(0) = dynamic_pressure_wings*CHORD_W*(coeff.Cm_lt + coeff.Cm_rt)
                     - 2*LZ_S*(output.F_aero(1)*cbeta + output.F_aero(0)*sbeta);

  output.M_aero(1) = dynamic_pressure_wings*CHORD_W*(coeff.Cm_lw + coeff.Cm_up)
                     - 2*LZ_S*(output.F_aero(2)*calpha + output.F_aero(0)*cbeta*salpha
                               - output.F_aero(1)*salpha*sbeta);

  output.M_aero(2) = 0.0;

  // Assign it to the dynamic inversion
  output.inner_loop_dynamic_inv = output.M_aero;

  // Return the output 
  return output;

}

} // namespace _qrbp_

#endif // AERO_QRBP_