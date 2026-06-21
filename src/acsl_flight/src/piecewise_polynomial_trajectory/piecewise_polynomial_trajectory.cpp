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
 * File:        piecewise_polynomial_trajectory.hpp
 * Author:      Mattia Gramuglia, Giri Mugundan Kumar
 * Date:        June 27, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation of the piecewise polynomial trajectory. The 
 *              code to generate this was written in Matlab by Matti called
 *              minimum jerk trajectory. Core functionality was written
 *              by Matti and modified by Giri to fit acsl_flight.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "piecewise_polynomial_trajectory.hpp"

namespace _piecewise_polynomial_trajectory_{

// Constructor
piecewise_polynomial_trajectory::piecewise_polynomial_trajectory(flight_params* p) : 
flight_params_ptr(p),
internal_time(0.0),
internal_time_adjusted(0.0),
segment_(0),
user_defined_position_previous_(Vector3d::Zero()),
user_defined_yaw_previous_(0.0),
norm_velocity_XY_(0.0)
{
    // Setup the polynomial coefficeient matrices
    setPolynomialCoefficientMatrices();   
}

// Function to assign polynomial coefficients to designated variables
std::vector<Eigen::MatrixXd> piecewise_polynomial_trajectory::polyCoefAssigning
(const std::vector<std::vector<double>>& poly_coeff_matrix_in) 
{
    // Get the size of the input matrix
    int rows = poly_coeff_matrix_in.size();
    int cols = poly_coeff_matrix_in[0].size();
    
    // Calculate the size of the output matrices
    int out_rows = rows / 3;
    
    // Initialize output matrices for x, y, and z variables
    Eigen::MatrixXd poly_coeff_matrix_out_x(out_rows, cols);
    Eigen::MatrixXd poly_coeff_matrix_out_y(out_rows, cols);
    Eigen::MatrixXd poly_coeff_matrix_out_z(out_rows, cols);
    
    // Assign polynomial coefficients to designated variables
    for (int i = 0; i < out_rows; ++i) {
        for (int j = 0; j < cols; ++j) {
        poly_coeff_matrix_out_x(i, j) = poly_coeff_matrix_in[3 * i][j];
        poly_coeff_matrix_out_y(i, j) = poly_coeff_matrix_in[3 * i + 1][j];
        poly_coeff_matrix_out_z(i, j) = poly_coeff_matrix_in[3 * i + 2][j];
        }
    }
    
    // Return the output matrices
    return {poly_coeff_matrix_out_x, poly_coeff_matrix_out_y, poly_coeff_matrix_out_z};
}

// Function to extract and set the polynomial coefficient matrices
void piecewise_polynomial_trajectory::setPolynomialCoefficientMatrices()
{
    const auto& pp_coefficients = flight_params_ptr->traj_params.piecewise_polynomial_coefficients_;
    
    // Call polyCoefAssigning to assign polynomial coefficients to designated variables
    auto position_coefs = polyCoefAssigning(pp_coefficients);

    // Extract coefficients for x, y, and z components
    position_coef_x_ = position_coefs[0];
    position_coef_y_ = position_coefs[1];
    position_coef_z_ = position_coefs[2];

    // Compute velocity coefficients
    velocity_coef_x_ = polyDerMatrix(position_coef_x_);
    velocity_coef_y_ = polyDerMatrix(position_coef_y_);
    velocity_coef_z_ = polyDerMatrix(position_coef_z_);

    // Compute acceleration coefficients
    acceleration_coef_x_ = polyDerMatrix(velocity_coef_x_);
    acceleration_coef_y_ = polyDerMatrix(velocity_coef_y_);
    acceleration_coef_z_ = polyDerMatrix(velocity_coef_z_);

    // Compute jerk coefficients
    jerk_coef_x_ = polyDerMatrix(acceleration_coef_x_);
    jerk_coef_y_ = polyDerMatrix(acceleration_coef_y_);
    jerk_coef_z_ = polyDerMatrix(acceleration_coef_z_);

    // Debug Message
    // FLIGHTSTACK_INFO("Flying with position coeffs: ");
    // FLIGHTSTACK_INFO_MATRIX("coefficients x: ", position_coef_x_);
    // FLIGHTSTACK_INFO_MATRIX("coefficients Y: ", position_coef_y_);
    // FLIGHTSTACK_INFO_MATRIX("coefficients Z: ", position_coef_z_);
}

// Adjusts the time to be fed to the various polynomials and identifies the segment of the trajectory
std::pair<double, int> piecewise_polynomial_trajectory::polynomialTimeAdjusted(double time)
{
    int segment = 0;
    double time_adjusted = 0.0;

    for (int i = 0; i < flight_params_ptr->traj_params.num_waypts - 1; ++ i)
    {
        if (time >= flight_params_ptr->traj_params.waypoint_times_[i] 
            && time <= flight_params_ptr->traj_params.waypoint_times_[i + 1])
        {
            segment = i;
            time_adjusted = time - flight_params_ptr->traj_params.waypoint_times_[segment];
            return {time_adjusted, segment};        
        }
    }

    return {time_adjusted, segment};
}


// Function that updates the user-defined trajectory based on the time at which it is requested to be evaluated
void piecewise_polynomial_trajectory::updateUserDefinedTrajectory(double controller_time)
{
    // Adjust the current controller time so that the piecewise polynomial trajectory starts at 0s. 
    // The coefffiecnts are generated in matlab assuming we start at 0.
    internal_time = controller_time - flight_params_ptr->start_time;

    // Adjust time and identify segment
    // NOTE: If the current time has passed the last waypoint time, then segment_ = 0;
    std::tie(internal_time_adjusted, segment_) = polynomialTimeAdjusted(internal_time);

    // If the internal time has passed the last waypoint time but it is lesser than the landing start time,
    // Set the trajectory to perform HOVER in the last position of the trajectory
    if(internal_time >= flight_params_ptr->traj_params.waypoint_times_.back() && 
        controller_time < flight_params_ptr->landing_start_time)
    {
        setUserDefinedPosition(user_defined_position_previous_);
        setUserDefinedVelocity(Vector3d::Zero());
        setUserDefinedAcceleration(Vector3d::Zero());
    }
    // If the controller_time has passed the landing start time but not the landing end time,
    // perform the LANDING maneuver
    else if (controller_time >= flight_params_ptr->landing_start_time && 
                controller_time < flight_params_ptr->landing_end_time)
    {
        setUserDefinedPosition(Vector3d(user_defined_position_previous_(0),
                                        user_defined_position_previous_(1),
                                        evaluatePolynomial(landing_position_coef_z_,
                                        controller_time - flight_params_ptr->landing_start_time )));

        // Evaluate and update user_defined_velocity_
        setUserDefinedVelocity(Vector3d(0.0,
                                        0.0,
                                        evaluatePolynomial(landing_velocity_coef_z_,
                                        controller_time - flight_params_ptr->landing_start_time )));

        // Evaluate and update user_defined_acceleration_
        setUserDefinedAcceleration(Vector3d(0.0,
                                            0.0,
                                            evaluatePolynomial(landing_acceleration_coef_z_,
                                            controller_time - flight_params_ptr->landing_start_time )));
    }    
    // If the current time has passed the landing end time, stay at the origin       
    else if (controller_time >= flight_params_ptr->landing_end_time)
    {
        setUserDefinedPosition(user_defined_position_previous_);
        setUserDefinedVelocity(Vector3d::Zero());
        setUserDefinedAcceleration(Vector3d::Zero());
    }
    // If any of the above conditions are not met, follow the user defined trajectory
    else 
    {
        // Evaluate and update user_defined_position_
        setUserDefinedPosition(Vector3d(
        evaluatePolynomial(position_coef_x_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(position_coef_y_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(position_coef_z_.row(segment_), internal_time_adjusted)));

        // Evaluate and update user_defined_velocity_
        setUserDefinedVelocity(Vector3d(
        evaluatePolynomial(velocity_coef_x_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(velocity_coef_y_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(velocity_coef_z_.row(segment_), internal_time_adjusted)));

        // Evaluate and update user_defined_acceleration_
        setUserDefinedAcceleration(Vector3d(
        evaluatePolynomial(acceleration_coef_x_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(acceleration_coef_y_.row(segment_), internal_time_adjusted),
        evaluatePolynomial(acceleration_coef_z_.row(segment_), internal_time_adjusted)));
    }

    // Update user_defined_position_previous_
    user_defined_position_previous_ = getUserDefinedPosition();

    // Update the user defined yaw
    updateUserDefinedYaw();
}

// Function that updates the user-defined yaw, yaw_dot, and yaw_dot_dot based on the time at which it is requested 
// to be evaluated
void piecewise_polynomial_trajectory::updateUserDefinedYaw()
{
    // Compute the 2D norm of the trajectory velocity in the XY plane
    norm_velocity_XY_ = norm2D<Eigen::VectorXd, double>(velocity_coef_x_.row(segment_),
                                                        velocity_coef_y_.row(segment_),
                                                        internal_time_adjusted);

    // If the norm of the velocity in the XY plane is close to zero OR If the current time has passed the
    // last waypoint time, THEN set the trajectory to keep the last yaw angle
    if (this->norm_velocity_XY_ < 1e-3 || (internal_time >= flight_params_ptr->traj_params.waypoint_times_.back()))
    {
        setUserDefinedYaw(user_defined_yaw_previous_);
        setUserDefinedYawRate(0.0);
        setUserDefinedYawRateDot(0.0);
    }
    else
    {
        // Update the User defined yaw angle
        setUserDefinedYaw(yawComputation(velocity_coef_x_.row(segment_),
                                         velocity_coef_y_.row(segment_),
                                         internal_time_adjusted)); 
        
        // Update the first derivative of the user-defined yaw angle
        setUserDefinedYawRate(yawDotComputation(velocity_coef_x_.row(segment_),
                                                velocity_coef_y_.row(segment_),
                                                acceleration_coef_x_.row(segment_),
                                                acceleration_coef_y_.row(segment_),
                                                internal_time_adjusted));
        
        // Update the second derivative of the user-defined yaw angle
        setUserDefinedYawRateDot(yawDotDotComputation(velocity_coef_x_.row(segment_),
                                                      velocity_coef_y_.row(segment_),
                                                      acceleration_coef_x_.row(segment_),
                                                      acceleration_coef_y_.row(segment_),
                                                      jerk_coef_x_.row(segment_),
                                                      jerk_coef_y_.row(segment_),
                                                      internal_time_adjusted));

    }

    // Update user_defined_yaw_previous_
    user_defined_yaw_previous_ = getUserDefinedYaw();
}

// Compute the user-defined yaw from the trajectory velocity in the XY plane
double piecewise_polynomial_trajectory::yawComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef, double t)
{
    // Compute the velocity components Vx and Vy at time t using polynomial coefficients
    double Vx = evaluatePolynomial(Vx_coef, t);
    double Vy = evaluatePolynomial(Vy_coef, t);

    // Compute the yaw angle using atan2
    double yaw = std::atan2(Vy, Vx);

    return yaw;
}

// Compute the user-defined yaw_dot from the trajectory velocity in the XY plane
double piecewise_polynomial_trajectory:: yawDotComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef,
                                                           const VectorXd& Ax_coef, const VectorXd& Ay_coef, 
                                                           double t)
{
    // Compute the acceleration components Ax and Ay at time t using polynomial coefficients
    double Ax = evaluatePolynomial(Ax_coef, t);
    double Ay = evaluatePolynomial(Ay_coef, t);

    // Compute the acceleration angle in the complex plane
    double acceleration_angle = std::atan2(Ay, Ax);

    // Compute the velocity and acceleration norms
    double velocity_norm = norm2D<Eigen::VectorXd, double>(Vx_coef, Vy_coef, t);
    double acceleration_norm = norm2D<Eigen::VectorXd, double>(Ax_coef, Ay_coef, t);

    // Compute the yaw angle
    double yaw = yawComputation(Vx_coef, Vy_coef, t);

    // Compute the yaw rate
    double yaw_dot = (acceleration_norm / velocity_norm) * std::sin(acceleration_angle - yaw);

    return yaw_dot;
}

// Compute the user-defined yaw_dot_dot from the trajectory velocity in the XY plane
double piecewise_polynomial_trajectory::yawDotDotComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef,
                                                             const VectorXd& Ax_coef, const VectorXd& Ay_coef,
                                                             const VectorXd& Jx_coef, const VectorXd& Jy_coef,
                                                             double t)
{
    // Compute the jerk components Jx and Jy at time t using polynomial coefficients
    double Jx = evaluatePolynomial(Jx_coef, t);
    double Jy = evaluatePolynomial(Jy_coef, t);

    // Compute the jerk angle in the complex plane
    double jerk_angle = std::atan2(Jy, Jx);

    // Compute the velocity and jerk norms
    double velocity_norm = norm2D<Eigen::VectorXd, double>(Vx_coef, Vy_coef, t);
    double jerk_norm = norm2D<Eigen::VectorXd, double>(Jx_coef, Jy_coef, t);

    // Compute the derivative of the velocity norm
    double velocity_norm_prime = norm2Dderivative<Eigen::VectorXd, double>(Vx_coef, Vy_coef, Ax_coef, Ay_coef, t);

    // Compute the yaw angle and its derivative
    double yaw = yawComputation(Vx_coef, Vy_coef, t);
    double yaw_dot = yawDotComputation(Vx_coef, Vy_coef, Ax_coef, Ay_coef, t);

    // Compute the second derivative of yaw angle
    double yaw_dot_dot = (jerk_norm * std::sin(jerk_angle - yaw) - 
                        2 * velocity_norm_prime * yaw_dot) / velocity_norm;

    return yaw_dot_dot;
}


} // namespace _piecewise_polynomial_trajectory_