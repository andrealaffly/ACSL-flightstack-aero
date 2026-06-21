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
 * Description: Header file for piecewise polynomial trajectory. The code
 *              to generate this was written in Matlab by Matti called
 *              minimum jerk trajectory. Core functionality was written
 *              by Matti and modified by Giri to fit acsl_flight.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_
#define PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_

#include "user_defined_trajectory_base.hpp" // Include for the base trajectory class
#include "flight_params.hpp"                // Include for the flight params
#include "global_helpers.hpp"               // Include for the flightstack global functions

using namespace _user_defined_trajectory_base_;

// Use the namespace _flightstack_ for the global functions
using namespace _flightstack_;

namespace _piecewise_polynomial_trajectory_{

class piecewise_polynomial_trajectory : public trajectory_base
{
    public:
        // Constructor
        piecewise_polynomial_trajectory(flight_params* p);

        // Function to update the user defined trajectory according to the controller time
        void updateUserDefinedTrajectory(double controller_time);

    private: 
        /// Instance of the flight_params_ptr to point to the flight_parameter object in
        /// flight_main.cpp
        flight_params* flight_params_ptr;

        // Function that updates the user-defined yaw, yaw_dot, and yaw_dot_dot based on the time at which 
        // it is requested to be evaluated
        void updateUserDefinedYaw();

        // Compute the user-defined yaw from the trajectory velocity in the XY plane
        double yawComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef, double t);

        // Compute the user-defined yaw_dot from the trajectory velocity in the XY plane
        double yawDotComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef,
                                 const VectorXd& Ax_coef, const VectorXd& Ay_coef, 
                                 double t);

        // Compute the user-defined yaw_dot_dot from the trajectory velocity in the XY plane
        double yawDotDotComputation(const VectorXd& Vx_coef, const VectorXd& Vy_coef,
                                    const VectorXd& Ax_coef, const VectorXd& Ay_coef,
                                    const VectorXd& Jx_coef, const VectorXd& Jy_coef,
                                    double t);


        // Function to extract and set the polynomial coefficient matrices
        void setPolynomialCoefficientMatrices();

        // Function to adjust the time to be fed to the various polynomials and identify the segment of the trajectory
        std::pair<double, int> polynomialTimeAdjusted(double time);

        // Function to extract the polynomial coeffiecients
        std::vector<Eigen::MatrixXd> polyCoefAssigning(const std::vector<std::vector<double>>& poly_coeff_matrix_in);

        // Internal time for the piecewise polynomail trajectory
        double internal_time;

        // current time adjusted with respect to the trajectory segment
        double internal_time_adjusted; 

        // segment of the trajectory in which I am. If the trajectory has 5 waypoints, it has 4 segments
        int segment_; 

        // user_defined_position_ computed during the previous iteration
        Vector3d user_defined_position_previous_; 

        // user_defined_yaw_ computed during the previous iteration
        double user_defined_yaw_previous_; 
        
        // norm of the X and Y components of the user_defined_velocity_
        double norm_velocity_XY_; 

        // Polynomial coefficients
        MatrixXd position_coef_x_;
        MatrixXd position_coef_y_;
        MatrixXd position_coef_z_;
        MatrixXd velocity_coef_x_;
        MatrixXd velocity_coef_y_;
        MatrixXd velocity_coef_z_;
        MatrixXd acceleration_coef_x_;
        MatrixXd acceleration_coef_y_;
        MatrixXd acceleration_coef_z_;
        MatrixXd jerk_coef_x_;
        MatrixXd jerk_coef_y_;
        MatrixXd jerk_coef_z_;

        // Position polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
        const VectorXd landing_position_coef_z_ = (VectorXd(8) << 
            -0.001220703125,
             0.01708984375,
            -0.0820312499999999,
             0.13671875,
             0.0,
             0.0,
             0.0,
            -1.0
        ).finished();

        // Velocity polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
        const VectorXd landing_velocity_coef_z_ = (VectorXd(7) << 
            -0.00854492187499999,
             0.1025390625,
            -0.41015625,
             0.546874999999999,
             0.0,
             0.0,
             0.0
        ).finished();

        // Acceleration polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
        const VectorXd landing_acceleration_coef_z_ = (VectorXd(6) << 
            -0.0512695312499999,
             0.5126953125,
            -1.640625,
             1.640625,
             0.0,
             0.0
        ).finished();
};

} // namespace _piecewise_polynomial_trajectory_



#endif // PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_  