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
 * File:        PID_OMEGA.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 25, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID with angular veocities for the QRBP. Inherits 
 *              controller_base class for the basic functionality 
 *              that is to be used for all control algorithms.
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

#ifndef CONTROLLERS_PID_OMEGA_HPP_
#define CONTROLLERS_PID_OMEGA_HPP_

#include "control_algorithm_base.hpp"       // Include for the base class for the basis of control algorithms
#include "PID_OMEGA_members.hpp"            // Include for all the members of the PID class
#include "PID_OMEGA_logger.hpp"             // Include for the logging.

#include "piecewise_polynomial_trajectory.hpp" // Include for the piecwise polynomial trajectory

using namespace _control_algorithm_base_;
using namespace _piecewise_polynomial_trajectory_;

// Define the number of states in the boost array for integration
#define NSI 16

namespace _qrbp_{
namespace _pid_omega_{

class pid_omega : public controller_base
{
    public:
        // Constructor
        pid_omega(flight_params* p, const std::string & controller_log_dir_);

        // Destructor 
        ~pid_omega();

        // Implementing functions from controller_base
        void run(const double time_step_rk4_);
        void update(double time, 
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
                    double w_z);

        // Implementing getter functions from controller_base
        float get_t1() const;
        float get_t2() const;
        float get_t3() const;
        float get_t4() const;
        float get_t5() const;
        float get_t6() const;
        float get_t7() const;
        float get_t8() const;

    private:
        // Implementing functions from controller_base
        void read_params(const std::string& jsonFile); 
        void init();

        // Piecewise polynomial trajectory object
        piecewise_polynomial_trajectory ud;

        // Initialize the logger
        pid_omega_logger logger;

    private:
        // Define the state vector
        rk4_array<double, NSI> y;
        rk4_array<double, NSI> dy;

        // Define the model
        void model(const rk4_array<double, NSI> &y, rk4_array<double, NSI> &dy, double t);

        // Create a RungeKutta object.
        boost::numeric::odeint::runge_kutta4<rk4_array<double, NSI>> rk4;

        // Define the internal parameter members of the controller 
        controller_internal_parameters cip;

        // Define the internal members of the controller
        controller_internal_members cim;   

        // Define the internal integrated state members of the controller
        controller_integrated_state_members csm; 

    private:
        // Function to compute the translational control in the inertial frame
        void compute_translational_control_in_I();

        // Function tot compute the thrust (u1) and the desired angles
        void compute_u1_eta_d();

        // Function to compute the rotational control input
        void compute_rotational_control();

        // Function to compute the normalized thrust
        void compute_normalized_thrusts();

        // Assign the values from rk4 to controller internal members
        void assign_from_rk4();

        // Function to print to terminal
        void debug2terminal();
};

} // namespace _pid_omega_
} // namespace _qrbp_

#endif  // CONTROLLERS_PID_OMEGA_HPP_