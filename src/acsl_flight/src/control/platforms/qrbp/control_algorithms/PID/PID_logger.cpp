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
 * File:        PID_logger.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 26, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: logging for the PID controller. Inherits the Blackbox class
 *              for logging and takes in the internal members for the PID.
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

#include <PID_logger.hpp>

namespace _qrbp_{
namespace _pid_{

pid_logger::pid_logger(controller_internal_members* cim, controller_integrated_state_members* csm, 
                       Eigen::Vector<float, 8>* control_input, const std::string & controller_log_dir_) : 
                       CIM(cim), CSM(csm), cntrl_input(control_input),
                       flight_run_log_directory(controller_log_dir_)
{
    // Initilize the logging from blackbox class.
    if(logInitLogging()) {FLIGHTSTACK_INFO("pid Logger Created");};
}

// Implementing virtual functions from Blackbox
void pid_logger::logInitHeaders() {
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
        << "Phi [rad], "
        << "Theta [rad], "
        << "Psi [rad], "
        << "Phi rate [rad/s], "
        << "Theta rate [rad/s], "
        << "Psi rate [rad/s], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m], "
        << "User Velocity y [m], "
        << "User Velocity z [m], "
        << "Desired Phi [rad], "
        << "Desired Theta [rad], "
        << "Desired Psi [rad], "
        << "Deisred Phi rate [rad/s], "
        << "Deisred Theta rate [rad/s], "
        << "Deisred Psi rate [rad/s], "
        << "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Integral Error in x [m], "
        << "Integral Error in y [m], "
        << "Integral Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Error in Phi [rad], "
        << "Error in Theta [rad], "
        << "Error in Psi [rad], "
        << "Integral Error in Phi [rad], "
        << "Integral Error in Theta [rad], "
        << "Integral Error in Psi [rad], "
        << "Error in Phi rate [rad/s], "
        << "Error in Theta rete [rad/s], "
        << "Error in Psi rate [rad/s], "
        << "Virtual Mu x I [N], "
        << "Virtual Mu y I [N], "
        << "Virtual Mu z I [N], "
        << "Virtual Mu x J [N], "
        << "Virtual Mu y J [N], "
        << "Virtual Mu z J [N], "
        << "u1 [N], "
        << "u2 [Nm], "
        << "u3 [Nm], "
        << "u4 [Nm], "
        << "Thrust Motor 1 [N], "
        << "Thrust Motor 2 [N], "
        << "Thrust Motor 3 [N], "
        << "Thrust Motor 4 [N], "
        << "Normalized Thrust Motor 1 [-], "
        << "Normalized Thrust Motor 2 [-], "
        << "Normalized Thrust Motor 3 [-], "
        << "Normalized Thrust Motor 4 [-], "
        << "Desired Phi dot dot [rad/s^2], "
        << "Desired Theta dot dot [rad/s^2], "
        << "Desired Psi dot dot [rad/s^2], "
        ;     


    BOOST_LOG(logger_logdata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool pid_logger::logInitLogging() {
    try {
        // Create the directory path for the controller specific flight run logs
        std::string flight_run_controller_specific_directory = flight_run_log_directory + "/" + "PID";

        // Check if the directory exits, and create it if it doesn't
        if (!std::filesystem::exists(flight_run_controller_specific_directory))
        {
            std::filesystem::create_directories(flight_run_controller_specific_directory);
        }

        // Generate the log file name with a timestamp
        std::stringstream log_ss;
        log_ss << flight_run_controller_specific_directory << "/controller_log" << ".log";
        std::string log_filename = log_ss.str();

        // Add the "Tag" attribute with a constant value of "LogDataTag" to the logger
        logger_logdata.add_attribute("Tag", attrs::constant<std::string>("LogDataTag"));

        // Define a synchronous sink with a text ostream backend
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> logdata_sink = boost::make_shared<text_sink>();

        // Add a stream to the backend (in this case, a file stream)
        logdata_sink->locked_backend()->add_stream(boost::make_shared<std::ofstream>(log_filename));

        // Set the formatter for the sink
        logdata_sink->set_formatter(
            expr::stream
            << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] " // Format date and time
            << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] " // Current thread ID
            << "[" << expr::attr<std::string>("Tag") << "] " // Tag attribute value
            << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] " // Current process ID
            << "[" << expr::attr<unsigned int>("LineID") << "] " // Line ID
            << expr::smessage // Log message
        );

        // Add the sink to the logging core
        logging::core::get()->add_sink(logdata_sink);

        // Set a filter for the sink
        logdata_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "LogDataTag");

        logging::add_common_attributes(); // Add attributes like timestamp

        // Initilaize the logging headers
        logInitHeaders();

        // Create the directory path for the controller specific flight run for parameters and gains used
        std::string flight_run_controller_specific_parameter_directory = flight_run_controller_specific_directory + "/" + "params";

        // Check if the direcotry exits, and create it if it doesn't
        if (!std::filesystem::exists(flight_run_controller_specific_parameter_directory))
        {
            std::filesystem::create_directories(flight_run_controller_specific_parameter_directory);
        }

        // Copy the flight parameters over
        std::string params_source_file = "./src/acsl_flight/params/flight_params/flight_main_params.json";
        std::stringstream params_target_ss;
        params_target_ss << flight_run_controller_specific_parameter_directory << "/flight_main_params.json" ;
        std::string params_target_file = params_target_ss.str();

        if (!std::filesystem::exists(params_source_file)) {
            FLIGHTSTACK_ERROR("MAIN PARAMS FILE NOT PRESENT");
        }

        if (std::filesystem::exists(params_source_file)) {
            std::filesystem::copy(params_source_file, params_target_file);
        }

        // Copy the gains parameters over
        std::string gains_source_file = "./src/acsl_flight/params/control_algorithms/qrbp/PID/gains_PID.json";
        std::stringstream gains_target_ss;
        gains_target_ss << flight_run_controller_specific_parameter_directory << "/gains_PID.json" ;
        std::string gains_target_file = gains_target_ss.str();

        if (!std::filesystem::exists(gains_source_file)) {
            FLIGHTSTACK_ERROR("GAINS FILE NOT PRESENT");
        }

        if (std::filesystem::exists(gains_source_file)) {
            std::filesystem::copy(gains_source_file, gains_target_file);
        }

        // Return true if all successful
        return true;       

    } catch (const std::filesystem::filesystem_error& e) {
        FLIGHTSTACK_ERROR("Filesystem error:", e.what());
    } catch (const std::exception& e) {
        FLIGHTSTACK_ERROR("Exeption:", e.what());
    } 

    return false; // IF there are errors
}

// Implementing virtual functions from Blackbox
void pid_logger::logLogData() {
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << CIM->t << ", "
        << CIM->alg_duration << ", "
        << CIM->x_tran_pos(0) << ", "
        << CIM->x_tran_pos(1) << ", "
        << CIM->x_tran_pos(2) << ", "
        << CIM->x_tran_vel(0) << ", "
        << CIM->x_tran_vel(1) << ", "
        << CIM->x_tran_vel(2) << ", "
        << CIM->eta_rot(0) << ", "
        << CIM->eta_rot(1) << ", "
        << CIM->eta_rot(2) << ", "
        << CIM->eta_rot_rate(0) << ", "
        << CIM->eta_rot_rate(1) << ", "
        << CIM->eta_rot_rate(2) << ", "
        << CIM->r_user(0) << ", "
        << CIM->r_user(1) << ", "
        << CIM->r_user(2) << ", "
        << CIM->r_dot_user(0) << ", "
        << CIM->r_dot_user(1) << ", "
        << CIM->r_dot_user(2) << ", "
        << CIM->eta_rot_d(0) << ", "
        << CIM->eta_rot_d(1) << ", "
        << CIM->eta_rot_d(2) << ", "
        << CIM->eta_rot_rate_d(0) << ", "
        << CIM->eta_rot_rate_d(1) << ", "
        << CIM->eta_rot_rate_d(2) << ", "
        << CIM->e_tran_pos(0) << ", "
        << CIM->e_tran_pos(1) << ", "
        << CIM->e_tran_pos(2) << ", "
        << CSM->e_tran_pos_I(0) << ", "
        << CSM->e_tran_pos_I(1) << ", "
        << CSM->e_tran_pos_I(2) << ", "
        << CIM->e_tran_vel(0) << ", "
        << CIM->e_tran_vel(1) << ", "
        << CIM->e_tran_vel(2) << ", "
        << CIM->e_eta(0) << ", "
        << CIM->e_eta(1) << ", "
        << CIM->e_eta(2) << ", "
        << CSM->e_eta_I(0) << ", "
        << CSM->e_eta_I(1) << ", "
        << CSM->e_eta_I(2) << ", "
        << CIM->e_eta_rate(0) << ", "
        << CIM->e_eta_rate(1) << ", "
        << CIM->e_eta_rate(2) << ", "
        << CIM->mu_tran_I(0) << ", "
        << CIM->mu_tran_I(1) << ", "
        << CIM->mu_tran_I(2) << ", "
        << CIM->mu_tran_J(0) << ", "
        << CIM->mu_tran_J(1) << ", "
        << CIM->mu_tran_J(2) << ", "
        << CIM->u(0) << ", "
        << CIM->u(1) << ", "
        << CIM->u(2) << ", "
        << CIM->u(3) << ", "
        << CIM->Thrust(0) << ", "
        << CIM->Thrust(1) << ", "
        << CIM->Thrust(2) << ", "
        << CIM->Thrust(3) << ", "
        << (*cntrl_input)(0) << ", "
        << (*cntrl_input)(1) << ", "
        << (*cntrl_input)(2) << ", "
        << (*cntrl_input)(3) << ", "
        << CIM->eta_rot_acceleration_d(0) << ", "
        << CIM->eta_rot_acceleration_d(1) << ", "
        << CIM->eta_rot_acceleration_d(2) << ", "
        ;

    BOOST_LOG(logger_logdata) << oss.str();
}

} // namespace _pid_
} // namespace _qrbp_
