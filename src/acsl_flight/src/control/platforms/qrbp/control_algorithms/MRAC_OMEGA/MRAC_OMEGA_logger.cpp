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
 * File:        MRAC_OMEGA_logger.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 09, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: logging for the MRAC with angular velocities controller. 
 *              Inherits the Blackbox class for logging and takes in the
 *              internal members for the MRAC_OMEGA.
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

#include <MRAC_OMEGA_logger.hpp>

namespace _qrbp_{
namespace _mrac_omega_{

mrac_omega_logger::mrac_omega_logger(controller_internal_members* cim, controller_integrated_state_members* csm, 
                                     Eigen::Vector<float, 8>* control_input, const std::string & controller_log_dir_) : 
                                     CIM(cim), CSM(csm), cntrl_input(control_input), 
                                     flight_run_log_directory(controller_log_dir_)
{
	// Initialize the logging from blackbox class.
	if(logInitLogging()) {FLIGHTSTACK_INFO("mrac omega Logger Created");};
}		

// Implementing virtual functions from Blackbox
void mrac_omega_logger::logInitHeaders() {
	std::ostringstream oss;

	oss << ", "
	    << "Controller Time [s], "
		<< "Algorithm Execution Time [us], "
		<< "Coefficient of Lift Upper wing [-], "
        << "Coefficient of Lift Lower wing [-], "
        << "Coefficient of Lift Left stabilizer [-], "
        << "Coefficient of Lift Right stabilizer [-], "
        << "Coefficient of Drag Upper wing [-], "
        << "Coefficient of Drag Lower wing [-], "
        << "Coefficient of Drag Left stabilizer [-], "
        << "Coefficient of Drag Right stabilizer [-], "
        << "Coefficient of Aero Moment Upper wing [-], "
        << "Coefficient of Aero Moment Lower wing [-], "
        << "Coefficient of Aero Moment Left stabilizer [-], "
        << "Coefficient of Aero Moment Right stabilizer [-], "
		<< "Squared Norm of Velocity [-], "
        << "Angle of attack COG [rad], "
        << "Angle of attack Upper wing [rad], "
        << "Angle of attack Lower wing [rad], "
        << "Sideslip angle COG [rad], "
        << "Sideslip angle Left stabilizer [rad], "
        << "Sideslip angle Right stabilizer [rad], "
		<< "Aero F_x [N], "
        << "Aero F_y [N], "
        << "Aero F_z [N], "
        << "Aero M_x [Nm], "
        << "Aero M_y [Nm], "
        << "Aero M_z [Nm], "
		<< "outer_loop_dyn_inv F_x [N], "
		<< "outer_loop_dyn_inv F_y [N], "
		<< "outer_loop_dyn_inv F_z [N], "
        << "inner_loop_dyn_inv M_x [Nm], "
		<< "inner_loop_dyn_inv M_y [Nm], "
		<< "inner_loop_dyn_inv M_z [Nm], "
		<< "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
		<< "Reference Position x [m], "
        << "Reference Position y [m], "
        << "Reference Position z [m], "
        << "Reference Velocity x [m/s], "
        << "Reference Velocity y [m/s], "
        << "Reference Velocity z [m/s], "
		<< "Reference Acceleration x [m/s^2], "
        << "Reference Acceleration y [m/s^2], "
        << "Reference Acceleration z [m/s^2], "
        << "User Position x [m], "
        << "User Position y [m], "
        << "User Position z [m], "
        << "User Velocity x [m/s], "
        << "User Velocity y [m/s], "
        << "User Velocity z [m/s], "
        << "User Acceleration x [m/s^2], "
        << "User Acceleration y [m/s^2], "
        << "User Acceleration z [m/s^2], "
		<< "Error in x [m], "
        << "Error in y [m], "
        << "Error in z [m], "
        << "Error in vx [m/s], "
        << "Error in vy [m/s], "
        << "Error in vz [m/s], "
        << "Integral Error in x [m], "
        << "Integral Error in y [m], "
        << "Integral Error in z [m], "
        << "Reference Error in x [m], "
        << "Reference Error in y [m], "
        << "Reference Error in z [m], "
        << "Integral Reference Error in x [m], "
        << "Integral Reference Error in y [m], "
        << "Integral Reference Error in z [m], "
        << "Reference Command in x [-], "
        << "Reference Command in y [-], "
        << "Reference Command in z [-], "
        << "mu_baseline x [N], " 
        << "mu_baseline y [N], " 
        << "mu_baseline z [N], " 
        << "mu_adaptive x [N], "
        << "mu_adaptive y [N], "
        << "mu_adaptive z [N], "
        << "mu in Body x [N], "
        << "mu in Body y [N], "
        << "mu in Body z [N], "
        << "Desired Phi [rad], "
        << "Desired Theta [rad], "
        << "Desired Psi [rad], "
        << "Desired w_x [rad/s], "
        << "Desired w_y [rad/s], "
        << "Desired w_z [rad/s], "
        << "Desired w_x_dot [rad/s^2], "
        << "Desired w_y_dot [rad/s^2], "
        << "Desired w_z_dot [rad/s^2], "
        << "Phi [rad], "
        << "Theta [rad], "
        << "Psi [rad], "
        << "w_x [rad/s], "
        << "w_y [rad/s], "
        << "w_z [rad/s], "
        << "Command w_x [rad/s], "
        << "Command w_y [rad/s], "
        << "Command w_z [rad/s], "
        << "Reference w_x [rad/s], "
        << "Reference w_y [rad/s], "
        << "Reference w_z [rad/s], "
        << "Reference alpha_x [rad/s^2], "
        << "Reference alpha_y [rad/s^2], "
        << "Reference alpha_z [rad/s^2], "
        << "Error in phi [rad], "
        << "Error in theta [rad], "
        << "Error in psi [rad], "
        << "Integral Error in phi [rad], "
        << "Integral Error in theta [rad], "
        << "Integral Error in psi [rad], "
        << "Reference error in w_x [rad/s], "
        << "Reference error in w_y [rad/s], "
        << "Reference error in w_z [rad/s], "
        << "Error in w_x [rad/s], "
        << "Error in w_y [rad/s], "
        << "Error in w_z [rad/s], "
        << "Integral reference error in w_x [rad/s], "
        << "Integral reference error in w_y [rad/s], "
        << "Integral reference error in w_z [rad/s], "
        << "Tau x baseline [Nm], "
        << "Tau y baseline [Nm], "
        << "Tau z baseline [Nm], "
        << "Tau x adaptive [Nm], "
        << "Tau y adaptive [Nm], "
        << "Tau z adaptive [Nm], "
        << "Control input 1 [N], "
        << "Control input 2 [N], "
        << "Control input 3 [N], "
        << "Control input 4 [N], "
        << "Thrust Motor 1 [N], "
        << "Thrust Motor 2 [N], "
        << "Thrust Motor 3 [N], "
        << "Thrust Motor 4 [N], "
        << "Normalized Thrust 1 [-], "
        << "Normalized Thrust 2 [-], "
        << "Normalized Thrust 3 [-], "
        << "Normalized Thrust 4 [-], "
        << "dead_zone_value_translational [-], "
        << "dead_zone_value_rotational [-], "
        << "proj_op_activated_K_hat_x_translational [-], "
        << "proj_op_activated_K_hat_r_translational [-], "
        << "proj_op_activated_Theta_hat_translational [-], "
        << "proj_op_activated_K_hat_x_rotational [-], "
        << "proj_op_activated_K_hat_r_rotational [-], "
        << "proj_op_activated_Theta_hat_rotational [-], "
		;

        // Use the deader function to create the header for the matrix data
        generateMatrixHeaders(oss, "K_hat_x_translational", CSM->K_hat_x_tran, "[-]");
        generateMatrixHeaders(oss, "K_hat_r_translational", CSM->K_hat_r_tran, "[-]");
        generateMatrixHeaders(oss, "Theta_hat_translational", CSM->Theta_hat_tran, "[-]");
        generateMatrixHeaders(oss, "K_hat_x_rotational", CSM->K_hat_x_rot, "[-]");
        generateMatrixHeaders(oss, "K_hat_r_rotational", CSM->K_hat_r_rot, "[-]");
        generateMatrixHeaders(oss, "Theta_hat_rotational", CSM->Theta_hat_rot, "[-]");

		BOOST_LOG(logger_logdata) << oss.str();
}

// Implementing virtual functions from Blackbox
bool mrac_omega_logger::logInitLogging() {
    try {
        // Create the directory path for the controller specific flight run logs
        std::string flight_run_controller_specific_directory = flight_run_log_directory + "/" + "MRAC_OMEGA";

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
        std::string gains_source_file = "./src/acsl_flight/params/control_algorithms/qrbp/MRAC_OMEGA/gains_MRAC_OMEGA.json";
        std::stringstream gains_target_ss;
        gains_target_ss << flight_run_controller_specific_parameter_directory << "/gains_MRAC_OMEGA.json" ;
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
        FLIGHTSTACK_ERROR("Exception:", e.what());
    } 

    return false; // IF there are errors
}

// Implementing virtual functions from Blackbox
void mrac_omega_logger::logLogData() {
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << CIM->t << ", "
        << CIM->alg_duration << ", "
        << CIM->aero.coeff.Cl_up << ", "
        << CIM->aero.coeff.Cl_lw << ", "
        << CIM->aero.coeff.Cl_lt << ", "
        << CIM->aero.coeff.Cl_rt << ", "
        << CIM->aero.coeff.Cd_up << ", "
        << CIM->aero.coeff.Cd_lw << ", "
        << CIM->aero.coeff.Cd_lt << ", "
        << CIM->aero.coeff.Cd_rt << ", "
        << CIM->aero.coeff.Cm_up << ", "
        << CIM->aero.coeff.Cm_lw << ", "
        << CIM->aero.coeff.Cm_lt << ", "
        << CIM->aero.coeff.Cm_rt << ", "
        << CIM->aero.states.v_norm_sq << ", "
        << CIM->aero.states.alpha << ", "
        << CIM->aero.states.alpha_up << ", "
        << CIM->aero.states.alpha_lw << ", "
        << CIM->aero.states.beta << ", "
        << CIM->aero.states.beta_lt << ", "
        << CIM->aero.states.beta_rt << ", "
        << CIM->aero.dyn.F_aero(0) << ", "
        << CIM->aero.dyn.F_aero(1) << ", "
        << CIM->aero.dyn.F_aero(2) << ", "
        << CIM->aero.dyn.M_aero(0) << ", "
        << CIM->aero.dyn.M_aero(1) << ", "
        << CIM->aero.dyn.M_aero(2) << ", "
        << CIM->aero.dyn.outer_loop_dynamic_inv(0) << ", "
        << CIM->aero.dyn.outer_loop_dynamic_inv(1) << ", "
        << CIM->aero.dyn.outer_loop_dynamic_inv(2) << ", "
        << CIM->aero.dyn.inner_loop_dynamic_inv(0) << ", "
        << CIM->aero.dyn.inner_loop_dynamic_inv(1) << ", "
        << CIM->aero.dyn.inner_loop_dynamic_inv(2) << ", "
        << CIM->x_tran(0) << ", "
        << CIM->x_tran(1) << ", "
        << CIM->x_tran(2) << ", "
        << CIM->x_tran(3) << ", "
        << CIM->x_tran(4) << ", "
        << CIM->x_tran(5) << ", "
        << CSM->x_tran_ref(0) << ", "
        << CSM->x_tran_ref(1) << ", "
        << CSM->x_tran_ref(2) << ", "
        << CSM->x_tran_ref(3) << ", "
        << CSM->x_tran_ref(4) << ", "
        << CSM->x_tran_ref(5) << ", "
        << CIM->x_tran_ref_dot(3) << ", "
        << CIM->x_tran_ref_dot(4) << ", "
        << CIM->x_tran_ref_dot(5) << ", "
        << CIM->r_user(0) << ", "
        << CIM->r_user(1) << ", "
        << CIM->r_user(2) << ", "
        << CIM->r_dot_user(0) << ", "
        << CIM->r_dot_user(1) << ", "
        << CIM->r_dot_user(2) << ", "
        << CIM->r_ddot_user(0) << ", "
        << CIM->r_ddot_user(1) << ", "
        << CIM->r_ddot_user(2) << ", "
        << CIM->e_tran(0) << ", "
        << CIM->e_tran(1) << ", "
        << CIM->e_tran(2) << ", "
        << CIM->e_tran(3) << ", "
        << CIM->e_tran(4) << ", "
        << CIM->e_tran(5) << ", "
        << CSM->e_tran_pos_I(0) << ", "
        << CSM->e_tran_pos_I(1) << ", "
        << CSM->e_tran_pos_I(2) << ", "
        << CIM->e_tran_pos_ref(0) << ", "
        << CIM->e_tran_pos_ref(1) << ", "
        << CIM->e_tran_pos_ref(2) << ", "
        << CSM->e_tran_pos_ref_I(0) << ", "
        << CSM->e_tran_pos_ref_I(1) << ", "
        << CSM->e_tran_pos_ref_I(2) << ", "
        << CIM->r_cmd_tran(0) << ", "
        << CIM->r_cmd_tran(1) << ", "
        << CIM->r_cmd_tran(2) << ", "
        << CIM->mu_tran_baseline(0) << ", "
        << CIM->mu_tran_baseline(1) << ", "
        << CIM->mu_tran_baseline(2) << ", "
        << CIM->mu_tran_adaptive(0) << ", "
        << CIM->mu_tran_adaptive(1) << ", "
        << CIM->mu_tran_adaptive(2) << ", "
        << CIM->mu_tran_J(0) << ", "
        << CIM->mu_tran_J(1) << ", "
        << CIM->mu_tran_J(2) << ", "
        << CIM->eta_rot_d(0) << ", "
        << CIM->eta_rot_d(1) << ", "
        << CIM->eta_rot_d(2) << ", "
        << CIM->omega_rot_d(0) << ", "
        << CIM->omega_rot_d(1) << ", "
        << CIM->omega_rot_d(2) << ", "
        << CIM->alpha_rot_d(0) << ", "
        << CIM->alpha_rot_d(1) << ", "
        << CIM->alpha_rot_d(2) << ", "
        << CIM->eta_rot(0) << ", "
        << CIM->eta_rot(1) << ", "
        << CIM->eta_rot(2) << ", "
        << CIM->omega_rot(0) << ", "
        << CIM->omega_rot(1) << ", "
        << CIM->omega_rot(2) << ", "
        << CIM->omega_cmd(0) << ", "
        << CIM->omega_cmd(1) << ", "
        << CIM->omega_cmd(2) << ", "
        << CSM->omega_rot_ref(0) << ", "
        << CSM->omega_rot_ref(1) << ", "
        << CSM->omega_rot_ref(2) << ", "
        << CIM->omega_rot_ref_dot(0) << ", "
        << CIM->omega_rot_ref_dot(1) << ", "
        << CIM->omega_rot_ref_dot(2) << ", "
        << CIM->e_rot_eta(0) << ", "
        << CIM->e_rot_eta(1) << ", "
        << CIM->e_rot_eta(2) << ", "
        << CSM->e_rot_eta_I(0) << ", "
        << CSM->e_rot_eta_I(1) << ", "
        << CSM->e_rot_eta_I(2) << ", "
        << CIM->e_rot_omega_ref(0) << ", "
        << CIM->e_rot_omega_ref(1) << ", "
        << CIM->e_rot_omega_ref(2) << ", "
        << CIM->e_rot_omega(0) << ", "
        << CIM->e_rot_omega(1) << ", "
        << CIM->e_rot_omega(2) << ", "
        << CSM->e_rot_omega_ref_I(0) << ", "
        << CSM->e_rot_omega_ref_I(1) << ", "
        << CSM->e_rot_omega_ref_I(2) << ", "
        << CIM->tau_rot_baseline(0) << ", "
        << CIM->tau_rot_baseline(1) << ", "
        << CIM->tau_rot_baseline(2) << ", "
        << CIM->tau_rot_adaptive(0) << ", "
        << CIM->tau_rot_adaptive(1) << ", "
        << CIM->tau_rot_adaptive(2) << ", "
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
        << CIM->dead_zone_value_translational << ", "
        << CIM->dead_zone_value_rotational << ", "
        << CIM->proj_op_activated_K_hat_x_translational << ", "
        << CIM->proj_op_activated_K_hat_r_translational << ", "
        << CIM->proj_op_activated_Theta_hat_translational << ", "
        << CIM->proj_op_activated_K_hat_x_rotational << ", "
        << CIM->proj_op_activated_K_hat_r_rotational << ", "
        << CIM->proj_op_activated_Theta_hat_rotational << ", "
        ;

        // Use helper function to output the matrix data
        appendEigenData(oss, CSM->K_hat_x_tran);
        appendEigenData(oss, CSM->K_hat_r_tran);
        appendEigenData(oss, CSM->Theta_hat_tran);
        appendEigenData(oss, CSM->K_hat_x_rot);
        appendEigenData(oss, CSM->K_hat_r_rot);
        appendEigenData(oss, CSM->Theta_hat_rot);

    BOOST_LOG(logger_logdata) << oss.str();
}

} // namespace _mrac_omega_
} // namespace _qrbp_