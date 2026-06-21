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
 * File:        control.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Contorl node definition. Runs all the control algorithms. Adds as a hub for all the control actions.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "control.hpp"

namespace _control_
{
    /***********************************************************************************************/
    /*                                  NODE CONSTRUCTOR                                           */
    /***********************************************************************************************/
    controlNode::controlNode(vehicle_states* d, flight_params* p, const std::string & global_log_dir)
    : rclcpp::Node("control_node"), vehicle_ptr(d), flight_params_ptr(p), controller_(p, global_log_dir)
    {
        /// Set offboard flag to 0.
        offboard_flag_ = 0; 

        using std::placeholders::_1;

        /// Declare the parameter for the control period in milliseconds
        // -> Read in the flight_main.cpp functon and passed to this file throught the flight_bridge.
        std::chrono::milliseconds duration(flight_params_ptr->controller_rate);
        /// Create the timer for the controller and declare it to run at the CONTROL_RATE specified in
        /// flight_main_params.json
        control_timer_ = this->create_wall_timer(duration, std::bind(&controlNode::run, this));

        /// Creation of the actuator motors publisher
        publisher_actuator_motors_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
            "/fmu/in/actuator_motors",
            10);

        /// Creation of the vehicle command publisher
        publisher_vehicle_command_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command",
            10);

        /// Creation of the offboard control mode publisher
        publisher_offboard_control_mode_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode",
            10);


        /// Get the initial time stamp after the setup is done for more accuracy
        timestamp_initial_ = this->get_clock()->now().seconds();     

        /// Print that control node started at initial timestamp
        RCLCPP_INFO(this->get_logger(), "CONTROL NODE STARTED AT ROS2 NODE TIME: %lu", timestamp_initial_.load());
    }

    /***********************************************************************************************/
    /*                                  HELPER FUNCTIONS                                           */
    /***********************************************************************************************/
    /// Funtion that updates the current time when called.
    void controlNode::updateCurrentTime(){
        // Udpate local time
        time_current_ = this->get_clock()->now().seconds() - timestamp_initial_;

        // Update the time in the vehicle object
        vehicle_ptr->set_controltime(time_current_);

        // Output the time to the screen
        FLIGHTSTACK_INFO_STREAM_NO_TAG("t:", time_current_, 1000);

    }

    void controlNode::publish_vehicle_command(uint16_t command, float param1, float param2)
    {
        /// Construct the message
        px4_msgs::msg::VehicleCommand msg{};

        // -> Package the vehicle command 
        // -> defines for this are found in control.hpp
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = TARGET_SYSTEM;
        msg.target_component = TARGET_COMPONENT;
        msg.source_system = SOURCE_SYSTEM;
        msg.source_component = SOURCE_COMPONENT;
        msg.from_external = FROM_EXTERNAL_BOOL;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // -> Publish the message
        publisher_vehicle_command_->publish(msg);
    }

    /// Function to arm the vehicle
    void controlNode::arm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, ARM_ACSL_VEH);

        RCLCPP_INFO(this->get_logger(), "Arm command sent");
    }

    /// Function to disarm the vehicle
    void controlNode::disarm()
    {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, DISARM_ACSL_VEH);

        if(!disarm_info_bool_) { RCLCPP_INFO(this->get_logger(), "Disarm command send"); disarm_info_bool_ = true;}
    }

    /// Function to publish offboard control mode
    void controlNode::publish_offboard_control_mode()
    {
        /// Construct the message
        px4_msgs::msg::OffboardControlMode msg{};

        // -> Package the offboard control mode
        // -> defines for this are found in px4_defines.hpp
        msg.position = OFFBOARD_POS_CTRL_BOOL;
        msg.velocity = OFFBOARD_VEL_CTRL_BOOL;
        msg.acceleration = OFFBOARD_ACCEL_CTRL_BOOL;
        msg.attitude = OFFBOARD_ATT_CTRL_BOOL;
        msg.body_rate = OFFBOARD_BODY_RATE_CTRL_BOOL;
        msg.thrust_and_torque = OFFBOARD_THRST_TRQ_CTRL_BOOL;
        msg.direct_actuator = OFFBOARD_DIRECT_ACT_CTRL_BOOL;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        // -> publish the message
        publisher_offboard_control_mode_->publish(msg);
    }

    /// Function to publish motor thrust commands
    void controlNode::publish_actuator_motors(const float t1,
                                              const float t2,
                                              const float t3,
                                              const float t4,
                                              const float t5,
                                              const float t6,
                                              const float t7,
                                              const float t8)
    {
        /// Construct the message
        px4_msgs::msg::ActuatorMotors msg{};
        
        // -> Package the message
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;        
        msg.control[0] = t1;
        msg.control[1] = t2;
        msg.control[2] = t3;
        msg.control[3] = t4;
        msg.control[4] = t5;
        msg.control[5] = t6;
        msg.control[6] = t7;
        msg.control[7] = t8;        
        msg.reversible_flags = 0;
        
        // -> publish the message   
        publisher_actuator_motors_->publish(msg);
    }

    double controlNode::get_rk4_timestep()
    {
        // Convert to ms by multiplying with 1e-3
        return flight_params_ptr->controller_rate*0.001;
    }

    float controlNode::get_min_thrust()
    {
        return flight_params_ptr->min_thrust_after_arm;
    }

    float controlNode::get_zero_thrust()
    {
        return MOTOR_ZERO_THRUST;
    }


    /***********************************************************************************************/
    /*                                  MAIN CONTROL LOOP                                          */
    /***********************************************************************************************/
    /// Definition of the run loop
    void controlNode::run()
    {
        /// Update the current time at the beginning of every loop
        updateCurrentTime();

        // Debug to screen function
        // debug2screen();

        // Main control loop state machine logic
        switch (offboard_flag_) 
        {
            case 0: // Logic before arm time
                if (time_current_ > flight_params_ptr->arm_time) {
                    // Change to Offboard mode
                    this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 
                                                            ENABLE_CUSTOM_MODE, CUSTOM_MAIN_MODE_OFFBOARD);
                    // Arm the vehicle
                    this->arm();

                    // Publish a control message to keep the pixhawk satisfied for a constant offboard signal
                    publish_offboard_control_mode();
                    publish_actuator_motors(get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust(),
                                            get_zero_thrust());

                    // Change the state machine to state 1
                    offboard_flag_ = 1;                    
                }
                break;
            // -------------------------------------------------------------
            case 1: // Logic after arm time and before start time
                if (time_current_ > flight_params_ptr->arm_time && time_current_ < flight_params_ptr->start_time)
                {
                    // Publish least motor spin for quadcopters
                    publish_offboard_control_mode();      

                    if constexpr (_config_param_::PUBLISH_ACTUATOR)
                    {              
                        publish_actuator_motors(get_min_thrust(),
                                                get_min_thrust(),
                                                get_min_thrust(),
                                                get_min_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust());
                    }
                    else
                    {
                        publish_actuator_motors(get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust(),
                                                get_zero_thrust());
                    }

                } else if (time_current_ > flight_params_ptr->start_time) {
                    // Change the state machine to state 2
                    offboard_flag_ = 2;
                }
                break;
            // -------------------------------------------------------------
            case 2: // Logic after start time
                // Compute and publish the control command.
                compute_and_publish_control();

                // If greater than end time. 
                if (time_current_ > flight_params_ptr->end_time)
                {
                    // Change state machine to state 3
                    offboard_flag_ = 3;
                }

                break;
            // -------------------------------------------------------------
            case 3: // Logic after end time

                // Publish zero thrust before disarming the vehicle
                publish_actuator_motors(get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust(),
                                        get_zero_thrust());

                // Disarm the vehicle
                this->disarm();

                // Reaffrim the offboard_flag_ to 3 and wait there
                offboard_flag_ = 3;
                
                break;            
            // -------------------------------------------------------------
            default:
                // Error message for invalid controller choice
                FLIGHTSTACK_ERROR("Error: Contol State Machine entered an invalid state. RUN!!!!!");
                // Terminate the program
                std::exit(EXIT_FAILURE);
                break;
        }
    }

    /***********************************************************************************************/
    /*                                  CONTROL COMPUTATION                                        */
    /***********************************************************************************************/
    // Add your controller to the switch case statement and the controllers can be picked in the flight_main_params.json
    void controlNode::compute_and_publish_control()
    {
        // Update the states and trajectories and compute some matrices.
        controller_.update(vehicle_ptr->get_controltime(),
                           vehicle_ptr->get_x(),
                           vehicle_ptr->get_y(),
                           vehicle_ptr->get_z(),
                           vehicle_ptr->get_vx(),
                           vehicle_ptr->get_vy(),
                           vehicle_ptr->get_vz(),
                           vehicle_ptr->get_q0(),
                           vehicle_ptr->get_q1(),
                           vehicle_ptr->get_q2(),
                           vehicle_ptr->get_q3(),
                           vehicle_ptr->get_roll(),
                           vehicle_ptr->get_pitch(),
                           vehicle_ptr->get_yaw(),
                           vehicle_ptr->get_rollspeed(),
                           vehicle_ptr->get_pitchspeed(),
                           vehicle_ptr->get_yawspeed());

        // Process the dynamics and do the integration.
        controller_.run(get_rk4_timestep());

        // Publish the command if you set it to publish
        publish_offboard_control_mode();
        if constexpr (_config_param_::PUBLISH_ACTUATOR)
        {
            publish_actuator_motors(controller_.get_t1(), 
                                    controller_.get_t2(), 
                                    controller_.get_t3(), 
                                    controller_.get_t4(), 
                                    controller_.get_t5(), 
                                    controller_.get_t6(), 
                                    controller_.get_t7(), 
                                    controller_.get_t8());
        }
        else
        {
            publish_actuator_motors(get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust(), 
                                    get_zero_thrust());
        }
    }
    

}   // namespace _control_