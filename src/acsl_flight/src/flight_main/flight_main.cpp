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
 * File:        flight_main.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 16, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Main function to initiate ROS2.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#include "flight_main.hpp"
#include "global_helpers.hpp"

using namespace _flight_bridge_;
using namespace _flightstack_;


int main(int argc, char * argv[])
{

    std::cout << COLOR_GREEN 
              << "\n"
              << "===================== STARTING-ACSL-FLIGHTSTACK-WINGED =====================" 
              << COLOR_RESET << std::endl;
    
    // Intialzie the reader object for reading the flight parameters
    FlightConfigReader reader;  

    // Read in the flight parameters
    flight_params run_params = reader.readFlightConfig("./src/acsl_flight/params/flight_params/flight_main_params.json");

    // Print the flight parameter table
    reader.printParameterTable(run_params);

    // Initialize ROS2
    FLIGHTSTACK_INFO("Initializing ROS2");
    rclcpp::init(argc, argv);    

    // Initialize flight_bridge
    flight_bridge flight(&run_params);

    // Intialize the nodes
    flight.init();
   
    // ROS2 housekeeping
    rclcpp::shutdown();
}