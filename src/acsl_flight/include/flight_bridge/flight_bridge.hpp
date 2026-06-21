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

 /**********************************************************************************************************************  
 * Part of the code in this file leverages the following material.
 *
 * Referenced:  https://github.com/ros2/examples/tree/galactic/rclcpp/executors/cbg_executor
 *              Copyright (c) 2020 Robert Bosch GmbH
 *
 *              Licensed under the Apache License, Version 2.0 (the "License");
 *              you may not use this file except in compliance with the License.
 *              You may obtain a copy of the License at
 *
 *                        http://www.apache.org/licenses/LICENSE-2.0
 *
 *              Unless required by applicable law or agreed to in writing, software
 *              distributed under the License is distributed on an "AS IS" BASIS,
 *              WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *              See the License for the specific language governing permissions and
 *              limitations under the License.
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * File:        flight_bridge.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 12, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Utilities for multithreading.
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef FLIGHT_BRIDGE_HPP_
#define FLIGHT_BRIDGE_HPP_

#include <chrono>
#include <string>
#include <thread>
#include <cinttypes>
#include <cstdlib>
#include <ctime>
#include <atomic>

#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <csignal>

#include <pthread.h> // For Linux

#include "rclcpp/rclcpp.hpp"   // ROS2 Client Library C++ header
#include "rclcpp/executor.hpp" // For executors
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

/// ------------- USER DEFINED ------------- ///
#include "read_pix.hpp"
#include "vehicle_class.hpp"
#include "control.hpp"
#include "mocap.hpp"
#include "vio.hpp"
#include "weatherapp.hpp"
#include "flight_params.hpp"
#include "global_config.hpp"


using namespace std::chrono_literals; // For creating executors
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

namespace _flight_bridge_{
    
/***********************************************************************************************/
/*                                  HELPER FUNCTIONS                                           */
/***********************************************************************************************/
/// Retrieves the value of the given seconds parameter in std::chrono nanoseconds.
inline std::chrono::milliseconds get_millis_from_secs_parameter(
    const rclcpp::Node * node,
    const std::string & name)
{
    double seconds = 0.0;
    node->get_parameter(name, seconds);
    auto millis = std::chrono::milliseconds(static_cast<int64_t>(seconds * 1000000.0));
    return millis;
}

/// Enum for simple configuration of threads in two priority classes.
enum class ThreadPriority
{
    LOW,
    HIGH
};

/// Sets the priority of the given native thread to max or min as given.
/// The exact priority value depends on the operating system. On Linux,
/// this requires elevated privileges.
/// Furthermore, if a non-negative CPU id is given, the thread is pinned
/// to that CPU.
template<typename T>
bool configure_native_thread(T native_handle, ThreadPriority priority, int cpu_id)
{
    bool success = true;
    sched_param params;
    int policy;
    success &= (pthread_getschedparam(native_handle, &policy, &params) == 0);
    if (priority == ThreadPriority::HIGH) {
    params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    } else {
    params.sched_priority = sched_get_priority_min(SCHED_FIFO);
    }
    success &= (pthread_setschedparam(native_handle, SCHED_FIFO, &params) == 0);
    if (cpu_id >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    success &= (pthread_setaffinity_np(native_handle, sizeof(cpu_set_t), &cpuset) == 0);
    }

    return success;
}

/// Sets the priority of the given thread to max or min as given. The exact
/// scheduler priority depends on the operating system. On Linux, this
/// requires elevated privileges.
/// Furthermore, if a non-negative CPU id is given, the thread is pinned
/// to that CPU.
inline bool configure_thread(std::thread & thread, ThreadPriority priority, int cpu_id)
{
    return configure_native_thread(thread.native_handle(), priority, cpu_id);
}

/// Returns the time of the given native thread handle as std::chrono
/// timestamp. This allows measuring the execution time of this thread.
template<typename T>
std::chrono::nanoseconds get_native_thread_time(T native_handle)
{
    // i.e., Linux platform.
    clockid_t id;
    pthread_getcpuclockid(native_handle, &id);
    timespec spec;
    clock_gettime(id, &spec);
    return std::chrono::seconds{spec.tv_sec} + std::chrono::nanoseconds{spec.tv_nsec};

}

/// Returns the time of the given thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
inline std::chrono::nanoseconds get_thread_time(std::thread & thread)
{
    return get_native_thread_time(thread.native_handle());
}

/// Returns the time of the current thread as std::chrono timestamp.
/// This allows measuring the execution time of this thread.
inline std::chrono::nanoseconds get_current_thread_time()
{
    return get_native_thread_time(pthread_self());
}

/// Retruns the global log file directory as a string to be used in logging mocap, vio, control data.
/// This allows us to unify the data collection directory as vio and mocap are often times slower to 
/// start than the control thread.
inline std::string create_global_log_directory(const std::string& platform_)
{
    // Get the current time and date
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);

    // Get the current date in the desired format
    std::stringstream date_ss;
    date_ss << std::put_time(std::localtime(&now_c), "%Y_%m_%d");   // Current date
    std::string current_date = date_ss.str();

    // Create the directory path for the logs
    std::string log_directory = "./src/acsl_flight/flight_log/" + platform_ + "/" + current_date;

    // Check if the directory exists, and create it if it doesn't
    if (!std::filesystem::exists(log_directory)) {
        std::filesystem::create_directories(log_directory);
    }

    // Get the current time in the desired format
    std::stringstream time_ss;
    time_ss << std::put_time(std::localtime(&now_c), "%H_%M_%S"); // Current time in hours and minutes
    std::string current_time = time_ss.str();

    // Create the directory path for the flight run logs
    std::string flight_run_log_directory = log_directory + "/" + "flight_run_" + current_time;

    // Check if the directory exists, and create it if it doesn't
    if (!std::filesystem::exists(flight_run_log_directory)) {
        std::filesystem::create_directories(flight_run_log_directory);
    }

    return flight_run_log_directory;  // Return the log directory path
}

/***********************************************************************************************/
/*                                  FLIGHT BRIDGE CLASS                                        */
/***********************************************************************************************/
class flight_bridge
{
    public:

        /// Constructor
        flight_bridge(flight_params* p);

        /// Destructor

        // Add Callback Group
        void init();

    private:
        /// Thread creation boolean
        bool ret;

        /// CPU numbers
        // -> Here the ODroid M1s with an rk3566 is used. IT has 4 CPU cores and 4 threads.
        // -> You can specify the more cpu cores if you do possess more cores.
        const int CPU_ZERO = 0;
        const int CPU_ONE = 1;
        const int CPU_TWO = 2;
        const int CPU_THREE = 3;

        // Create an instance of vehicle_state as object vehicle
        vehicle_states vehicle;

        /// Instance of the flight_params_ptr to point to the flight_parameter object in
        /// flight_main.cpp
        flight_params* flight_params_ptr;

        /// Instance of the global flight_run_directory
        std::string global_log_dir;
};

} // namespace _flight_bridge_





#endif  // FLIGHT_BRIDGE_HPP_