# ======================================================================================================================
# BSD 3-Clause License

# Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# ======================================================================================================================

cmake_minimum_required(VERSION 3.8)
project(acsl_flight)

# Set CMake build type to release
set(CMAKE_BUILD_TYPE Release)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realsense2 REQUIRED)  # For VIO                                        
find_package(Boost 1.86 REQUIRED COMPONENTS log log_setup system filesystem thread)   # Most systems have 1.71.0. Change it to that if needed.

# include directories
include_directories(${Boost_INCLUDE_DIRS}
                    ${realsense_INCLUDE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/json/include/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/eigen/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/config/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/flightstack_helpers/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/flight_log/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/lyapunov_solver/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/adaptive_laws/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/adaptive_laws/operators/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/control_algorithm_base/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/user_defined_trajectory_base/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_driver/io_context/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_driver/udp_driver
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/udp_driver/udp_socket/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/mocap/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/vio/  
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/WeatherAPP/  
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/flight_main/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/flight_bridge/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/read_pix/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/vehicle_class/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/flight_params/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/piecewise_polynomial_trajectory/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/                  
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/aerodynamics/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/control_algorithms/PID/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/control_algorithms/MRAC_PID/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/control_algorithms/PID_OMEGA/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/control_algorithms/MRAC_OMEGA/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/qrbp/control_algorithms/MRAC_OBSERVER/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/quadm/
                    ${CMAKE_CURRENT_SOURCE_DIR}/include/control/platforms/quadm/control_algorithms/PID/
                  )

# add source files
set(SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/lyapunov_solver/continuous_lyapunov_equation.cpp
  src/udp_driver/io_context/io_context.cpp
  src/udp_driver/udp_driver/udp_driver.cpp
  src/udp_driver/udp_socket/udp_socket.cpp
  src/mocap/mocap.cpp
  src/vio/vio.cpp
  src/WeatherAPP/weatherapp.cpp
  src/flight_main/flight_main.cpp
  src/flight_bridge/flight_bridge.cpp
  src/read_pix/read_pix.cpp
  src/vehicle_class/vehicle_class.cpp
  src/control/control.cpp
  src/flight_params/flight_params.cpp
  src/piecewise_polynomial_trajectory/piecewise_polynomial_trajectory.cpp
  src/control/platforms/qrbp/control_algorithms/PID/PID.cpp
  src/control/platforms/qrbp/control_algorithms/PID/PID_logger.cpp
  src/control/platforms/qrbp/control_algorithms/PID_OMEGA/PID_OMEGA.cpp
  src/control/platforms/qrbp/control_algorithms/PID_OMEGA/PID_OMEGA_logger.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_OMEGA/MRAC_OMEGA.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_OMEGA/MRAC_OMEGA_logger.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_OBSERVER/MRAC_OBSERVER.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_OBSERVER/MRAC_OBSERVER_logger.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_PID/MRAC_PID.cpp
  src/control/platforms/qrbp/control_algorithms/MRAC_PID/MRAC_PID_logger.cpp
  src/control/platforms/quadm/control_algorithms/PID/PID_TD.cpp
  src/control/platforms/quadm/control_algorithms/PID/PID_TD_logger.cpp
)

# add executable
add_executable(start ${SOURCES})

# Link against Boost libraries and realsense2 libraries
target_link_libraries(start ${Boost_LIBRARIES} realsense2::realsense2)

# ament dependencies
ament_target_dependencies(start rclcpp px4_msgs std_msgs lifecycle_msgs rclcpp_lifecycle)

# install targets
install(TARGETS
  start
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# export ament dependencies
ament_export_dependencies(rclcpp px4_msgs std_msgs lifecycle_msgs)

ament_package()
