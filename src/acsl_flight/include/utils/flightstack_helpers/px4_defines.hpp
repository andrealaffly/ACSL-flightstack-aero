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
 * File:        px4_defines.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        September 11, 2024
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Contains useful constant defines for use in the flight stack in a central location
 * 
 * GitHub:    https://github.com/andrealaffly/ACSL-flightstack-winged
 **********************************************************************************************************************/

#ifndef PX4_DEFINES_
#define PX4_DEFINES_

#include <cstdint>   // For uint8_t, uint16_t

/// ------------- USER DEFINED ------------- ///
// -> Pixhawk vehicle mode command message package definitions
inline constexpr uint8_t TARGET_SYSTEM = 1;
inline constexpr uint8_t TARGET_COMPONENT = 1;
inline constexpr uint8_t SOURCE_SYSTEM = 1;
inline constexpr uint16_t SOURCE_COMPONENT = 1;
inline constexpr bool FROM_EXTERNAL_BOOL = true;
// -> Set Custom mode
inline constexpr float ENABLE_CUSTOM_MODE = 1;
// -> Custom Control mode definitions for pixhawk vehicle mode command
inline constexpr float CUSTOM_MAIN_MODE_MANUAL = 1;
inline constexpr float CUSTOM_MAIN_MODE_ALTCTL = 2;
inline constexpr float CUSTOM_MAIN_MODE_POSCTL = 3;
inline constexpr float CUSTOM_MAIN_MODE_AUTO = 4;
inline constexpr float CUSTOM_MAIN_MODE_ACRO = 5;
inline constexpr float CUSTOM_MAIN_MODE_OFFBOARD = 6;
inline constexpr float CUSTOM_MAIN_MODE_STABILIZED = 7;
inline constexpr float CUSTOM_MAIN_MODE_RATTITUDE = 8;
// -> ARM/DISARM defines
inline constexpr float ARM_ACSL_VEH = 1.0;
inline constexpr float DISARM_ACSL_VEH = 0.0;
// -> OFFBOARD_CONTROL_MODE define
inline constexpr bool OFFBOARD_POS_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_VEL_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_ACCEL_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_ATT_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_BODY_RATE_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_THRST_TRQ_CTRL_BOOL = false;
inline constexpr bool OFFBOARD_DIRECT_ACT_CTRL_BOOL = true;

// -> Pixhawk vehicle odometry pose frame message package definitions
inline constexpr uint8_t POSE_FRAME_UNKNOWN = 0; // Pose frame unknown
inline constexpr uint8_t POSE_FRAME_NED     = 1; // NED earth-fixed frame
inline constexpr uint8_t POSE_FRAME_FRD     = 2; // FRD world-fixed frame, arbitrary heading reference
// -> Pixhawk vehicle odometry velocity frame message package definitions
inline constexpr uint8_t VELOCITY_FRAME_UNKNOWN = 0;  // Velocity frame unknown
inline constexpr uint8_t VELOCITY_FRAME_NED      = 1; // NED earth-fixed frame
inline constexpr uint8_t VELOCITY_FRAME_FRD      = 2; // FRD world-fixed frame, arbitrary heading reference
inline constexpr uint8_t VELOCITY_FRAME_BODY_FRD = 3; // FRD body-fixed frame


#endif // PX4_DEFINES_