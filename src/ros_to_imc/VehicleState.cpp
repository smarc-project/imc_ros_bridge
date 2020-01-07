/* Copyright 2019 The SMaRC project (https://smarc.se/)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <imc_ros_bridge/ros_to_imc/VehicleState.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_bridge::VehicleState& ros_msg, IMC::VehicleState& imc_msg)
{
    imc_msg.op_mode = ros_msg.op_mode;
    imc_msg.error_count = ros_msg.error_count;
    imc_msg.error_ents = ros_msg.error_ents;
    imc_msg.maneuver_type = ros_msg.maneuver_type;
    imc_msg.maneuver_stime = ros_msg.maneuver_stime;
    imc_msg.maneuver_eta = ros_msg.maneuver_eta;
    imc_msg.control_loops = ros_msg.control_loops;
    imc_msg.flags = ros_msg.flags;
    imc_msg.last_error = ros_msg.last_error;
    imc_msg.last_error_time = ros_msg.last_error_time;

    return true;
}

} // namespace imc_to_ros
