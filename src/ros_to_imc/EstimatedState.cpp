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

#include <imc_ros_bridge/ros_to_imc/EstimatedState.h>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_bridge::EstimatedState& ros_msg, IMC::EstimatedState& imc_msg)
{
    imc_msg.lat = ros_msg.lat;
    imc_msg.lon = ros_msg.lon;
    imc_msg.height = ros_msg.height;
    imc_msg.x = ros_msg.x;
    imc_msg.y = ros_msg.y;
    imc_msg.z = ros_msg.z;
    imc_msg.phi = ros_msg.phi;
    imc_msg.theta = ros_msg.theta;
    imc_msg.psi = ros_msg.psi;
    imc_msg.u = ros_msg.u;
    imc_msg.v = ros_msg.v;
    imc_msg.w = ros_msg.w;
    imc_msg.vx = ros_msg.vx;
    imc_msg.vy = ros_msg.vy;
    imc_msg.vz = ros_msg.vz;
    imc_msg.p = ros_msg.p;
    imc_msg.q = ros_msg.q;
    imc_msg.r = ros_msg.r;
    imc_msg.depth = ros_msg.depth;
    imc_msg.alt = ros_msg.alt;

    return true;
}

} // namespace ros_to_imc
