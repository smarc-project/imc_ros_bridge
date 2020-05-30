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

#include <imc_ros_bridge/ros_to_imc/PlanDB.h>

#include <imc_ros_bridge/PlanDBInformation.h>

#include <IMC/Spec/PlanDBInformation.hpp>
#include <IMC/Spec/PlanDBState.hpp>

#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/MessageList.hpp>

namespace ros_to_imc{

template <>
bool convert(const imc_ros_bridge::PlanDB& ros_msg, IMC::PlanDB& imc_msg)
{
	// A very incomplete conversion that does not handle the arg field of a real
	// PlanDB message. Hopefully it wont be needed ever...
	// This is used to answer Neptus 'SET SUCCESS for plan_id' only.
	imc_msg.type = ros_msg.type;
	imc_msg.op = ros_msg.op;
	imc_msg.request_id = ros_msg.request_id;
	imc_msg.plan_id = ros_msg.plan_id;

	
	imc_ros_bridge::PlanDBInformation ros_plandbinfo = ros_msg.plandb_information;
	// check if there is anything in PlanDBInformation
	if(ros_plandbinfo.plan_id != ""){
		// okay, there is some info in here that we need to send to imc
		// this message is normally set as the 'arg' field of the imc plandb message
		// so we need to construct that and fill it in from the ros msg.
		IMC::PlanDBInformation plandbinfo = IMC::PlanDBInformation();
		plandbinfo.plan_id = ros_plandbinfo.plan_id;
		plandbinfo.plan_size = ros_plandbinfo.plan_size;
		plandbinfo.change_time = ros_plandbinfo.change_time;
		plandbinfo.change_sid = ros_plandbinfo.change_sid;
		plandbinfo.change_sname = ros_plandbinfo.change_sname;

		// md5 is a uint8[], but IMC::plandb is a std::vector<char>
		// so we fill that in one by one while casting our uint8s.
		// A working system was found at 23:15 on a Friday.
		// Very healthy. yes.
		for(int i=0; i<ros_plandbinfo.md5.size(); i++){
			plandbinfo.md5.push_back((char)ros_plandbinfo.md5[i]);
		}

		// arg is of type IMC::InlineMessage<>, so we gotta put our arg into it
		imc_msg.arg.set(plandbinfo);
		
	}

	// same thing as above, but for arg=PlanDBState type
	imc_ros_bridge::PlanDBState plan_state = ros_msg.plandb_state;
	if(plan_state.plan_count >= 1){
		IMC::PlanDBState pdb_state = IMC::PlanDBState();

		pdb_state.plan_count = plan_state.plan_count;
		pdb_state.plan_size = plan_state.plan_size;
		pdb_state.change_time = plan_state.change_time;
		pdb_state.change_sid = plan_state.change_sid;
		pdb_state.change_sname = plan_state.change_sname;

		for(int i=0; i<plan_state.md5.size(); i++){
			pdb_state.md5.push_back((char)plan_state.md5[i]);
		}

		auto plans_info = IMC::MessageList<IMC::PlanDBInformation>();
		for(int i=0; i<plan_state.plans_info.size(); i++){
			auto plan_info = IMC::PlanDBInformation();
			auto ros_plan_info = plan_state.plans_info[i];

			// This should be a function really... 
			plan_info.plan_id = ros_plan_info.plan_id;
			plan_info.plan_size = ros_plan_info.plan_size;
			plan_info.change_time = ros_plan_info.change_time;
			plan_info.change_sid = ros_plan_info.change_sid;
			plan_info.change_sname = ros_plan_info.change_sname;
			for(int i=0; i<ros_plan_info.md5.size(); i++){
				plan_info.md5.push_back((char)ros_plan_info.md5[i]);
			}

			plans_info.push_back(plan_info);
		}
		pdb_state.plans_info = plans_info;

		imc_msg.arg.set(pdb_state);
	}



	return true;


}


} // namespace ros_to_imc 
