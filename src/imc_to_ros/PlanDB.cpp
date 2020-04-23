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

#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <sstream>
#include <list>

#include <imc_ros_bridge/PlanSpecification.h>
#include <imc_ros_bridge/PlanManeuver.h>

#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/Message.hpp>

#include <IMC/Base/MessageList.hpp>
#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/Goto.hpp>

namespace imc_to_ros {

// template <>
// bool convert(const IMC::PlanDB& imc_msg, std_msgs::String& ros_msg)
// {
    // std::stringstream ostr;
    // ostr.precision(9);
    // imc_msg.fieldsToJSON(ostr, 0);
    // ros_msg.data = ostr.str();
    // std::cout << "Ros message to send:" << std::endl;
    // std::cout << ros_msg.data << std::endl;
    // return true;
// }

template <>
bool convert(const IMC::PlanDB& imc_msg, imc_ros_bridge::PlanDB& ros_msg)
{
	ros_msg.type = imc_msg.type;
	ros_msg.op = imc_msg.op;
	ros_msg.request_id = imc_msg.request_id;
	ros_msg.plan_id = imc_msg.plan_id;

	IMC::InlineMessage<IMC::Message> arg = imc_msg.arg;
	if(arg.isNull()){
		// arg is empty, so the plan_spec is empty too
		// maybe this is a control message or sth.
		std::cout << "PlanDB : Arg was null" << std::endl;
	}else{
		ros_msg.plan_spec = imc_ros_bridge::PlanSpecification();

		int arg_msg_id = arg.get()->getId();
		std::cout << "id:" << arg_msg_id << std::endl;

		// 551 is PlanSpecification
		if(arg_msg_id == 551){
			// cast it to its proper type finally
			// arg.get() returns a Message*, cast that pointer to a pointer to a PlanSpec because we KNOW
			// it is actually pointing to a real PlanSpec object, thanks to the id of the message.
			IMC::PlanSpecification* plan_spec = (IMC::PlanSpecification*) arg.get();
			// fill in the ros side
			ros_msg.plan_spec.plan_id = plan_spec->plan_id;
			ros_msg.plan_spec.description = plan_spec->description;
			ros_msg.plan_spec.vnamespace = plan_spec->vnamespace;
			ros_msg.plan_spec.start_man_id = plan_spec->start_man_id;
			// can we reach this now?
			IMC::MessageList<IMC::PlanManeuver> plan_man_list = plan_spec->maneuvers;
			// i gotta make it into an array before i set it into the ros msg later.
			std::list<imc_ros_bridge::PlanManeuver> maneuvers = std::list<imc_ros_bridge::PlanManeuver>();
			// fill in the list from the imc message
			for(IMC::PlanManeuver* pm : plan_man_list){
				imc_ros_bridge::PlanManeuver plan_maneuver = imc_ros_bridge::PlanManeuver();
				plan_maneuver.maneuver_id = pm->maneuver_id;
				plan_maneuver.maneuver = imc_ros_bridge::Maneuver();

				IMC::InlineMessage<IMC::Maneuver> pm_data = pm->data;
				// another friggin inline message...
				// all i wanted was to get x,y,z from plandb to ros. 
				// now i am 3 indents in, still no x,y,z in sight.
				// i hate imc at this point.
				// and its too hot, im melting
				if(pm_data.isNull()){
					// there aint anything in it...
				}else{
					// there is a maneuver in it, at least it is defined this time
					// but this maneuver has different concrete implementations ........ASDJAHSJKHD
					// so... which one is THIS one?
					int man_id = pm_data.get()->getId();
					std::cout << "Maneuver:" << man_id << std::endl;
					// 450==Goto
					if(man_id==450){
						IMC::Goto* goto_man = (IMC::Goto*) pm_data.get();
						plan_maneuver.maneuver.maneuver_name = "goto";
						plan_maneuver.maneuver.maneuver_imc_id = man_id;

						// AND WE FINALLY GET SOME NUMBEEEEEERSSSSSS
						plan_maneuver.maneuver.timeout = goto_man->timeout;
						plan_maneuver.maneuver.lat = goto_man->lat;
						plan_maneuver.maneuver.lon = goto_man->lon;
						plan_maneuver.maneuver.z = goto_man->z;
						plan_maneuver.maneuver.z_units = goto_man->z_units;
						plan_maneuver.maneuver.speed = goto_man->speed;
						plan_maneuver.maneuver.speed_units = goto_man->speed_units;
						plan_maneuver.maneuver.roll = goto_man->roll;
						plan_maneuver.maneuver.pitch = goto_man->pitch;
						plan_maneuver.maneuver.yaw = goto_man->yaw;
						plan_maneuver.maneuver.custom_string = goto_man->custom;
					} // man_id=450

					
				}
				// done with creating the plan_maneuver ros message, list it.
				maneuvers.push_back(plan_maneuver);
			}
			// and assign to ros message, finally.
			ros_msg.plan_spec.maneuvers.resize(maneuvers.size());
			int i=0;
			for(imc_ros_bridge::PlanManeuver const &pm: maneuvers){
				ros_msg.plan_spec.maneuvers[i++] = pm;
			}

		} // msg_id=551
	} //else
	






	std::cout << std::endl << "Ros message to send:" << std::endl;
	std::cout << ros_msg << std::endl;
	return true;


}


} // namespace imc_to_ros
