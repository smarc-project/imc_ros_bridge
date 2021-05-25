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

#include <list>
#include <md5.h>

#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <imc_ros_bridge/PlanSpecification.h>
#include <imc_ros_bridge/PlanManeuver.h>
#include <imc_ros_bridge/PolygonVertex.h>

#include <IMC/Base/InlineMessage.hpp>
#include <IMC/Base/Message.hpp>
#include <IMC/Base/MessageList.hpp>
// #include <IMC/Base/Packet.hpp> this has the IMC::serialize function in it.

#include <IMC/Spec/PlanManeuver.hpp>
#include <IMC/Spec/PlanSpecification.hpp>
#include <IMC/Spec/Goto.hpp>
#include <IMC/Spec/Sample.hpp>
#include <IMC/Spec/PolygonVertex.hpp>
#include <IMC/Spec/CoverArea.hpp>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanDB& imc_msg, imc_ros_bridge::PlanDB& ros_msg)
{
	ros_msg.type = imc_msg.type;
	ros_msg.op = imc_msg.op;
	ros_msg.request_id = imc_msg.request_id;
	ros_msg.plan_id = imc_msg.plan_id;

	IMC::InlineMessage<IMC::Message> arg = imc_msg.arg;
	if(arg.isNull()){
		// arg is empty, 
		// maybe this is a control message or sth.
	}else{
		int arg_msg_id = arg.get()->getId();

		// 551 is PlanSpecification
		if(arg_msg_id == 551){
			ros_msg.plan_spec = imc_ros_bridge::PlanSpecification();
			// cast it to its proper type finally
			// arg.get() returns a Message*, cast that pointer to a pointer to a PlanSpec because we KNOW
			// it is actually pointing to a real PlanSpec object, thanks to the id of the message.
			const IMC::PlanSpecification* plan_spec = (IMC::PlanSpecification*) arg.get();

			// MD5 OF PLAN_SPEC
			//
			// See this before you start here:
			// https://github.com/LSTS/imcjava/blob/efed1384cc0cc5bacb8ce8ec7bcd536d19c91c8c/src/pt/lsts/imc/IMCMessage.java#L1592
			//
			// Get its serialized form, this will be used to calculate the md5
			// Doing it here allows us to use the serialization
			// as defined in IMC, because it is done in a recursive way that I really dont want to
			// replicate on the receiving end
			// First find the size of the serialized message
			const size_t serialization_size = plan_spec->getSerializationSize();
			// then allocate a buffer to put the message in
			uint8_t plan_spec_serialized[serialization_size] = {0};

			// and fill it in
			// BUT WITH WHAT?! NEITHER OF THESE CREATE THE SAME MD5 AS NEPTUS
			// IMC::serialize(plan_spec, plan_spec_serialized);
			// It should really be this one, since this is the function used in the Java version
			plan_spec->serializeFields(plan_spec_serialized);

			// Create an empty md5 object because we want to use the
			// update() function which takes bytes as input
			// Neptus requires RAW md5, not a hex version of one.
			// that RAW data is the digest. Use that.
			MD5 md5_o = MD5();
			md5_o.update(plan_spec_serialized, serialization_size);
			md5_o.finalize();
			auto digest = md5_o.get_digest();

			// But then, ROS messages expect std::vectors and not arrays
			// so we just fill it in
			// digest size is fixed to 16
			std::cout << "(unsigned) md5 digest of PlanSpecification:";
			for(int i=0; i<16; i++){
				ros_msg.plan_spec_md5.push_back(digest[i]);
				// unsigned() because the digest has invis. characters in it.
				// this way they just become readable ints
				std::cout << unsigned(digest[i]);
				std::cout << " ";
			}
			std::cout << std::endl;
			// DONE WITH MD5 OF PLAN_SPEC


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
					}
					// 489==Sample
					else if(man_id==489){
						IMC::Sample* sample_man = (IMC::Sample*) pm_data.get();
						plan_maneuver.maneuver.maneuver_name = "sample";
						plan_maneuver.maneuver.maneuver_imc_id = man_id;

						plan_maneuver.maneuver.timeout = sample_man->timeout;
						plan_maneuver.maneuver.lat = sample_man->lat;
						plan_maneuver.maneuver.lon = sample_man->lon;
						plan_maneuver.maneuver.z = sample_man->z;
						plan_maneuver.maneuver.z_units = sample_man->z_units;
						plan_maneuver.maneuver.speed = sample_man->speed;
						plan_maneuver.maneuver.speed_units = sample_man->speed_units;
						// hardcoded 3 syringes... in the spec! BAH!
						plan_maneuver.maneuver.syringe0 = sample_man->syringe0;
						plan_maneuver.maneuver.syringe1 = sample_man->syringe1;
						plan_maneuver.maneuver.syringe2 = sample_man->syringe2;
						plan_maneuver.maneuver.custom_string = sample_man->custom;

					}
					//  473==CoverArea
					else if(man_id==473){
						IMC::CoverArea* cover_area = (IMC::CoverArea*) pm_data.get();
						plan_maneuver.maneuver.maneuver_name = "cover_area";
						plan_maneuver.maneuver.maneuver_imc_id = man_id;

						plan_maneuver.maneuver.lat = cover_area->lat;
						plan_maneuver.maneuver.lon = cover_area->lon;
						plan_maneuver.maneuver.z = cover_area->z;
						plan_maneuver.maneuver.z_units = cover_area->z_units;
						plan_maneuver.maneuver.speed = cover_area->speed;
						plan_maneuver.maneuver.speed_units = cover_area->speed_units;

						// ros_msg.polygon = imc_msg.polygon
						for(IMC::PolygonVertex* imc_pv : cover_area->polygon){
							auto ros_pv = imc_ros_bridge::PolygonVertex();
							ros_pv.lat = imc_pv->lat;
							ros_pv.lon = imc_pv->lon;
							plan_maneuver.maneuver.polygon.push_back(ros_pv);
						}

					}
					else{
						std::cout << "Maneuver not implemented! id:" << man_id << std::endl;
					}

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
		else{
			// Not implemented
			std::cout << "imc to ros:planDB arg not implemented! id:" << arg_msg_id << std::endl;
		}
	} //else







	// std::cout << std::endl << "Ros message to send:" << std::endl;
	// std::cout << ros_msg << std::endl;
	return true;


}


} // namespace imc_to_ros
