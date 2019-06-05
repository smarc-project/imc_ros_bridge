#include <imc_ros_bridge/imc_to_ros/PlanDB.h>
#include <sstream>

namespace imc_to_ros {

template <>
bool convert(const IMC::PlanDB& imc_msg, std_msgs::String& ros_msg)
{
    std::stringstream ostr;
    imc_msg.fieldsToJSON(ostr, 0);
    ros_msg.data = ostr.str();
    std::cout << "Ros message to send:" << std::endl;
    std::cout << ros_msg.data << std::endl;
    return true;
}

} // namespace imc_to_ros
