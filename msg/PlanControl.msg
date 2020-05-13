# IMC PlanControl message in ROS form.



# enums
uint8 type #0,1,2,3 -> request, success, failure, in progress
uint8 op # 0,1,2,3 -> start, stop, load, get

# fields
uint16 request_id
string plan_id
uint16 flags
string info




