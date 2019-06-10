# IMC PlanControlState message in ROS form.

uint8 state
# state enums
uint8 BLOCKED = 0
uint8 READY = 1
uint8 INITIALIZING = 2
uint8 EXECUTING = 3


string plan_id
int32 plan_eta
# percent
float32 plan_progress

string man_id
uint16 man_type
int32 man_eta


uint8 last_outcome
# last outcome enums
uint8 NONE = 0
uint8 SUCCESS = 1
uint8 FAILURE = 2




