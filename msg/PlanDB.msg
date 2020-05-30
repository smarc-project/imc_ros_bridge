# Type enums
uint8 REQUEST = 0
uint8 SUCCESS = 1
uint8 FAILURE = 2
uint8 IN_PROGESS = 3

# Operation enums
uint8 SET = 0
uint8 DEL = 1
uint8 GET = 2
uint8 GET_INFO = 3
uint8 CLEAR = 4
uint8 GET_STATE = 5
uint8 GET_DSTATE = 6
uint8 BOOT = 7

uint8 type
uint8 op
uint16 request_id
string plan_id

# this part is special made for just waypoint type plan_dbs. ill handle other types later.
# this is the Message arg in the original
PlanSpecification plan_spec
uint8[] plan_spec_md5
# might be other types of fields that come into 'arg'. In IMC, this arg is  a "message" type, so who knows what it can become...
# Apparently, arg can also be a PlanDBInformation message. 
PlanDBInformation plandb_information
PlanDBState plandb_state


