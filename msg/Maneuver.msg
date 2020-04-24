# many IMC maneuvers should fit in this message
# and we should just decide what to do with the data
# depending on the maneuver name or id
# i will take the IMC/Spec/Goto maneuver as my basis
string maneuver_name
uint16 maneuver_imc_id

uint16 timeout
float64 lat
float64 lon
float32 z
uint8 z_units
float32 speed
uint8 speed_units
float32 roll
float32 pitch
float32 yaw
string custom_string
