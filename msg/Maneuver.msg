# Z Units.
# None.
uint8 Z_NONE = 0
# Depth.
uint8 Z_DEPTH = 1
# Altitude.
uint8 Z_ALTITUDE = 2
# Height.
uint8 Z_HEIGHT = 3

# Speed Units.
# Meters per second.
uint8 SUNITS_METERS_PS = 0
# RPM.
uint8 SUNITS_RPM = 1
# Percentage.
uint8 SUNITS_PERCENTAGE = 2

# many IMC maneuvers should fit in this message
# and we should just decide what to do with the data
# depending on the maneuver name or id
# i will take the IMC/Spec/Goto maneuver as my basis
string maneuver_name
uint16 maneuver_imc_id

# 450 GOTO
float64 lat
float64 lon
float32 z
uint8 z_units
float32 speed
uint8 speed_units

float32 roll
float32 pitch
float32 yaw

uint16 timeout
string custom_string

# 489 SAMPLE that are not in GOTO
uint8 syringe0
uint8 syringe1
uint8 syringe2

# 473 COVER_AREA that are not in GOTO
PolygonVertex[] polygon


