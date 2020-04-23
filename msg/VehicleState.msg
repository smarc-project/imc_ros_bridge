# IMC VehicleState message in ROS form.


#enums
# op mode
uint8 SERVICE       = 0
uint8 CALIBRATION   = 1
uint8 ERROR         = 2
uint8 MANEUVER      = 3
uint8 EXTERNAL      = 4
uint8 BOOT          = 5


# fields
uint8 op_mode           # Operation Mode.
uint8 error_count       # Errors   -- Count.
string error_ents       # Errors   -- Entities.
uint16 maneuver_type    # Maneuver -- Type.
float64 maneuver_stime  # Maneuver -- Start Time.
uint16 maneuver_eta     # Maneuver -- ETA.
uint32 control_loops    # Control Loops.
uint8 flags             # Flags.
string last_error       # Last Error -- Description.
float64 last_error_time # Last Error -- Time.
