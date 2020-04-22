# IMC SonarData message in ROS form.


uint8 ST_SIDESCAN     = 0    # Sidescan.
uint8 ST_ECHOSOUNDER  = 1    # Echo Sounder.
uint8 ST_MULTIBEAM    = 2    # Multibeam.

uint8 type                            # Type.
uint32 frequency                      # Frequency.
uint16 min_range                      # Minimum Range.
uint16 max_range                      # Maximum Range.
uint8 bits_per_point                  # Bits Per Data Point.
float32 scale_factor                  # Scaling Factor.
# TODO add beamconfig
uint8[] data                          # Data. first half of vector is left side, other half is right side of side scan

#MessageList<BeamConfig> beam_config  # Beam Configuration.
