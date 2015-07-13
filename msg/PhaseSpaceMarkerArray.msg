# Array of PhaseSpaceMarker messages with a header for global reference

std_msgs/Header header

# Acquisition time
duration acquisition_time
# Number of markers acquired
int32 num_acquired_makers

# Marker position data
PhaseSpaceMarker[] data
