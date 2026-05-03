# Georeferenced aerial-image metadata from a completed drone mapping mission.
# Contract: ICD-MAP-002. Producer: georeferencer (SRS-MAP-003).

std_msgs/Header header

# Absolute path on rover disk; mission_api resolves this internally and
# does not expose it directly to the operator app.
string image_path

# Drone pose at shutter (FC frame, WGS84).
float64 capture_lat
float64 capture_lon
float32 capture_alt_m
geometry_msgs/Quaternion capture_attitude

# Image-corner geographic coordinates in NE / SE / SW / NW order.
# Computed under the flat-ground assumption documented in ICD-MAP-002 §4.
float64[4] corner_lats
float64[4] corner_lons

# FC timestamp at shutter (separate from header.stamp, which carries
# georeferencer wall-clock at publication).
builtin_interfaces/Time capture_time
