# Unified safety state from safety_monitor (SRS-SAF-001).
# Contract: ICD-SAF-001.

std_msgs/Header header

# State enum. Worst trigger wins.
uint8 STATE_OK = 0
uint8 STATE_WARNING = 1
uint8 STATE_ESTOP = 2
uint8 STATE_CRITICAL = 3
uint8 state

# Human-readable list of currently-active or latched triggers (e.g.
# "tilt_exceeded", "topic_stale:/hal/imu/data"). Empty when state=OK.
string[] reasons

# Permission flags. INV-1: tool_permitted ⇒ state ∈ {OK, WARNING}.
# CRITICAL is the documented exception that allows motion (to reach safe stop)
# while blocking the tool.
bool motion_permitted
bool tool_permitted

# True iff the current state could clear without /safety/reset (i.e. no
# currently-cleared latched conditions are blocking auto-recovery).
bool clearable
