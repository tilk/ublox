# NAV-SAT (0x01 0x35)
# Satellite Information
#
#std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 53

uint32 iTOW             # GPS Millisecond time of week [ms]

uint8 version
uint8 numSvs
uint8[2] reserved1

NavSAT_SV[] sv

