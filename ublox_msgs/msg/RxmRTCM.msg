# RXM-RTCM (0x02 0x32)
# RTCM input status
std_msgs/Header header
uint8 CLASS_ID = 2
uint8 MESSAGE_ID = 50

uint8 version
uint8 flags
uint8[2] reserved1
uint16 refStation
uint16 msgType

