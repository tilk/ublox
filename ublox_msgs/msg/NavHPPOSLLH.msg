# NAV-HPPOSLLH (0x01 0x14)
# High precision geodetic position/solution
#
# std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 20

uint8 version
uint8[3] reserved1
uint32 iTOW
int32 lon
int32 lat
int32 height
int32 hMSL
int8 lonHp
int8 latHp
int8 heightHp
int8 hMSLHp
uint32 hAcc
uint32 vAcc

