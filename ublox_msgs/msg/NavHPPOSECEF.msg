# NAV-HPPOSECEF (0x01 0x13)
# High precision geodetic position/solution in ECEF
#
# std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 19

uint8 version
uint8[3] reserved1
uint32 iTOW
int32 ecefX
int32 ecefY
int32 ecefZ
int8 ecefXHp
int8 ecefYHp
int8 ecefZHp
uint8 reserved2
uint32 pAcc

