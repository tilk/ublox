# NAV-RELPOSNED (0x01 0x3c)
# Relative positioning information in NED frame
#
# std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 60

uint8 version
uint8 reserved1
uint16 refStationId
uint32 iTOW
int32 relPosN
int32 relPosE
int32 relPosD
int8 relPosHPN
int8 relPosHPE
int8 relPosHPD
uint8 reserved2
uint32 accN
uint32 accE
uint32 accD
uint32 flags

uint32 FLAGS_GNSSFIXOK = 1
uint32 FLAGS_DIFFSOLN = 2
uint32 FLAGS_RELPOSVALID = 4
uint32 FLAGS_CARRSOLN_MASK = 24
uint32 FLAGS_CARRSOLN_NO = 0
uint32 FLAGS_CARRSOLN_FLOAT = 8
uint32 FLAGS_CARRSOLN_FIXED = 16

