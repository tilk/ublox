# CFG-TMODE3 (0x06 0x71)
# Time Mode Settings 3
# Configures the receiver to be in Time Mode.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 113

uint8 version
uint8 reserved1
uint16 flags
int32 ecefXOrLat
int32 ecefYOrLon
int32 ecefZOrAlt
int8 ecefXOrLatHP
int8 ecefYOrLonHP
int8 ecefZOrAltHP
uint8 reserved2
uint32 fixedPosAcc
uint32 svinMinDur
uint32 svinAccLimit
uint8[8] reserved3

uint16 FLAGS_MODE_MASK = 255
uint16 FLAGS_MODE_DISABLED = 0
uint16 FLAGS_MODE_SURVEYIN = 1
uint16 FLAGS_MODE_FIXEDMODE = 2
uint16 FLAGS_LLA = 256

