# CFG-TMODE2 (0x06 0x3d)
# Time Mode Settings 3
# Configures the receiver to be in Time Mode.

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 61

uint8 timeMode
uint8 reserved
uint16 flags
int32 ecefXOrLat
int32 ecefYOrLon
int32 ecefZOrAlt
uint32 fixedPosAcc
uint32 svinMinDur
uint32 svinAccLimit

uint8 TIMEMODE_DISABLED = 0
uint8 TIMEMODE_SURVEYIN = 1
uint8 TIMEMODE_FIXEDMODE = 2

uint16 FLAGS_LLA = 1
uint16 FLAGS_ALTINV = 2
