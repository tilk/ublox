# NAV-SVIN (0x01 0x3b)
# Survey-in data
#
# std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 59

uint8 version
uint8[3] reserved1
uint32 iTOW
uint32 dur              # Passed survey-in observation time
int32 meanX             # Survey-in mean ECEF
int32 meanY
int32 meanZ
int8 meanXHP            # Survey-in precision
int8 meanYHP
int8 meanZHP
uint8 reserved2
uint32 meanAcc
uint32 obs
uint8 valid
uint8 active
uint8[2] reserved3

