# CFG-ODO (0x06 0x1E)
# Odometer, Low-speed COG Engine Settings

uint8 CLASS_ID = 6
uint8 MESSAGE_ID = 30

uint8 version
uint8[3] reserved1
uint8 flags
uint8 FLAGS_USE_ODO = 1
uint8 FLAGS_USE_COG = 2
uint8 FLAGS_OUT_LP_VEL = 3
uint8 FLAGS_OUT_LP_COG = 4
uint8 odoCfg
uint8 ODOCFG_PROFILE_MASK = 7
uint8 ODOCFG_PROFILE_RUNNING = 0
uint8 ODOCFG_PROFILE_CYCLING = 1
uint8 ODOCFG_PROFILE_SWIMMING = 2
uint8 ODOCFG_PROFILE_CAR = 3
uint8 ODOCFG_PROFILE_CUSTOM = 4
uint8[6] reserved2
uint8 cogMaxSpeed
uint8 cogMaxPosAcc
uint8[2] reserved3
uint8 velLpGain
uint8 cogLpGain
uint8[2] reserved4
