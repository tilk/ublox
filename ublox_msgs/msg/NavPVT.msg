# NAV-PVT (0x01 0x07)
# Position Velocity Time solution
#
# std_msgs/Header header
uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 iTOW             # GPS Millisecond Time of Week [ms]

uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 valid                     # Validity flags
uint8 VALID_DATE = 1            # Valid UTC date
uint8 VALID_TIME = 2            # Valid UTC Time of Day
uint8 VALID_FULLYRESOLVED = 4   # UTC Time of Day fully resolved

uint32 tAcc             # Time accuracy estimate
int32 nano              # Fraction of second, range -1e9..1e9 (UTC)

uint8 fixType           # Fix type
uint8 GPS_NO_FIX = 0
uint8 GPS_DEAD_RECKONING_ONLY = 1
uint8 GPS_2D_FIX = 2
uint8 GPS_3D_FIX = 3
uint8 GPS_GPS_DEAD_RECKONING_COMBINED = 4
uint8 GPS_TIME_ONLY_FIX = 5

uint8 flags             # Fix Status flags
uint8 FLAGS_GNSSFIXOK = 1           # i.e. within DOP & ACC Masks
uint8 FLAGS_DIFFSOLN = 2            # DGPS used
uint8 FLAGS_PSM_NOT_ACTIVE = 0      # PSM not active
uint8 FLAGS_PSM_ENABLED = 4
uint8 FLAGS_PSM_ACQUISITION = 8
uint8 FLAGS_PSM_TRACKING = 12
uint8 FLAGS_PSM_OPT_TRACKING = 16
uint8 FLAGS_PSM_INACTIVE = 20
uint8 FLAGS_HEADVEHVALID = 32       # heading of vehicle is valid
uint8 FLAGS_CARRSOLN_NO = 64        # no carrier phase range solution
uint8 FLAGS_CARRSOLN_FLOAT = 128    # float solution
uint8 FLAGS_CARRSOLN_FIXED = 192    # fixed solution

uint8 flags2            # Additional flags
uint8 FLAGS2_CONFIRMED_AVAI = 32    # Information about UTC Date and Time of Day validity confirmation is available
uint8 FLAGS2_CONFIRMED_DATE = 64    # UTC Date validity could be confirmed
uint8 FLAGS2_CONFIRMED_TIME = 128   # UTC Time of Day could be confirmed

uint8 numSV             # Number of satellites used
int32 lon               # Longitude
int32 lat               # Latitude
int32 height            # Height above ellipsoid
int32 hMSL              # Height above mean sea level (mm)
uint32 hAcc             # Horizontal accuracy estimate (mm)
uint32 vAcc             # Vertical accuracy estimate (mm)
int32 velN              # NED north velocity (mm/s)
int32 velE              # NED east velocity (mm/s)
int32 velD              # NED down velocity (mm/s)
int32 gSpeed            # Ground speed (2-D, mm/s)
int32 headMot           # Heading of motion (2-D, deg)
uint32 sAcc             # Speed accuracy estimate
uint32 headAcc          # Heading accuracy estimate
uint16 pDOP             # position DOP
uint8[6] reserved1
int16 headVeh           # Heading of vehicle (2-D, deg)
uint8[4] reserved2

