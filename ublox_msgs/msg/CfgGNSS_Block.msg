# See message CfgGNSS

uint8 gnssId            # GNSS identifier (select from following list)
uint8 resTrkCh          # Number of reserved (minimum) tracking channels for this GNSS system
uint8 maxTrkCh          # Maximum number of tracking channels (>=resTrkChn)
uint8 reserved1         # Reserved
uint32 flags            # Bitfield of flags (bit 0 = enable/disable)

