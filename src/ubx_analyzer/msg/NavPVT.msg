# NAV-PVT (0x01 0x07)
# Navigation Position Velocity Time Solution
#
# Note that during a leap second there may be more (or less) than 60 seconds in
# a minute; see the description of leap seconds for details.
#
# This message combines Position, velocity and time solution in LLH, 
# including accuracy figures
#
# WARNING: For firmware version 7, this message is a different length.
#

uint8 CLASS_ID = 1
uint8 MESSAGE_ID = 7

uint32 iTOW             # GPS Millisecond time of week [ms]
uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)

uint8 numSV             # Number of SVs used in Nav Solution
uint8 fix_status        # 2:fixed, 1:float, 0:no carrier phase solution
float64 lon               # Longitude [deg / 1e-7]
float64 lat               # Latitude [deg / 1e-7]
float64 height            # Height above Ellipsoid [mm]
int32 hMSL              # Height above mean sea level [mm]
uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]

int32 velN              # NED north velocity [mm/s]
int32 velE              # NED east velocity [mm/s]
int32 velD              # NED down velocity [mm/s]
int32 gSpeed            # Ground Speed (2-D) [mm/s]
int32 headMot           # Heading of motion 2-D [deg / 1e-5]
uint32 sAcc             # Speed Accuracy Estimate [mm/s]
uint32 headAcc          # Heading Accuracy Estimate (both motion & vehicle) 

uint16 pDOP             # Position DOP [1 / 0.01]
int32 headVeh           # Heading of vehicle (2-D) [deg / 1e-5]
