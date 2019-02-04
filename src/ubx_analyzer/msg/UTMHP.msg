# NAV-HPPOSLLH (0x01 0x14)

uint32 iTOW             # GPS Millisecond time of week [ms]
uint8 numSV             # Number of SVs used in Nav Solution
uint8 fix_status        # 2:fixed, 1:float, 0:no carrier phase solution
float64 lonHp               # Longitude [deg / 1e-7]
float64 latHp               # Latitude [deg / 1e-7]
float64 utm_easting
float64 utm_northing
float64 heightHp            # Height above Ellipsoid [mm]
uint32 hAcc             # Horizontal Accuracy Estimate [mm]
uint32 vAcc             # Vertical Accuracy Estimate [mm]

