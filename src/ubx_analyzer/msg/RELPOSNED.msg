# NAV-RELPOSNED (0x01 0x3C)

uint32 iTOW             # GPS Millisecond time of week [ms]
uint8 fix_status        # 2:fixed, 1:float, 0:no carrier phase solution
float64 relPosN         # North component of relative position[cm]
float64 relPosE         # East component of relative position[cm]
float64 relPosD         # Down component of relative position[cm]
float64 relPosHPN       # High-precision North component [cm]
float64 relPosHPE       # High-precision East component [cm]
float64 relPosHPD       # High-precision Down component [cm]
float64 accN            # Accuracy of relative position North component [mm]
float64 accE            # Accuracy of relative position East component [mm]
float64 accD            # Accuracy of relative position Down component [mm]
float64 heading_rad     # calculate heading using NE component [radian]
float64 heading_deg     # calculate heading using NE component [degree]
float64 QGC_heading     # right-handed, Z-down, X-front, Y-right [radian]

