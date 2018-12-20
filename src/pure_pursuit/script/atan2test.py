import numpy as np

#xy_yaw = np.arctan2(368003.878079-368003.837598 ,3955745.78778-3955744.9446)/np.pi*180
#yx_yaw = np.arctan2(3955745.78778-3955744.9446, 368003.878079-368003.837598)/np.pi*180
yx_yaw = np.arctan2(3955747.93715-3955747.94109, 368003.562834-368002.772338)/np.pi*180
xy_yaw = np.arctan2(-368003.562834+368002.772338,-3955747.93715+3955747.94109)/np.pi*180

if xy_yaw < 0:
    xy_yaw = xy_yaw +360
if yx_yaw < 0:
    yx_yaw = yx_yaw +360

print yx_yaw, xy_yaw
