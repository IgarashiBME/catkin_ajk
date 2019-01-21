import numpy as np

x1 = 1
y1 = 1
x2 = 4 
y2 = 4

cx2 = x2 - x1
cy2 = y2 - y1 

theta = np.arctan2(cy2, cx2)
print theta/np.pi *180

tx2 = cx2*np.cos(-theta) - cy2*np.sin(-theta)
ty2 = cx2*np.sin(-theta) + cy2*np.cos(-theta)

print tx2, ty2 
