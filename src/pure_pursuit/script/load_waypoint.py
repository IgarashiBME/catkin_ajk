import os
import csv
import numpy as np
from scipy.interpolate import interp1d

ki = 0.10 # interpolate coefficient, unit is meter

def load_csv():
    target_x = []
    target_y = []
    csvdir = os.path.abspath(os.path.dirname(__file__))
    os.chdir(csvdir)
    f = open('route.csv', 'r')
    reader = csv.reader(f)

    line_count = 0
    for row in reader:
        target_x.append(float(row[0]))
        target_y.append(float(row[1]))
    return target_x, target_y

def interpolation(target_x, target_y, spacing):
    waypoint_x = []
    waypoint_y = []
    waypoint_goal = []
    for i in range(len(target_x)-1):
        x0 = target_x[i]
        y0 = target_y[i]
        x1 = target_x[i+1]
        y1 = target_y[i+1]

        a = np.array([x0, y0])
        b = np.array([x1, y1])
        waypoint_dist = np.abs(np.linalg.norm(b-a))

        if abs(x1-x0) > abs(y1-y0):
            f = interp1d([x0,x1], [y0,y1])
            x = np.linspace(x0, x1, int(abs(x1-x0)/ki))
            y = f(x)
        elif abs(x1-x0) < abs(y1-y0):
            f = interp1d([y0,y1], [x0,x1])
            y = np.linspace(y0, y1, int(abs(y1-y0)/ki))
            x = f(y)
        waypoint_x = np.append(waypoint_x, x)
        waypoint_y = np.append(waypoint_y, y)

        # set waypoint goal flag
        for i in range(len(x)):
            if x[i] == x1 and y[i] == y1 and waypoint_dist > spacing:
                waypoint_goal = np.append(waypoint_goal, 1)
            else:
                waypoint_goal = np.append(waypoint_goal, 0)
    #for i in range(waypoint_x.shape[0]):
    #    print waypoint_x[i],",", waypoint_y[i]
    return waypoint_x, waypoint_y, waypoint_goal
