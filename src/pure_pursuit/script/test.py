import load_waypoint

target_x, target_y = load_waypoint.load_csv()
print target_x, target_y

waypoint_x, waypoint_y, waypoint_goal = load_waypoint.interpolation(target_x, target_y)

for i, x in enumerate(waypoint_x):
    print x, waypoint_y[i], waypoint_goal[i]
