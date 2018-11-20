import load_waypoint

target_x, target_y = load_waypoint.load_csv()
print target_x, target_y

waypoint_x, waypoint_y = load_waypoint.interpolation(target_x, target_y)
print waypoint_x, waypoint_y
