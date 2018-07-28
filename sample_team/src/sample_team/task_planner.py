# task status
TAKEOFF = 'TAKEOFF'
WAYPOINTS = 'WAYPOINTS'
LAND = 'LAND'
IDLE = 'IDLE'


class TaskPlanner:

    def __init__(self, uav_controller, sensor_manager, uav_name):
        self.controller = uav_controller
        self.sensor_manager = sensor_manager
        self.uav_name = uav_name
        self.status = TAKEOFF

        self.waypoint_counter = 0
        self.way_points = [[0, 500, 50]]

    def step(self):
        if self.status == TAKEOFF:
            # print('task planner takeoff')
            completed = self.controller.takeoff(30)
            if completed:
                self.status = WAYPOINTS
                self.waypoint_counter = 0
        elif self.status == WAYPOINTS:
            print('task planner waypoints')
            way_point = self.way_points[self.waypoint_counter]
            completed = False
            if self.uav_name.find('zephyr') >= 0:
                completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 400)
            elif self.uav_name.find('iris') >= 0:
                completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 440)
            if completed:
                self.waypoint_counter += 1
                if self.waypoint_counter == len(self.way_points):
                    self.status = LAND
        elif self.status == LAND:
            completed = self.controller.land()
            if completed:
                self.status = IDLE
        elif self.status == IDLE:
            print('idle state')
            self.controller.stop_motors()
            # pass
            # do nothing





