# task status
TAKEOFF = 'TAKEOFF'
WAYPOINTS = 'WAYPOINTS'
LAND = 'LAND'
IDLE = 'IDLE'

class TaskPlanner:

    def __init__(self, uav_controller, sensor_manager):
        self.controller = uav_controller
        self.sensor_manager = sensor_manager
        self.status = TAKEOFF

        self.waypoint_counter = 0
        self.way_points = [[0, 500, 50]]

    def step(self):
        if self.status == TAKEOFF:
            # print('task planner takeoff')
            completed = self.controller.takeoff(50)
            if completed:
                self.status = WAYPOINTS
                self.waypoint_counter = 0
        elif self.status == WAYPOINTS:
            print('task planner waypoints')
            way_point = self.way_points[self.waypoint_counter]
            completed = self.controller.goto_position(way_point[0], way_point[1], way_point[2], 400)
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
            pass
            # do nothing





