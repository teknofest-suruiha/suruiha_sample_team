#!/usr/bin/env python
import roslib

roslib.load_manifest('sample_team')
import rospy

from geometry_msgs.msg import Twist, Pose
from suruiha_gazebo_plugins.srv import AirTraffic
from suruiha_gazebo_plugins.msg import UAVMessage
from suruiha_gazebo_plugins.msg import UAVSensorMessage
from sample_team.air_traffic_manager import AirTrafficManager
from sample_team.iris_controller import IrisController
from sample_team.task_planner import TaskPlanner
from sample_team.comm_manager import CommManager
from sample_team.sensor_manager import SensorManager


if __name__ == "__main__":
    rospy.init_node('quadrotor_controller', anonymous=True)
    uav_name = rospy.get_param('~name')
    air_manager = AirTrafficManager(uav_name)

    iris_controller = IrisController(uav_name, air_manager)
    comm_manager = CommManager(uav_name, iris_controller)
    sensor_manager = SensorManager(uav_name)

    task_planner = TaskPlanner(iris_controller, sensor_manager, uav_name)

    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # air_manager.step()
        # comm_manager.step()
        task_planner.step()

        # share the latest pose of the uav with other uavs
        comm_manager.publish_pose()
        # uav_controller.step()
        ros_rate.sleep()

        # print 'current throttle:', str(throttle), ' pitch:', str(pitch), ' roll:', str(roll)
        # control_cmd = Twist()
        # control_cmd.linear.z = throttle  # for iris quadrotor
        # control_cmd.linear.x = throttle  # for zephyr fixeed wing
        # control_cmd.angular.y = pitch
        # control_cmd.angular.x = roll
        # pub.publish(control_cmd)
