#!/usr/bin/env python
import roslib

roslib.load_manifest('sample_team')
import rospy
import sys
from sample_team.terrorist_detector import TerroristDetector
from sample_team.air_traffic_manager import AirTrafficManager
from sample_team.zephyr_controller import ZephyrController
from sample_team.iris_controller import IrisController
from sample_team.task_planner import TaskPlanner
from sample_team.comm_manager import CommManager
from sample_team.sensor_manager import SensorManager
from sample_team.score_manager import ScoreManager
import sample_team.scenario as scenario


if __name__ == "__main__":
    rospy.init_node('uav_controller', anonymous=True)
    uav_index = int(sys.argv[1])
    scenario.get_scenario_params()

    # ornek takim kodu icin tek indeks sayısı icin iris cift sayısı icin zephyr modeli seciliyo
    uav_name = 'iris' + str(uav_index)
    if uav_index % 2 == 0:
        uav_name = 'zephyr' + str(uav_index)

    air_manager = AirTrafficManager(uav_name)

    controller = None
    if uav_name.find('zephyr') >= 0:
        controller = ZephyrController(uav_name, air_manager)
    elif uav_name.find('iris') >= 0:
        controller = IrisController(uav_name, air_manager)

    comm_manager = CommManager(uav_name, controller)
    sensor_manager = SensorManager()
    task_planner = TaskPlanner(controller, uav_name)
    terrorist_detector = TerroristDetector(sensor_manager)
    score_manager = ScoreManager()

    # how many times in a second the control loop is going to run
    ros_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        score_manager.step()
        terrorist_detector.step()
        task_planner.step()
        # share the latest pose of the uav with other uavs
        comm_manager.publish_pose()
        ros_rate.sleep()
