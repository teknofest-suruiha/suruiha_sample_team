import rospy
from suruiha_gazebo_plugins.msg import UAVTracking
from std_msgs.msg import String


class TerroristDetector:
    def __init__(self, sensor_manager):
        self.terrorist_tracking_publisher = rospy.Publisher('/terrorist_tracking', UAVTracking, queue_size=1)
        self.terrorist_detection_publisher = rospy.Publisher('/terrorist_detection', String, queue_size=1)
        self.sensor_manager = sensor_manager

    def step(self):
        perception = self.sensor_manager.get_last_perception()
        trackingMsg = UAVTracking()
        for i in range(len(perception.types)):
            if perception.names[i].find('terrorist') >= 0:
                trackingMsg.names.append(perception.names[i])
                trackingMsg.poses.append(perception.poses[i])

        if len(trackingMsg.names) > 0:
            self.terrorist_tracking_publisher.publish(trackingMsg)

        if len(trackingMsg.names) > 0:
            detectionMsg = String()
            detectionMsg.data = 'building_1'
            self.terrorist_detection_publisher.publish(detectionMsg)