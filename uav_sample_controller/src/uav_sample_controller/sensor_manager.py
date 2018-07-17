from suruiha_gazebo_plugins.msg import UAVSensorMessage


class SensorManager:
    def __init__(self, rospy):
        sensor_topic_name = rospy.get_param('~sensor')
        rospy.Subscriber(sensor_topic_name, UAVSensorMessage, self.message_received)
        self.last_perception = UAVSensorMessage()

    def message_received(self, sensor_msg):
        self.last_perception = sensor_msg

    def get_last_perception(self):
        return self.last_perception
