from suruiha_gazebo_plugins.msg import UAVMessage
from geometry_msgs.msg import Pose
import rospy

# MESSAGE TYPES
POSE_MSG = 0


class CommManager:
    def __init__(self, uav_name, uav_controller):
        self.uav_name = uav_name
        self.msg_publisher = rospy.Publisher('comm_request', UAVMessage, queue_size=10)
        rospy.Subscriber('comm_' + self.uav_name, UAVMessage, self.message_received)
        self.uav_controller = uav_controller
        self.teammate_poses = {}

    def message_received(self, msg):
        msg_data_fields = msg.msg.split(' ')
        msg_type = int(msg_data_fields[0])
        if msg_type == POSE_MSG:
            pose = Pose()
            pose.position.x = float(msg_data_fields[1])
            pose.position.y = float(msg_data_fields[2])
            pose.position.z = float(msg_data_fields[3])
            pose.orientation.x = float(msg_data_fields[4])
            pose.orientation.y = float(msg_data_fields[5])
            pose.orientation.z = float(msg_data_fields[6])
            pose.orientation.w = float(msg_data_fields[7])
            self.teammate_poses[msg.sender] = pose
        else:
            print('Unknown message type:' + str(msg_type))

        print('comm_manager msg received from:' + msg.sender + ' msg:' + msg.msg)

    def publish_pose(self):
        msg_string_list = []
        pose = self.uav_controller.get_latest_pose()
        msg_string_list.append(str(POSE_MSG) + ' ')
        msg_string_list.append(str(pose.position.x) + ' ')
        msg_string_list.append(str(pose.position.y)+ ' ')
        msg_string_list.append(str(pose.position.z)+ ' ')
        msg_string_list.append(str(pose.orientation.x)+ ' ')
        msg_string_list.append(str(pose.orientation.y)+ ' ')
        msg_string_list.append(str(pose.orientation.z)+ ' ')
        msg_string_list.append(str(pose.orientation.w))
        uav_msg = UAVMessage()
        uav_msg.sender = self.uav_name
        uav_msg.msg = ''.join(msg_string_list)
        self.msg_publisher.publish(uav_msg)
