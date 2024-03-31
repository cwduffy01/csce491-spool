import rospy
import rosbag
from std_msgs import String, Float32, Empty

class BagRecorder():

    shape = ""
    motor_gain = 0.0
    lin_vel = 0.0
    slack = 0.0

    def __init__(self):
        rospy.Subscriber("/path_shape", String, self.shape_callback)
        rospy.Subscriber("/motor_gain", Float32, self.gain_callback)
        rospy.Subscriber("/path_velocity", String, self.vel_callback)
        rospy.Subscriber("/tether_slackness", String, self.slack_callback)

        rospy.Subscriber("/start_bag_record", Empty, self.record)
        rospy.Subscriber("/stop_bag_record", Empty, self.stop)

    def shape_callback(self, msg):
        self.shape = msg.data

    def gain_callback(self, msg):
        self.motor_gain = msg.data

    def vel_callback(self, msg):
        self.lin_vel = msg.data

    def slack_callback(self, msg):
        self.slack = msg.data

    def record(self):
        pass
