#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32
import sys


class TetherModel():
    prev_dist = 0
    curr_dist = 0

    prev_length = 0
    curr_length = 0

    prev_time = 0

    spool_offset_x = 0.6096
    spool_offset_y = 0.0

    def __init__(self, slackness=0.1):
        rospy.init_node("tether_model")

        self.slackness = slackness

        rospy.Subscriber("/duckie_pose", Pose2D, self.pose_callback)
        self.pub = rospy.Publisher("/target_tether_length", Float32, queue_size=10)

        rospy.spin()

    def pose_callback(self, msg):
        dist = ((msg.x + self.spool_offset_x)**2 + (msg.y + self.spool_offset_y)**2)**0.5

        scale = 1.2
        tether_len = scale * dist + self.slackness
        self.pub.publish(tether_len)
        
    
if __name__ == "__main__":
    try:
        TetherModel(float(sys.argv[1]))
    except Exception:
        TetherModel()