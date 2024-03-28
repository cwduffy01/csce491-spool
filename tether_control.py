import odrive
from odrive.enums import *
import time
import math
import rospy
from std_msgs.msg import Float32

class TetherControl():

    spool_circ = 0.06 * 2 * math.pi    # in m
    offset = 0

    def __init__(self):
        print("finding an odrive...")
        self.odrv0 = odrive.find_any()

        print("starting calibration sequence")
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
        self.odrv0.axis0.controller.config.input_filter_bandwidth = 5.0

        self.offset = -self.odrv0.axis0.pos_estimate

        # self.offset = -self.odrv0.axis0.controller.input_pos

        rospy.Subscriber("/target_tether_length", Float32, self.callback)
        self.pub = rospy.Publisher("/current_tether_length", Float32, queue_size=10)

        rospy.init_node('tether_control', anonymous=True)

        print("node initialized")

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            current_length = self.readpos() * self.spool_circ
            self.pub.publish(current_length)
            rate.sleep()

        rospy.spin()


    def callback(self, msg):
        # print(self.odrv0.axis0.controller.input_pos)
        # self.odrv0.axis0.controller.input_pos = -msg.data / self.spool_circ - self.offset
        self.moveto(-msg.data / self.spool_circ)
        # print(msg.data)

    def moveto(self, pos):
        self.odrv0.axis0.controller.input_pos = pos - self.offset

    def readpos(self):
        return -(self.odrv0.axis0.pos_estimate + self.offset)

    def __del__(self):
        # self.odrv0.axis0.controller.input_pos = 0
        # self.moveto(0)
        # time.sleep(10)
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        print("destroyed")

if __name__ == "__main__":
    TetherControl()