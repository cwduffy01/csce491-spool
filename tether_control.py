import odrive
from odrive.enums import *
import time
import math
import rospy
from std_msgs.msg import Float32, Empty

class TetherControl():

    spool_circ = 0.06 * 2 * math.pi    # in m
    offset = 0

    spool_offset_x = 0.6096
    spool_offset_y = 0.0

    def __init__(self):
        print("finding an odrive...")
        self.odrv0 = odrive.find_any()

        print("starting calibration sequence")
        # self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        # while self.odrv0.axis0.current_state != AXIS_STATE_IDLE:
        #     time.sleep(0.1)

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER

        self.odrv0.axis0.controller.config.input_filter_bandwidth = 25.0

        self.offset = self.odrv0.axis0.pos_estimate

        self.tether_offset = (self.spool_offset_x**2 + self.spool_offset_y**2)**0.5

        rospy.Subscriber("/target_tether_length", Float32, self.callback)
        self.pub = rospy.Publisher("/current_tether_length", Float32, queue_size=10)
        rospy.Subscriber("/home", Empty, self.home)
        rospy.Subscriber("/motor_gain", Float32, self.set_gain)

        rospy.init_node('tether_control', anonymous=True)

        print("node initialized")

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            current_length = self.readpos() * self.spool_circ + self.tether_offset
            self.pub.publish(current_length)
            rate.sleep()


    def callback(self, msg):
        # print(self.odrv0.axis0.controller.input_pos)
        # self.odrv0.axis0.controller.input_pos = -msg.data / self.spool_circ - self.offset

        # self.moveto(-msg.data / self.spool_circ)

        self.moveto(-(msg.data - self.tether_offset) / self.spool_circ)
        # print(msg.data)

    def set_gain(self, msg):
        self.odrv0.axis0.controller.config.input_filter_bandwidth = msg.data
        print(self.odrv0.axis0.controller.config.input_filter_bandwidth)

    def moveto(self, pos):
        # print(str(self.odrv0.axis0.pos_estimate) + " -> " + str(pos - self.offset))
        self.odrv0.axis0.controller.input_pos = pos + self.offset

    def readpos(self):
        return -(self.odrv0.axis0.pos_estimate - self.offset)
    
    def home(self):
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        rospy.sleep(0.5)

        self.odrv0.axis0.controller.input_vel = 1.0
        while(self.odrv0.axis0.pos_estimate < self.offset):
            rospy.sleep(0.01)
        self.odrv0.axis0.controller.input_vel = 0.0
        
        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.odrv0.axis0.controller.config.input_mode = INPUT_MODE_POS_FILTER
        rospy.sleep(0.5)

        self.moveto(0)

        rospy.sleep(0.5)

    def __del__(self):
        # self.odrv0.axis0.controller.input_pos = 0
        # self.moveto(0)
        # time.sleep(10)
        self.odrv0.axis0.controller.input_pos = self.offset
        rospy.sleep(1.0)
        self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
        print("destroyed")

if __name__ == "__main__":
    TetherControl()