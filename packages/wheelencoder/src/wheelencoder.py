#!/usr/bin/env python3
import numpy as np
import os
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self._N_total=135
        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick',WheelEncoderStamped,self.cb_encoder_data,"left")
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick',WheelEncoderStamped,self.cb_encoder_data,"right")
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed',WheelsCmdStamped,self.cb_executed_commands)

        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/left_wheel_odometry_node/distance',Float32,queue_size=1)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/right_wheel_odometry_node/distance',Float32,queue_size=1)
        self.tick_left=0
        self.tick_right=0
        self.distance_left=0
        self.distance_right=0
        self.log("Initialized")

    def cb_encoder_data(self, msg,arg):
        """ Update encoder distance information from ticks.
        """
        if arg=="left":
            self.tick_left=msg.data
            self.distance_left=2*np.pi*self._radius*msg.data/self._N_total
            self.pub_integrated_distance_left.publish(self.distance_left)
        else:
            self.tick_right=msg.data
            self.distance_right=2*np.pi*self._radius*msg.data/self._N_total
            self.pub_integrated_distance_right.publish(self.distance_right)

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        pass
    
def calibration(node):
    cmd=input("slightly move the robot and then enter s to start calibration: ")
    if cmd=="s":
        left_tick_s=node.tick_left
        right_tick_s=node.tick_right
    else:
        print("unknown command")
    cmd=input("enter f to start calibration: ")
    if cmd=="f":
        left_tick_f=node.tick_left
        right_tick_f=node.tick_right
    else:
        print("unknown command")
    distance=input("how far did duckiebot go(m)?")
    radius_left=float(distance)*node._N_total/(2*np.pi*(left_tick_f-left_tick_s))
    radius_right=float(distance)*node._N_total/(2*np.pi*(right_tick_f-right_tick_s))
    print("left tick:",left_tick_f-left_tick_s," right tick:",right_tick_f-right_tick_s )
    print("left radius:",radius_left," right radius:", radius_right)
    
if __name__ == '__main__':
    node = OdometryNode(node_name='odometry_node')
    # Keep it spinning to keep the node alive
    calibration(node)
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")
    