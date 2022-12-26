#!/usr/bin/env python3.8

import rospy
import matplotlib.pyplot as plt
import math
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from robot4ws_msgs.msg import Dynamixel_parameters1
from robot4ws_msgs.srv import kinematic_mode, kinematic_modeResponse
from rover4ws_kinematics.src.modes.carLike import CarLike
from rover4ws_kinematics.src.modes.symmetricAckermann import SymmetricAckermann
from rover4ws_kinematics.src.modes.outerAckermann import OuterAckermann
from rover4ws_kinematics.src.modes.innerAckermann import InnerAckermann
from rover4ws_kinematics.src.modes.fullAckermann import FullAckermann
from rover4ws_kinematics.src.modes.parallel_steer_ackermann import ParallelAckermann
from rover4ws_kinematics.src.modes.parallelDrive import LateralDrive
from rover4ws_kinematics.src.modes.pureInplaceRotation import InplaceRotation
from rviz_visualizer import RvizVisualizer
from threading import Thread



class RoverKinematicNode:
    def __init__(self):
        self.valid_modes = ['car_like','symmetric_ackermann', 'outer_ackermann','inner_ackermann','full_ackermann','parallel_ackermann','lateral_drive','in_place_rotation']
        self.cmd_publisher = None
        self.cmd_subscriber = None
        self.animation = False
        self.plot_trigger = False

    def initialize(self) -> None:
        self.reset()
        rospy.loginfo('Starting RoverKinematicNode')
        self.node = rospy.init_node('rover_kinematics_node',anonymous=False)
        rospy.loginfo('rover_kinematics_node started')

        #Initialize publisher
        self.cmd_publisher = rospy.Publisher('cmd_vel_motors',Dynamixel_parameters1,queue_size=10)
        self.cmd_pub_msg = Dynamixel_parameters1()

        #Initialize subscriber
        self.cmd_subscriber = rospy.Subscriber('cmd_vel',Twist, self._cmd_sub_callback)
        self.reset_subscriber = rospy.Subscriber('kinematics_reset', Empty, self.reset)
        self.animate_subscriber = rospy.Subscriber('plot',Empty, self._updatePlot)
        #Initialize service
        self.mode_change_service = rospy.Service("kinematic_mode", kinematic_mode, self.changeKinematicMode)
        
        #Get node parameters
        self._getParameters()

    def changeKinematicMode(self,req):
        if req.requested_kinematic_mode not in self.valid_modes:
            rospy.logerr('The selected kinematic mode: ('+req.requested_kinematic_mode+') is not valid. Skipping...')
            return kinematic_modeResponse(False)
        else:
            current_mode = req.requested_kinematic_mode
            
            #Reset currently in use kinematics controller
            rospy.loginfo('Kinematic mode changed to the requested mode (' + req.requested_kinematic_mode +') successfully')
            self.current_mode = req.requested_kinematic_mode
            return kinematic_modeResponse(True)

    
    def _getParameters(self):
        pass

    def _updatePlot(self,in_d):
        plt.close('all')
        self.plot_trigger = not self.plot_trigger
        #self.controller_map[self.current_mode].show(plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=True)


    def reset(self):
        self.cmd_pub_msg = Dynamixel_parameters1()
        self.last_cmd_pub_msg = Dynamixel_parameters1()
        self.last_cmd_vel_msg = Twist()
        self.current_mode = 'outer_ackermann'
        self.visualizer = None


        self.controller_map = {
            'car_like': CarLike(),
            'symmetric_ackermann': SymmetricAckermann(),
            'outer_ackermann': OuterAckermann(),
            'inner_ackermann': InnerAckermann(),
            'full_ackermann': FullAckermann(),
            'parallel_ackermann': ParallelAckermann(),
            'lateral_drive': LateralDrive(),
            'in_place_rotation': InplaceRotation()
        }

        #for key in self.controller_map:
         #   self.controller_map[key].initialize()

    def _cmd_sub_callback(self, data):
        self.last_cmd_vel_msg = data
        vels = [data.linear.x, data.linear.y, data.angular.z]
        speeds, steers = self.controller_map[self.current_mode].kinematicsStep(vels, update_speeds=False, get_output=True)

        #Now we need to sort data
        self.cmd_pub_msg.One_Primary = speeds[3]
        self.cmd_pub_msg.Two_Primary = speeds[1]
        self.cmd_pub_msg.Three_Primary = speeds[2]
        self.cmd_pub_msg.Four_Primary = speeds[0]

        self.cmd_pub_msg.Five_Primary = steers[3]
        self.cmd_pub_msg.Six_Primary = steers[1]
        self.cmd_pub_msg.Seven_Primary = steers[2]
        self.cmd_pub_msg.Eight_Primary = steers[0]
    
    def visualize(self):
        self.visualizer = RvizVisualizer(self)



def main() -> None:
    node = RoverKinematicNode()
    node.initialize()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.cmd_publisher.publish(node.cmd_pub_msg)
        node.last_cmd_pub_msg = node.cmd_pub_msg
        if node.plot_trigger:
            node.visualize()
            t1 = Thread(target=node.visualizer.run)
            t1.start()
            #node.controller_map[node.current_mode].show(plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=True)
            node.plot_trigger = not node.plot_trigger

        #print(node.controller_map[node.current_mode]._current_wheel_speed)
        r.sleep()

    t1.join()







if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:

        rospy.loginfo('Shutting down ROS kinematic node')
