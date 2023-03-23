#!/usr/bin/env python3.8

"""
  @author Matteo Caruso
  @email matteo.caruso@phd.units.it
  @email matteo.caruso1993@gmail.com
"""


import rospy
import matplotlib.pyplot as plt
import math
import numpy as np
import quaternion
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import LinkStates
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
from rover4ws_kinematics.src.utils.default_config import default_config
from rviz_visualizer import RvizVisualizer
from threading import Thread



class RoverKinematicNode:
    def __init__(self):
        self.valid_modes = ['car_like','symmetric_ackermann', 'outer_ackermann','inner_ackermann','full_ackermann','parallel_ackermann','lateral_drive','in_place_rotation']
        self.cmd_publisher = None
        self.cmd_subscriber = None
        self.animation = False
        self.plot_trigger = False
        self._params = {**default_config, **{"base_frame": "", "wheel_frame": ""}}

    def initialize(self) -> None:
        rospy.loginfo('Starting RoverKinematicNode')
        self.node = rospy.init_node('rover_kinematics_node',anonymous=False)
        rospy.loginfo('rover_kinematics_node started')
        
        # Get node parameters
        self._getParameters()

        # Initialize publisher
        self.cmd_publisher = rospy.Publisher('cmd_vel_motors', Dynamixel_parameters1, queue_size=10)
        self.cmd_pub_msg = Dynamixel_parameters1()

        # Initialize subscriber
        self.cmd_subscriber = rospy.Subscriber('cmd_vel', Twist, self._cmd_sub_callback)
        self.reset_subscriber = rospy.Subscriber('kinematics_reset', Empty, self.reset)
        self.animate_subscriber = rospy.Subscriber('plot', Empty, self._updatePlot)

        # Initialize service
        self.mode_change_service = rospy.Service("kinematic_mode", kinematic_mode, self.changeKinematicMode)

        # Start listener for tf
        self.updateGeometry()

        # Reset kinematics controller
        self.reset()

    def updateGeometry(self):
        base_frame = self._params["base_frame"]
        wheel_frame = self._params["wheel_frame"]
        # print("Updating geometry")
        link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)
        # print("Geometry updated")
        # print(link_states.name)
        counter = 0
        base_frame_ID = None
        wheel_frame_ID = None
        for n, item in enumerate(link_states.name):
            if base_frame in item:
                print("Found link [%s] in position [%d]"%(base_frame, n))
                base_frame_ID = n
                counter+=1
            elif wheel_frame in item:
                print("Found link [%s] in position [%d]"%(wheel_frame, n))
                wheel_frame_ID = n
                counter+=1

            if counter > 1:
                break
        
        # Get pose
        # print(link_states.pose[base_frame_ID])
        # print(link_states.pose[wheel_frame_ID])

        # Get orientation in Euler angles
        base_frame_quat = np.quaternion(link_states.pose[base_frame_ID].orientation.w, 
            link_states.pose[base_frame_ID].orientation.x, 
            link_states.pose[base_frame_ID].orientation.y, 
            link_states.pose[base_frame_ID].orientation.z)
        wheel_frame_quat = np.quaternion(link_states.pose[wheel_frame_ID].orientation.w, 
            link_states.pose[wheel_frame_ID].orientation.x, 
            link_states.pose[wheel_frame_ID].orientation.y, 
            link_states.pose[wheel_frame_ID].orientation.z)
        base_rotM = quaternion.as_rotation_matrix(base_frame_quat)
        wheel_rotM = quaternion.as_rotation_matrix(wheel_frame_quat)
        base_frame_x_rot = np.matmul(base_rotM, np.array([[1, 0, 0]]).T)
        base_frame_y_rot = np.matmul(base_rotM, np.array([[0, 1, 0]]).T)
        basePosition = np.array([[
            link_states.pose[base_frame_ID].position.x, 
            link_states.pose[base_frame_ID].position.y, 
            link_states.pose[base_frame_ID].position.z]]).T
        wheelPosition = np.array([[
            link_states.pose[wheel_frame_ID].position.x, 
            link_states.pose[wheel_frame_ID].position.y, 
            link_states.pose[wheel_frame_ID].position.z]]).T
        baseToWheelVector = wheelPosition - basePosition
        baseToWheelVector_proj_x = np.dot(baseToWheelVector.T, base_frame_x_rot)
        baseToWheelVector_proj_y = np.dot(baseToWheelVector.T, base_frame_y_rot)
        # print(baseToWheelVector_proj_x)
        # print(baseToWheelVector_proj_y)

        # Save to params
        self._params["len_x_leg"] = baseToWheelVector_proj_x[0,0]
        self._params["len_y_leg"] = baseToWheelVector_proj_y[0,0]


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
        # Look for ROS parameters
        self._params['base_frame'] = rospy.get_param("base_frame", "base_link")
        self._params['wheel_frame'] = rospy.get_param("wheel_frame", "wheel_link")
        self._params['drive_mode'] = rospy.get_param("drive_mode", 'Ackermann_symmetric')
        self._params['max_wheel_speed'] = rospy.get_param("max_wheel_speed", 10)
        self._params['wheel_radius'] = rospy.get_param("wheel_radius", 0.085)
        self._params['wheel_width'] = rospy.get_param("wheel_width", 0.01)
        self._params['angle_crit'] = rospy.get_param("angle_crit", math.pi)
        self._params['angle_max'] = rospy.get_param("angle_max", math.pi)
        self._params['wheels_limits'] = dict()
        self._params['wheels_limits']['fl_wheel'] = rospy.get_param("wheels_limits/fl_wheel", [-math.pi, +math.pi])
        self._params['wheels_limits']['bl_wheel'] = rospy.get_param("wheels_limits/bl_wheel", [-math.pi, +math.pi])
        self._params['wheels_limits']['fr_wheel'] = rospy.get_param("wheels_limits/fr_wheel", [-math.pi, +math.pi])
        self._params['wheels_limits']['br_wheel'] = rospy.get_param("wheels_limits/br_wheel", [-math.pi, +math.pi])


    def _updatePlot(self,in_d):
        plt.close('all')
        self.plot_trigger = not self.plot_trigger
        #self.controller_map[self.current_mode].show(plot=True, show_frame=False, draw_wheels_arrows=False, draw_computed_wheel_lin_speed=True)


    def reset(self):
        self.cmd_pub_msg = Dynamixel_parameters1()
        self.last_cmd_pub_msg = Dynamixel_parameters1()
        self.last_cmd_vel_msg = Twist()
        self.current_mode = self._params['drive_mode']
        self.visualizer = None


        self.controller_map = {
            'car_like': CarLike(config_override=self._params),
            'symmetric_ackermann': SymmetricAckermann(config_override=self._params),
            'outer_ackermann': OuterAckermann(config_override=self._params),
            'inner_ackermann': InnerAckermann(config_override=self._params),
            'full_ackermann': FullAckermann(config_override=self._params),
            'parallel_ackermann': ParallelAckermann(config_override=self._params),
            'lateral_drive': LateralDrive(config_override=self._params),
            'in_place_rotation': InplaceRotation(config_override=self._params)
        }


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

        # Publish
        self.cmd_publisher.publish(self.cmd_pub_msg)
    
    def visualize(self):
        self.visualizer = RvizVisualizer(self)



def main() -> None:
    node = RoverKinematicNode()
    node.initialize()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        #node.cmd_publisher.publish(node.cmd_pub_msg)
        node.last_cmd_pub_msg = node.cmd_pub_msg
        if node.plot_trigger:
            node.visualize()
            t1 = Thread(target = node.visualizer.run)
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
