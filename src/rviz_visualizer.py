from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospy
import numpy as np




class RvizVisualizer:
    def __init__ (self, parent):
        self.parent = parent
        rospy.loginfo("Starting Rviz visualization node")
        #self.node = rospy.init_node('rover_kinematics_node',anonymous=False)
        self.marker_pub = rospy.Publisher("markers_pub", Marker, queue_size=10)
        self.initializeMessages()

    def __del__ (self):
        rospy.loginfo("Shutting down Rviz visualization node")

    def initializeMessages(self):
        self.points = Marker()
        self.lines_list = Marker()
        self.point_icr = Marker()
        self.feasible_icr_area = Marker()

        base_link = rospy.get_param("base_frame", "base_link")
        self.points.header.frame_id = base_link
        self.lines_list.header.frame_id = base_link
        self.point_icr.header.frame_id = base_link
        self.feasible_icr_area.header.frame_id = base_link

        self.points.action = Marker.ADD
        self.lines_list.action = Marker.ADD
        self.point_icr.action = Marker.ADD
        self.feasible_icr_area.action = Marker.ADD

        self.points.pose.orientation.w = 1
        self.lines_list.pose.orientation.w = 1
        self.point_icr.pose.orientation.w = 1
        self.feasible_icr_area.pose.orientation.w =1

        self.points.id = 0
        self.lines_list.id = 1
        self.point_icr.id = 2
        self.feasible_icr_area.id = 3

        self.points.type = Marker.POINTS
        self.lines_list.type = Marker.LINE_LIST
        self.point_icr.type = Marker.POINTS
        self.feasible_icr_area.type = Marker.TRIANGLE_LIST

        self.points.scale.x = 0.05
        self.points.scale.y = 0.05
        self.lines_list.scale.x = 0.01
        self.lines_list.scale.y = 0.01
        self.point_icr.scale.x = 0.05
        self.point_icr.scale.y = 0.05
        self.feasible_icr_area.scale.z = 1
        self.feasible_icr_area.scale.x = 1
        self.feasible_icr_area.scale.y = 1

        self.points.color.g = 0
        self.points.color.b = 1
        self.points.color.a = 1
        self.lines_list.color.r = 1
        self.lines_list.color.a = 1
        self.point_icr.color.g = 1
        self.point_icr.color.a = 1
        self.feasible_icr_area.color.g = 1
        self.feasible_icr_area.color.b = 1
        self.feasible_icr_area.color.a = 1
        self.feasible_icr_area.color.r = 204/255

    def clearMessages(self):
        self.point_icr.points = list()
        self.points.points = list()
        self.lines_list.points = list()
        self.feasible_icr_area.points = list()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            self.points.header.stamp = rospy.Time.now()
            self.lines_list.header.stamp = rospy.Time.now()


            # Create points
            p = Point() #ICR point
            p.x = self.parent.controller_map[self.parent.current_mode]._last_valid_icr[0]
            p.y = self.parent.controller_map[self.parent.current_mode]._last_valid_icr[1]

            l_max = np.sqrt(p.x**2 + p.y**2) + 2 

            self.point_icr.points.append(p)
            for i in range(self.parent.controller_map[self.parent.current_mode]._pos_wheels.shape[0]):
                p_w = Point()
                p_w.x = self.parent.controller_map[self.parent.current_mode]._pos_wheels[i,0]
                p_w.y = self.parent.controller_map[self.parent.current_mode]._pos_wheels[i,1]
                self.points.points.append(p_w)

                p_n = Point()
                wheels_steer = self.parent.controller_map[self.parent.current_mode]._current_steer
                p_n.x = self.parent.controller_map[self.parent.current_mode]._pos_wheels[i,0] - l_max*np.sin(wheels_steer[i])
                p_n.y = self.parent.controller_map[self.parent.current_mode]._pos_wheels[i,1] + l_max*np.cos(wheels_steer[i])

                self.lines_list.points.append(p_w)
                self.lines_list.points.append(p_n)


            pt_intersection_top = Point()
            pt_intersection_top.x = self.parent.controller_map[self.parent.current_mode]._icr_handler.data["intersections"]["top"][0][0]
            pt_intersection_top.y = self.parent.controller_map[self.parent.current_mode]._icr_handler.data["intersections"]["top"][1][0]

            pt_top1 = Point()
            pt_top1.x = pt_intersection_top.x + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_fl"]["direction"][0]
            pt_top1.y = pt_intersection_top.y + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_fl"]["direction"][1]
            pt_top2 = Point()
            pt_top2.x = pt_intersection_top.x + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_bl"]["direction"][0]
            pt_top2.y = pt_intersection_top.y + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_bl"]["direction"][1]


            pt_intersection_bottom = Point()
            pt_intersection_bottom.x = self.parent.controller_map[self.parent.current_mode]._icr_handler.data["intersections"]["bottom"][0][0]
            pt_intersection_bottom.y = self.parent.controller_map[self.parent.current_mode]._icr_handler.data["intersections"]["bottom"][1][0]

            pt_bottom_1 = Point()
            pt_bottom_1.x = pt_intersection_bottom.x + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_fr"]["direction"][0]
            pt_bottom_1.y = pt_intersection_bottom.y + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_fr"]["direction"][1]
            pt_bottom_2 = Point()
            pt_bottom_2.x = pt_intersection_bottom.x + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_br"]["direction"][0]
            pt_bottom_2.y = pt_intersection_bottom.y + 10*self.parent.controller_map[self.parent.current_mode]._icr_handler.data["wheel_br"]["direction"][1]

            self.feasible_icr_area.points.append(pt_intersection_top)
            
            self.feasible_icr_area.points.append(pt_top2)
            self.feasible_icr_area.points.append(pt_top1)
            self.feasible_icr_area.points += [pt_intersection_bottom, pt_bottom_1, pt_bottom_2]

            self.marker_pub.publish(self.point_icr)
            self.marker_pub.publish(self.points)
            self.marker_pub.publish(self.lines_list)
            self.marker_pub.publish(self.feasible_icr_area)

            self.clearMessages()

            r.sleep()

