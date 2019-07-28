#!/usr/bin/env python
import rospy
import copy 

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from tool_meshes.srv import *

import tf
import sys

server = None

tool_list = [{'names':['clamp'], 'links':['/clamp_link'], 'positions':[Point(1,0,0)]},
        {'names':['pliers'], 'links':['/left_pliers_link'], 'positions':[Point(1,1,0)]}, 
        {'names':['haxegon_screwdriver'], 'links':['/haxegon_screwdriver_link'], 'positions':[Point(2,1,0)]}, 
        {'names':['yellow_screwdriver'], 'links':['/yellow_screwdriver_link'], 'positions':[Point(1,2,0)]}, 
        {'names':['tape'], 'links':['/tape_link'], 'positions':[Point(1,-2,0)]}, 
        {'names':['knife'], 'links':['/knife_link'], 'positions':[Point(2,0,0)]}, 
        {'names':['hamer_head', 'hamer_handle'], 'links':['/hamer_head_link', '/hamer_handle_link'], 'positions':[Point(2,-2,0), Point(2,-1,0)]}, 
        {'names':['flashlight'], 'links':['/flashlight_link'], 'positions':[Point(1,-1,0)]}]

tool_list = [{'names':['clamp'], 'links':['/clamp_link']},
        {'names':['pliers'], 'links':['/left_pliers_link']}, 
        {'names':['haxegon_screwdriver'], 'links':['/haxegon_screwdriver_link']}, 
        {'names':['yellow_screwdriver'], 'links':['/yellow_screwdriver_link']}, 
        {'names':['tape'], 'links':['/tape_link']}, 
        {'names':['knife'], 'links':['/knife_link']}, 
        {'names':['hamer_head', 'hamer_handle'], 'links':['/hamer_head_link', '/hamer_handle_link']}, 
        {'names':['flashlight'], 'links':['/flashlight_link']}]

class toolInteractiveMarker:
    def __init__(self, frame_id, name):
        self.frame_id = '/tool{}'.format(frame_id)
        self.listener = tf.TransformListener()
        self.initPose()
        self.tool_name = name
        self.make6DofMarker()
        self.br = tf.TransformBroadcaster()
        self.menu_handler = MenuHandler()
        self.grasp_entry = 0
        self.place_entry = 0
        rospy.Subscriber(self.tool_name, Pose, self.callBack)
        self.initMenu()
        self.menu_handler.apply(server, self.tool_name)
        #rospy.Timer(rospy.Duration(0.02), self.updateFrame)

    def grasp_action(self, feedback):
        rospy.wait_for_service('tool_action')
        tool_action = rospy.ServiceProxy('tool_action', actionRequest)
        resp1 = tool_action(self.tool_name, "pickup")
        if resp1.succeed:
            rospy.loginfo("Grasping action completed.")
            self.menu_handler.setVisible(self.place_entry, True)
            self.menu_handler.setVisible(self.move_entry, True)
            self.menu_handler.setVisible(self.grasp_entry, False)
            rospy.loginfo("Grasping action triggered on {}.".format(self.tool_name))
            self.menu_handler.reApply(server)
            server.applyChanges()
        else:
            rospy.loginfo("Grasping action failed.")

    def place_action(self, feedback):
        rospy.wait_for_service('tool_action')
        tool_action = rospy.ServiceProxy('tool_action', actionRequest)
        resp1 = tool_action(self.tool_name, "place")
        if resp1.succeed:
            rospy.loginfo("Placing action completed.")
            self.menu_handler.setVisible(self.place_entry, False)
            self.menu_handler.setVisible(self.move_entry, False)
            self.menu_handler.setVisible(self.grasp_entry, True)
            rospy.loginfo("Placing action triggered on {}.".format(self.tool_name))
            self.menu_handler.reApply(server)
            server.applyChanges()
        else:
            rospy.loginfo("Placing action failed.")

    def move_action(self, feedback):
        rospy.loginfo("Moving action triggered on {}.".format(self.tool_name))
        rospy.wait_for_service('tool_action')
        tool_action = rospy.ServiceProxy('tool_action', actionRequest)
        resp1 = tool_action(self.tool_name, "move")
        if resp1.succeed:
            rospy.loginfo("Moving action completed.")
        else:
            rospy.loginfo("Moving action failed.")


    def initMenu(self):
        self.grasp_entry = self.menu_handler.insert("Grasp",\
                callback=self.grasp_action)
        self.move_entry = self.menu_handler.insert("Move",\
                callback=self.move_action)
        self.place_entry = self.menu_handler.insert("Place",\
                callback=self.place_action)
        self.menu_handler.setVisible(self.move_entry, False)
        self.menu_handler.setVisible(self.place_entry, False)

    def initPose(self):
        self.listener.waitForTransform('/tool/base_link', self.frame_id,\
                rospy.Time(0), rospy.Duration(0.1))
        (trans, rot) = self.listener.lookupTransform('/tool/base_link', \
                self.frame_id, rospy.Time(0))
        self.pose = self.copyPose(trans, rot)

    def copyPose(self, trans, rot):
        p = Pose()
        p.position.x = trans[0]
        p.position.y = trans[1]
        p.position.z = trans[2]
        p.orientation.x = rot[0]
        p.orientation.y = rot[1]
        p.orientation.z = rot[2]
        p.orientation.w = rot[3]
        return p

    def updateFrame(self):
        if self.pose:
            p = self.pose.position
            o = self.pose.orientation
            self.br.sendTransform((p.x, p.y, p.z),
                                  (o.x, o.y, o.z, o.w),
                                  rospy.Time.now(),
                                  self.frame_id,
                                  "/tool/base_link")

    def processFeedback(self, feedback):
        s = "Feedback from marker '" + feedback.marker_name
        s += "' / control '" + feedback.control_name + "'"
        p = feedback.pose.position
        self.pose = feedback.pose;

        mp = ""
        if feedback.mouse_point_valid:
            mp = " at " + str(feedback.mouse_point.x)
            mp += ", " + str(feedback.mouse_point.y)
            mp += ", " + str(feedback.mouse_point.z)
            mp += " in frame " + feedback.header.frame_id

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( s + ": pose changed: (x, y, z) = ({}, {}, {})"\
                    .format(p.x, p.y, p.z))
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            rospy.loginfo( s + ": mouse down" + mp + "." )
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            rospy.loginfo( s + ": mouse up" + mp + "." )
        self.updateFrame()
        server.applyChanges()

    def make6DofMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/tool/base_link" 
        int_marker.pose = self.pose
        int_marker.scale = 0.3

        int_marker.name = self.tool_name 
        int_marker.description = "6-DOF Control"

        """
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        int_marker.controls.append(control)
        """

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        #control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        server.insert(int_marker, self.processFeedback)

    def callBack(self, pose):
        server.setPose(self.tool_name, pose)
        server.applyChanges()
        p = pose.position
        o = pose.orientation
        self.br.sendTransform((p.x, p.y, p.z),
                              (o.x, o.y, o.z, o.w),
                              rospy.Time.now(),
                              self.frame_id,
                              "/tool/base_link")


if __name__=="__main__":
    rospy.init_node("tools_controls")

    server = InteractiveMarkerServer("tools_controls")

    tool_markers = []
    for i in range(1, len(sys.argv)):
        if (sys.argv[i]) == 'true':
            for j in range(len(tool_list[i-1]['names'])):
                name = tool_list[i-1]['names'][j]
                link_name = tool_list[i-1]['links'][j]
                tool_markers.append(toolInteractiveMarker(link_name, name))


    server.applyChanges()
    #marker_move()

    rospy.spin()
