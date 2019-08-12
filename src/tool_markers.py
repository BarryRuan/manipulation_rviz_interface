#!/usr/bin/env python
import rospy
import copy 

from interactive_markers.interactive_marker_server import *

from visualization_msgs.msg import *
from geometry_msgs.msg import Pose

import tf

server = []

class toolInteractiveMarker:
    def __init__(self, frame_id, parent_link, name, init_pose):
        self.frame_id = frame_id
        self.parent_link = parent_link
        self.tool_name = name
        self.init_pose = init_pose
        self.marker_up = False

        self.listener = tf.TransformListener()
        self.initPose()
        self.br = tf.TransformBroadcaster()
        #rospy.Subscriber(self.tool_name, Pose, self.callBack)

    def initPose(self):
        self.listener.waitForTransform(self.parent_link, self.frame_id,\
                rospy.Time(0), rospy.Duration(0.1))
        (trans, rot) = self.listener.lookupTransform(self.parent_link, \
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

    def updateTrans(self, trans):
        o = self.pose.orientation
        self.br.sendTransform((trans.pc_x, trans.pc_y, trans.pc_z),
                              (o.x, o.y, o.z, o.w),
                              rospy.Time.now(),
                              self.frame_id,
                              self.parent_link)

    def updateFrame(self):
        if self.pose:
            p = self.pose.position
            o = self.pose.orientation
            self.br.sendTransform((p.x, p.y, p.z),
                                  (o.x, o.y, o.z, o.w),
                                  rospy.Time.now(),
                                  self.frame_id,
                                  self.parent_link)

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

    def reset_pose(self):
        o = [0, 0, 0, 1]
        p = self.init_pose
        pose = self.copyPose(p, o)
        p = pose.position
        o = pose.orientation
        self.br.sendTransform((p.x, p.y, p.z),
                              (o.x, o.y, o.z, o.w),
                              rospy.Time.now(),
                              self.frame_id,
                              self.parent_link)
        if self.marker_up:
            server[0].setPose(self.tool_name, pose)
            server[0].applyChanges()

    def make6DofMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.parent_link
        int_marker.pose = self.pose
        int_marker.scale = 0.3

        int_marker.name = self.tool_name 
        int_marker.description = "6-DOF Control"

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

        server[0].insert(int_marker, self.processFeedback)
        server[0].applyChanges()
        self.marker_up = True

    def erase_marker(self):
        server[0].erase(self.tool_name)
        server[0].applyChanges()
        self.marker_up = False

    def callBack(self, pose):
        p = pose.position
        o = pose.orientation
        self.br.sendTransform((p.x, p.y, p.z),
                              (o.x, o.y, o.z, o.w),
                              rospy.Time.now(),
                              self.frame_id,
                              self.parent_link)
        server[0].setPose(self.tool_name, pose)
        server[0].applyChanges()
