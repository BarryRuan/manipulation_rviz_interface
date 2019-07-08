#!/usr/bin/env python
import rospy
import copy 

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
import tf

server = None

class toolInteractiveMarker:
    def __init__(self, frame_id, name, frame_origin):
        self.pose = None 
        self.frame_id = frame_id
        self.frame_origin = frame_origin 
        self.tool_name = name
        self.make6DofMarker()
        self.br = tf.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.02), self.updateFrame)

    def updateFrame(self, event):
        if self.pose:
            p = self.pose.position
            o = self.pose.orientation
            self.br.sendTransform((p.x, p.y, p.z),
                                  (o.x, o.y, o.z, o.w),
                                  rospy.Time.now(),
                                  self.frame_id,
                                  "base_link")

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
        server.applyChanges()

    def make6DofMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "base_link" 
        int_marker.pose.position = self.frame_origin 
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
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        server.insert(int_marker, self.processFeedback)


if __name__=="__main__":
    rospy.init_node("tools_controls")

    server = InteractiveMarkerServer("tools_controls")

    position = Point(1,1,0)
    plier_marker = toolInteractiveMarker("pliers_link", "pliers", position)

    position = Point(1,0,0)
    plier_marker = toolInteractiveMarker("clamp_link", "clamp", position)

    server.applyChanges()

    rospy.spin()
