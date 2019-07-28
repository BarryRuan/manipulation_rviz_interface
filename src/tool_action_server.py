#!/usr/bin/env python
import rospy
from tool_meshes.srv import *
import tf
from geometry_msgs.msg import *


class tool_action_server:
    def __init__(self):
        self.current_tool = None
        self.fixedTransformation = None
        self.server_start()

    def handle_pickup_(self, tool_name):
        if self.current_tool:
            rospy.loginfo("Robot is holding {}, so can't pick up {}"\
                    .format(self.current_tool, tool_name))
            return False
        self.current_tool = tool_name
        return True

    def handle_move_(self, tool_name):
        if not self.current_tool:
            rospy.loginfo("Robot is not holding anything, so can't move {}"\
                    .format(tool_name))
            return False
        if self.current_tool != tool_name:
            rospy.loginfo("Robot is holding {}, so can't move {}"\
                    .format(self.current_tool, tool_name))
            return False
        return True

    def handle_place_(self, tool_name):
        if not self.current_tool:
            rospy.loginfo("Robot is not holding anything, so can't place {}"\
                    .format(tool_name))
            return False
        if self.current_tool != tool_name:
            rospy.loginfo("Robot is holding {}, so can't place {}"\
                    .format(self.current_tool, tool_name))
            return False
        self.current_tool = None
        return True

    def calculateGripperPose_(self):
        if not self.current_tool:
            # Calculate from tool main axis
            return 0
        else:
            # Calculate from fixed transformation
            return 0

    def calculatePrepose_(self):
        return 0

    def handle_action(self, req):
        rospy.loginfo("tool name is : {}, action type is : {}".format(req.tool_name, req.action_type))
        (trans, rot) = self.listener.lookupTransform('/tool/base_link', '/tool/{}_pickup_link'.format(req.tool_name), rospy.Time(0))
        print(trans, rot)
        if req.action_type == 'pickup':
            return self.handle_pickup_(req.tool_name)
        elif req.action_type == 'move':
            return self.handle_move_(req.tool_name)
        elif req.action_type == 'place':
            return self.handle_place_(req.tool_name)
        else:
            rospy.loginfo("Action type {} is not recognized.".format(req.action_type))
            return False

    def server_start(self):
        rospy.init_node('tool_action_server')
        self.listener = tf.TransformListener()
        self.s = rospy.Service('tool_action', actionRequest, self.handle_action)
        print('action server is running')
        rospy.spin()

if __name__ == "__main__":
    s = tool_action_server()
