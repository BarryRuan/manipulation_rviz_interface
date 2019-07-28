#!/usr/bin/env python
import rospy

from tool_meshes.srv import *
import tf
from tf import transformations as t
from geometry_msgs.msg import *
import numpy as np

tool_dict = {'clamp':{'links':'/tool/clamp_pickup_link', 'main_axis':0},
        'pliers':{'links':'/tool/pliers_pickup_link', 'main_axis':1}, 
        'haxegon_screwdriver':{'links':'/tool/haxegon_screwdriver_pickup_link', 'main_axis':1}, 
        'yellow_screwdriver':{'links':'/tool/yellow_screwdriver_pickup_link', 'main_axis':2}, 
        'tape':{'links':'/tool/tape_pickup_link', 'main_axis':1}, 
        'knife':{'links':'/tool/knife_pickup_link', 'main_axis':0}, 
        'hamer':{'links':'/tool/hamer_pickup_link', 'main_axis':1}, 
        'flashlight':{'links':'/tool/flashlight_pickup_link', 'main_axis':0}}

class tool_action_server:
    def __init__(self):
        self.current_tool = None
        self.fixedTransformation = None
        self.robot_link = '/base_link'
        self.gripper_link = '/gripper_link'
        self.server_start()


    def handle_pickup_(self, tool_name):
        if self.current_tool:
            rospy.loginfo("ERROR: Robot is holding {}, so can't pick up {}"\
                    .format(self.current_tool, tool_name))
            return False
        if tool_name not in tool_dict:
            rospy.loginfo("ERROR: Tool {} is not recognized.".format(tool_name))
            return False
        pregrasp_frame, grasp_frame = self.calculateGripperPose_(tool_name)
        #TODO: send motion
        print(pregrasp_frame)
        #TODO: send motion
        print(grasp_frame)
        #TODO: send gripper motion
        self.current_tool = tool_name
        """
        tool_link = tool_dict[tool_name]['links']
        (trans, rot) = self.listener.lookupTransform(\
                tool_link, self.gripper_link, rospy.Time(0))
        self.fixedTransformation = t.quaternion_matrix(rot)
        self.fixedTransformation[:3,3] += trans
        """
        self.fixedTransformation = self.ff
        return True


    def handle_move_(self, tool_name):
        if not self.current_tool:
            rospy.loginfo("ERROR: Robot is not holding anything, so can't move {}"\
                    .format(tool_name))
            return False
        if self.current_tool != tool_name:
            rospy.loginfo("ERROR: Robot is holding {}, so can't move {}"\
                    .format(self.current_tool, tool_name))
            return False
        if tool_name not in tool_dict:
            rospy.loginfo("ERROR: Tool {} is not recognized.".format(tool_name))
            return False
        gripper_frame = self.calculateGripperPose_()
        #TODO: send motion
        print(gripper_frame)
        return True

    def handle_place_(self, tool_name):
        if not self.current_tool:
            rospy.loginfo("ERROR: Robot is not holding anything, so can't place {}"\
                    .format(tool_name))
            return False
        if self.current_tool != tool_name:
            rospy.loginfo("ERROR: Robot is holding {}, so can't place {}"\
                    .format(self.current_tool, tool_name))
            return False
        if tool_name not in tool_dict:
            rospy.loginfo("ERROR: Tool {} is not recognized.".format(tool_name))
            return False
        pregrasp_frame, grasp_frame = self.calculateGripperPose_(tool_name)
        #TODO: send motion
        print(pregrasp_frame)
        #TODO: send motion
        print(grasp_frame)
        #TODO: send gripper motion
        self.current_tool = None
        self.fixedTransformation = None
        return True

    def calculateGripperPose_(self, tool_name=None):
        if tool_name:
            # Calculate from tool main axis
            # Return pregrasp_frame_in_pickup_frame and grasp_frame_in_pickup_frame
            tool_link = tool_dict[tool_name]['links']
            (trans, rot) = self.listener.lookupTransform(self.robot_link, tool_link, rospy.Time(0))
            rotation_m = t.quaternion_matrix(rot)
            main_axis = rotation_m[:3, tool_dict[tool_name]['main_axis']]
            rot_z = np.arctan2(main_axis[1], main_axis[0])
            rot_y = np.arcsin(main_axis[2])
            rm_z = t.rotation_matrix(rot_z, [0, 0, 1])
            rm_y = t.rotation_matrix(rot_y, [0, 1, 0])
            rm_zy = np.matmul(rm_z, rm_y)
            rm_zy[:3,3] = rm_zy[:3,3] + trans
            pickup_frame = rm_zy.copy()
            pregrasp_frame_in_pickup_frame = t.quaternion_matrix(\
                    np.array([np.sqrt(2)/2, 0, np.sqrt(2)/2, 0]))
            grasp_frame_in_pickup_frame = pregrasp_frame_in_pickup_frame.copy()
            pregrasp_frame_in_pickup_frame[2,3] = 0.2
            pregrasp_frame = np.matmul(pickup_frame, pregrasp_frame_in_pickup_frame)
            grasp_frame = np.matmul(pickup_frame, grasp_frame_in_pickup_frame)
            #TODO:delete this!
            self.ff = grasp_frame_in_pickup_frame
            return pregrasp_frame, grasp_frame 
        else:
            # Calculate from fixed transformation
            tool_link = tool_dict[self.current_tool]['links']
            (trans, rot) = self.listener.lookupTransform(self.robot_link, tool_link, rospy.Time(0))
            rotation_m = t.quaternion_matrix(rot)
            rotation_m[:3, 3] += trans
            gripper_frame = np.matmul(rotation_m, self.fixedTransformation)
            return gripper_frame
        return 0

    def handle_action(self, req):
        rospy.loginfo("tool name is : {}, action type is : {}".format(req.tool_name, req.action_type))
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
