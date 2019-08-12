"""
Menu:
    Select object
        [x] tool_1
        [ ] tool_2
            ...
    Request tool action
        grasp
            pick_up_link1
            pick_up_link2
            ...
        move
        place
        push
            push_point_link1
            push_point_link2
            ...
        pull
            pull_point_link1
            pull_point_link2
            ...
    Request robot action
        head 
            up
            down
            left 
            right
        arm
            side
    Reset poses
        all
        tool_1
        tool_2
        ...
"""

#!/usr/bin/env python
import rospy
import copy 
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from rgbd_grab.srv import RGBImage, PCxyz
from faster_rcnn_object_detector.srv import *
from faster_rcnn_object_detector.msg import *
from diagnostic_msgs.srv import AddDiagnostics as actionRequest
from tool_markers import server

import numpy as np

detection_label_map = {'tape':'tape',
        'plier':'pliers',
        'hamer':'hamer',
        'torque screwdriver':'yellow_screwdriver',
        'utility knife':'knife',
        'hexagon screwdriver':'hexagon_screwdriver',
        'dewalt':'clamp',
        'light':'flashlight'}

class interactiveMenu:
    def __init__(self, marker_dict, tool_dict):
        self.markers = marker_dict
        self.tool_dict = tool_dict
        self.object_held = None
        self.current_tool = None
        self.current_target = None
        self.last_entries = []
        self.object_entries = {}
        self.menu_name = 'interactive_menu'
        rospy.wait_for_service('tool_action')
        self.pcxyz_client = None
        self.rgb_image_client = None
        self.detection_client = None
        self.tool_action = rospy.ServiceProxy('tool_action', actionRequest)
        self.menu_handler = MenuHandler()
        self.initMenu()
        self.makeCubeMarker()
        self.menu_handler.apply(server[0], self.menu_name)
        server[0].applyChanges()

    def select_tool(self, feedback):
        # checkbox in selection menu: cancel last and check current
        # erase last marker and create current marker
        # set tool action entries: invisble for last and visible for current
        tool_name = self.menu_handler.getTitle(feedback.menu_entry_id)
        if not self.current_tool:
            self.current_tool = tool_name
            self.menu_handler.setCheckState(self.object_entries[tool_name][0],\
                    MenuHandler.CHECKED)
            self.select_helper_(tool_name)
        else:
            for e in self.last_entries:
                self.menu_handler.setVisible(e, False)
            self.last_entries = []
            self.menu_handler.setCheckState(\
                    self.object_entries[self.current_tool][0],\
                    MenuHandler.UNCHECKED)
            self.menu_handler.setVisible(self.grasp_entry, False)
            self.menu_handler.setVisible(self.move_entry, False)
            self.menu_handler.setVisible(self.place_entry, False)
            self.menu_handler.setVisible(self.push_entry, False)
            self.menu_handler.setVisible(self.pull_entry, False)
            self.markers[self.current_tool].erase_marker()
            if self.current_tool == tool_name:
                # Select none, cancel all here
                self.menu_handler.setCheckState(\
                        self.object_entries[self.current_tool][0],\
                        MenuHandler.UNCHECKED)
                self.current_tool = None
            else:
                self.current_tool = tool_name
                self.menu_handler.setCheckState(self.object_entries[tool_name][0],\
                        MenuHandler.CHECKED)
                self.select_helper_(tool_name)
        self.menu_handler.reApply(server[0])
        server[0].applyChanges()

    def select_helper_(self, tool_name):
        # check whether the selected tool is held by the robot
        if self.object_held:
            if self.object_held == tool_name:
                self.menu_handler.setVisible(self.move_entry, True)
                self.menu_handler.setVisible(self.place_entry, True)
        else:
            if self.tool_dict[tool_name]['action']['action_type'] == 'pick_and_place':
                self.menu_handler.setVisible(self.grasp_entry, True)
            else:
                self.menu_handler.setVisible(self.push_entry, True)
                self.menu_handler.setVisible(self.pull_entry, True)
            for i in range(1, len(self.object_entries[tool_name])):
                self.menu_handler.setVisible(\
                        self.object_entries[tool_name][i], True)
                self.last_entries.append(self.object_entries[tool_name][i])
        self.markers[tool_name].make6DofMarker()

    def head_up(self, feedback):
        resp = self.tool_action("head+up")
        if resp.success:
            rospy.loginfo("Robot's head moved up successfully.")
        else:
            rospy.loginfo("Robot's head failed to move up.")

    def head_down(self, feedback):
        resp = self.tool_action("head+down")
        if resp.success:
            rospy.loginfo("Robot's head moved down successfully.")
        else:
            rospy.loginfo("Robot's head failed to move down.")

    def head_left(self, feedback):
        resp = self.tool_action("head+left")
        if resp.success:
            rospy.loginfo("Robot's head moved left successfully.")
        else:
            rospy.loginfo("Robot's head failed to move left.")

    def head_right(self, feedback):
        resp = self.tool_action("head+right")
        if resp.success:
            rospy.loginfo("Robot's head moved right successfully.")
        else:
            rospy.loginfo("Robot's head failed to move right.")

    def arm_side(self, feedback):
        resp = self.tool_action("arm+side")
        if resp.success:
            rospy.loginfo("Robot's arm moved to side successfully.")
        else:
            rospy.loginfo("Robot's arm failed to move to side.")

    def reset_pose(self, feedback):
        tool_name = self.menu_handler.getTitle(feedback.menu_entry_id)
        if tool_name == 'All':
            for tool_name, marker in self.markers.items():
                marker.reset_pose()
        else:
            self.markers[tool_name].reset_pose()

    def get_action(self, action_type, target):
        tool_name = self.current_tool
        target_dict = self.tool_dict[tool_name]['action']['target_dict'][target]
        action_info = [action_type]
        action_info.append(target_dict['target_link'])
        action_info.append(str(target_dict['main_axis']))
        if action_type in ['push', 'pull']:
            action_info.append(str(target_dict['normal_axis']))
            action_info.append(str(target_dict['distance']))
        action_info.append(str(target_dict['prepose_offset']))
        action_info.append(str(target_dict['prepose_gripper_level']))
        action_info.append(str(target_dict['postpose_gripper_level']))
        action = '+'.join(action_info)
        return action

    def grasp_action(self, feedback):
        """
        Grasp action template: 
        pickup+{target_link}+{main_axis}+{prepose_offset}+{pre_gripper_level}+{post_gripper_level}
        """
        target = self.menu_handler.getTitle(feedback.menu_entry_id)
        action = self.get_action('pickup', target)
        resp1 = self.tool_action(action)
        rospy.loginfo("Grasping action triggered on {}.".format(target))
        if resp1.success:
            rospy.loginfo("Grasping action completed.")
            self.menu_handler.setVisible(self.place_entry, True)
            self.menu_handler.setVisible(self.move_entry, True)
            self.menu_handler.setVisible(self.grasp_entry, False)
            self.menu_handler.reApply(server[0])
            server[0].applyChanges()
            self.object_held = self.current_tool 
            self.current_target = target
        else:
            rospy.loginfo("Grasping action failed.")

    def place_action(self, feedback):
        """
        Place action template: 
        place+{target_link}+{main_axis}+{prepose_offset}+{pre_gripper_level}
        """
        target = self.current_target
        action = self.get_action('place', target)
        resp1 = self.tool_action(action)
        rospy.loginfo("Placing action triggered on {}.".format(target))
        if resp1.success:
            rospy.loginfo("Placing action completed.")
            self.menu_handler.setVisible(self.place_entry, False)
            self.menu_handler.setVisible(self.move_entry, False)
            self.menu_handler.setVisible(self.grasp_entry, True)
            self.menu_handler.reApply(server[0])
            server[0].applyChanges()
            self.object_held = None
            self.current_target = None
        else:
            rospy.loginfo("Placing action failed.")

    def move_action(self, feedback):
        """
        Move action template: move
        """
        rospy.loginfo("Moving action triggered on {}.".format(self.current_tool))
        resp1 = self.tool_action("move")
        if resp1.success:
            rospy.loginfo("Moving action completed.")
        else:
            rospy.loginfo("Moving action failed.")

    def push_action(self, feedback):
        """
        Push action template: 
        push+{target_link}+{main_axis}+{normal_axis}+{push_distance}+{prepose_offset}+{pre_gripper_level}+{pose_gripper_level}
        """
        target = self.menu_handler.getTitle(feedback.menu_entry_id)
        resp1 = self.tool_action(self.get_action('push', target))
        rospy.loginfo("Pushing action triggered on {}.".format(target))
        if resp1.success:
            rospy.loginfo("Pushing action completed.")
        else:
            rospy.loginfo("Pushing action failed.")

    def pull_action(self, feedback):
        """
        Pull action template: 
        pull+{target_link}+{main_axis}+{normal_axis}+{pull_distance}+{prepose_offset}+{pre_gripper_level}+{post_gripper_level}
        """
        target = self.menu_handler.getTitle(feedback.menu_entry_id)
        resp1 = self.tool_action(self.get_action('pull', target))
        rospy.loginfo("Pulling action triggered on {}.".format(target))
        if resp1.success:
            rospy.loginfo("Pulling action completed.")
        else:
            rospy.loginfo("Pulling action failed.")

    def handle_detection_result_(self, objects):
        """
        For each detected tool, return the index of the center pixel in the 
        Highest scored bounding box
        """
        tool_pixel_index_map = {}
        for tool in objects:
            if tool.label in detection_label_map:
                tool_name = detection_label_map[tool.label]
                target_bbox = np.argmax(tool.score)
                ix = (tool.bbox_xmin[target_bbox]+tool.bbox_xmax[target_bbox])//2
                iy = (tool.bbox_ymin[target_bbox]+tool.bbox_ymax[target_bbox])//2
                tool_pixel_index_map[tool_name] = [ix, iy]
        return tool_pixel_index_map

    def tool_detection(self, feedback):
        """
        Execute tool detection model and initialize poses for detected tools
        Steps:
            1. Get current rgb image from robot camera
            2. Run tool detection on image from step #1
            3. Process detection result to get highest scored bbox
            4. Get xyz point cloud of the center pixel of each bbox from step #3
            5. Use xyz poses from step #4 to initialize corresponding meshes
        """
        if not self.pcxyz_client:
            rospy.wait_for_service('rgb_image_srv')
            rospy.wait_for_service('get_xyz_srv')
            rospy.wait_for_service('object_detection')
            self.pcxyz_client = rospy.ServiceProxy('get_xyz_srv', PCxyz)
            self.rgb_image_client = rospy.ServiceProxy('rgb_image_srv', RGBImage)
            self.detection_client = rospy.ServiceProxy('object_detection', ImageToObject)
        rgb_image = self.rgb_image_client(False, "").im
        objects = self.detection_client(rgb_image).objects
        tool_pixel_index_map = self.handle_detection_result_(objects)
        for tool_name, pixel in tool_pixel_index_map.items():
            trans = self.pcxyz_client(pixel[0], pixel[1], "")
            self.markers[tool_name].updateTrans(trans)

    def makeCubeMarker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "tool/base_link"
        int_marker.pose.position.z = 2.0
        int_marker.scale = 0.3
        int_marker.name = self.menu_name 
        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        control.always_visible=True
        cubeMarker = Marker()
        cubeMarker.type = Marker.CUBE
        cubeMarker.scale.x = 0.3
        cubeMarker.scale.y = 0.3
        cubeMarker.scale.z = 0.3
        cubeMarker.color.r = 1.0
        cubeMarker.color.g = 0
        cubeMarker.color.b = 0
        cubeMarker.color.a = 1.0
        control.markers.append(cubeMarker)
        int_marker.controls.append(control)
        server[0].insert(int_marker)

    def initMenu(self):
        # Top menu
        object_entry = self.menu_handler.insert('Select object')
        tool_action_entry = self.menu_handler.insert('Request tool actions')
        robot_action_entry = self.menu_handler.insert('Request robot actions')
        tool_detection_entry = self.menu_handler.insert('Tool detection', \
                callback=self.tool_detection)
        reset_entry = self.menu_handler.insert('Reset poses')

        # Sub menu for robot action
        head_action_entry = self.menu_handler.insert('Head', \
                parent=robot_action_entry)
        arm_action_entry = self.menu_handler.insert('Arm', \
                parent=robot_action_entry)

        # Sub menu for head action
        head_up_entry = self.menu_handler.insert('Up', \
                parent=head_action_entry, callback=self.head_up)
        head_down_entry = self.menu_handler.insert('Down', \
                parent=head_action_entry, callback=self.head_down)
        head_left_entry = self.menu_handler.insert('Left', \
                parent=head_action_entry, callback=self.head_left)
        head_right_entry = self.menu_handler.insert('Right', \
                parent=head_action_entry, callback=self.head_right)

        # Sub menu for arm action
        arm_side_entry = self.menu_handler.insert('Side', \
                parent=arm_action_entry, callback=self.arm_side)

        # Submenu for tool action
        self.grasp_entry = self.menu_handler.insert('Grasp', \
                parent=tool_action_entry)
        self.move_entry = self.menu_handler.insert('Move', \
                parent=tool_action_entry, callback=self.move_action)
        self.place_entry = self.menu_handler.insert('Place', \
                parent=tool_action_entry, callback=self.place_action)
        self.push_entry = self.menu_handler.insert('Push', \
                parent=tool_action_entry)
        self.pull_entry = self.menu_handler.insert('Pull', \
                parent=tool_action_entry)
        self.menu_handler.setVisible(self.grasp_entry, False)
        self.menu_handler.setVisible(self.move_entry, False)
        self.menu_handler.setVisible(self.place_entry, False)
        self.menu_handler.setVisible(self.push_entry, False)
        self.menu_handler.setVisible(self.pull_entry, False)

        # Sub mune for reset
        reset_all_entry = self.menu_handler.insert('All', \
                parent=reset_entry, callback=self.reset_pose)

        # Sub menu for tool action and object and reset pose
        for tool_name, tool_info in self.tool_dict.items():
            # object entry
            tool_entry = self.menu_handler.insert(tool_name, \
                    parent=object_entry, callback=self.select_tool)
            self.object_entries[tool_name] = [tool_entry]

            self.menu_handler.setCheckState(tool_entry, MenuHandler.UNCHECKED)
            # tool action entry
            if tool_info['action']['action_type'] == 'pick_and_place':
                for t, t_info in tool_info['action']['target_dict'].items():
                    target_entry = self.menu_handler.insert(t, \
                            parent=self.grasp_entry, callback=self.grasp_action)
                    self.menu_handler.setVisible(target_entry, False)
                    self.object_entries[tool_name].append(target_entry)
            else:
                for t, t_info in tool_info['action']['target_dict'].items():
                    push_target_entry = self.menu_handler.insert(t, \
                            parent=self.push_entry, callback=self.push_action)
                    pull_target_entry = self.menu_handler.insert(t, \
                            parent=self.pull_entry, callback=self.pull_action)
                    self.menu_handler.setVisible(push_target_entry, False)
                    self.menu_handler.setVisible(pull_target_entry, False)
                    self.object_entries[tool_name].append(push_target_entry)
                    self.object_entries[tool_name].append(pull_target_entry)
            # Reset entry
            self.menu_handler.insert(tool_name, \
                    parent=reset_entry, callback=self.reset_pose)
