#!/usr/bin/env python
import rospy
import copy 


from geometry_msgs.msg import Pose
from tool_markers import *
from interactive_menu import * 
from tool_info import tool_dict

if __name__=="__main__":
    rospy.init_node("tools_controls")

    server.append(InteractiveMarkerServer("tools_controls"))

    tool_markers = {} 
    for tool, tool_info in tool_dict.items():
        name = tool
        link_name = tool_info['marker_link']
        parent_link = tool_info['parent_link']
        init_pose = tool_info['original_pose']
        tool_markers[name] = toolInteractiveMarker(\
                link_name, parent_link, name, init_pose)
    menu = interactiveMenu(tool_markers, tool_dict)
    server[0].applyChanges()
    rospy.spin()
