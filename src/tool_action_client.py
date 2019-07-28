#!/usr/bin/env python
import rospy
from tool_meshes.srv import *

def tool_action_client():
    rospy.wait_for_service('tool_action')
    tool_action = rospy.ServiceProxy('tool_action', actionRequest)
    resp1 = tool_action("clamp", "pick")
    print(resp1.succeed)


if __name__ == "__main__":
    tool_action_client()
