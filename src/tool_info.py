#!/usr/bin/env python
tool_list = [{'names':['clamp'], 'links':['/clamp_link']},
        {'names':['pliers'], 'links':['/left_pliers_link']}, 
        {'names':['haxegon_screwdriver'], 'links':['/haxegon_screwdriver_link']}, 
        {'names':['yellow_screwdriver'], 'links':['/yellow_screwdriver_link']}, 
        {'names':['tape'], 'links':['/tape_link']}, 
        {'names':['knife'], 'links':['/knife_link']}, 
        {'names':['hamer_head', 'hamer_handle'], 'links':['/hamer_head_link', '/hamer_handle_link']}, 
        {'names':['flashlight'], 'links':['/flashlight_link']}]


"""
tool_dict = {
    tool_1:{
        'action':{
            'action_type':'pick_and_place'/'push_and_pull',
            'target_dict':{
                'target_name':{
                    'target_link':'',
                    'prepose_offset':0.10,
                    'prepose_gripper_level':0.4,
                    'postpose_gripper_level':0.1,
                    'main_axis': 0
                },
                ...
            }
        },
        'marker_link':'',
        'parent_link':'',
        'original_pose':[x, y, z]
    },
    ...
}
pick_and_place action type includes actions: grasp, move, place
push_and_pull action type includes actions: push, pull
push/pull: target_link
           normal_axis=0, 
           main_axis=0, 
           distance=0.3,
           prepose_offset=0.1,
           prepose_gripper_level=0.4,
           postpose_gripper_level=0.1

"""
tool_dict = {
        'tool_chest':{
            'action':{
                'action_type':'push_and_pull',
                'target_dict':{
                    'shelf1':{
                        'target_link':'tool/shelf1_pickup_link',
                        'normal_axis': 0,
                        'main_axis': 1,
                        'distance': 0.227,
                        'prepose_offset':0.05,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.1,
                        },
                    'shelf2':{
                        'target_link':'tool/shelf2_pickup_link',
                        'normal_axis': 0,
                        'main_axis': 1,
                        'distance': 0.227,
                        'prepose_offset':0.05,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.1,
                        },
                    'shelf3':{
                        'target_link':'tool/shelf3_pickup_link',
                        'normal_axis': 0,
                        'main_axis': 1,
                        'distance': 0.227,
                        'prepose_offset':0.05,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.1,
                        },
                    'shelf4':{
                        'target_link':'tool/shelf4_pickup_link',
                        'normal_axis': 0,
                        'main_axis': 1,
                        'distance': 0.227,
                        'prepose_offset':0.05,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.1,
                        }
                    },
                },
            'marker_link':'tool/tool_chest_link',
            'parent_link':'tool/base_link',
            'original_pose':[2, 2, 0.2]
            },
        'clamp': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'clamp_body':{
                        'target_link':'tool/clamp_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':0
                        }
                    }
                },
            'marker_link':'tool/clamp_link',
            'parent_link':'tool/base_link',
            'original_pose':[1, 0, 0]
            },
        'pliers': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'left_plier':{
                        'target_link':'tool/plier_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':1
                        },
                    'right_plier':{
                        'target_link':'tool/plier_right_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':1,
                        }
                    }
                },
            'marker_link':'tool/left_pliers_link',
            'parent_link':'tool/base_link',
            'original_pose':[1, 1, 0]
            },
        'haxegon_screwdriver': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'haxegon_screwdriver':{
                        'target_link':'tool/haxegon_screwdriver_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.7,
                        'postpose_gripper_level':0.0,
                        'main_axis':1,
                        }
                    }
                },
            'marker_link':'tool/haxegon_screwdriver_link',
            'parent_link':'tool/base_link',
            'original_pose':[2, 1, 0]
            },
        'yellow_screwdriver': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'yellow_screwdriver_handle':{
                        'target_link':'tool/yellow_screwdriver_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':2
                        }
                    }
                },
            'marker_link':'tool/yellow_screwdriver_link',
            'parent_link':'tool/base_link',
            'original_pose':[1, 2, 0]
            },
        'tape': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'tape':{
                        'target_link':'tool/tape_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.1,
                        'main_axis':1
                        }
                    }
                },
            'marker_link':'tool/tape_link',
            'parent_link':'tool/base_link',
            'original_pose':[1, -2, 0]
            },
        'knife': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'knife_handle':{
                        'target_link':'tool/knife_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':0
                        }
                    }
                },
            'marker_link':'tool/knife_link',
            'parent_link':'tool/base_link',
            'original_pose':[2, 0, 0]
            },
        'hamer': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'hamer_handle':{
                        'target_link':'tool/hamer_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':1
                        }
                    }
                },
            'marker_link':'tool/hamer_handle_link',
            'parent_link':'tool/base_link',
            'original_pose':[2, -1, 0]
            },
        'flashlight': {
            'action':{
                'action_type':'pick_and_place',
                'target_dict':{
                    'flashlight_body':{
                        'target_link':'tool/flashlight_pickup_link',
                        'prepose_offset':0.10,
                        'prepose_gripper_level':0.4,
                        'postpose_gripper_level':0.0,
                        'main_axis':2
                        }
                    }
                },
            'marker_link':'tool/flashlight_link',
            'parent_link':'tool/base_link',
            'original_pose':[1, -1, 0]
            },
        }
