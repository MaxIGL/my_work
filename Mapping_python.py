  GNU nano 2.9.3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        MappingStateMachine.py                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 28 08:23:19 2021
@author: j.rimani
"""

# Import your directories
import sys
# Call the directory where the hipop package are
# sys.path.append('/home/dcas/j.rimani/hipop-python-master')
# from hddl_wrapper import hddl_wrapper

import time
import logging
import rospy
import json
import smach
import smach_ros
import roslaunch
import subprocess
import glob
import os


from math import radians, pi, sin, cos, atan2, sqrt, fabs, degrees

LOGGER = logging.getLogger(__name__)

# from move_to.msg import MoveToAction, MoveToGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import Empty
from std_msgs.msg import Bool


# Clarification -  mux: multiplex between multiple topics
# utils.mux relays on pkg="topic_tools"


# Start The State Machine
class InitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'], output_keys=['launch_package'])

    def execute(self, userdata):
        # In Which package are my launch files? - in the general state because is a general entry
        launch_package = rospy.get_param("launch_package", 'my_pcl_tutorial')
        userdata.launch_package = launch_package
        rospy.loginfo('Which Launch Package?')
        rospy.loginfo(launch_package)

        time.sleep(5)
        return 'start'


class RealTimeMappingProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['mapped'], input_keys=['launch_package'], output_keys=['launch_package'])

    def execute(self, userdata):
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, [userdata.launch_folder+userdata.launch_file_mapping])
        # launch.start()
        # rospy.sleep(3)
        # rospy.loginfo("started"+userdata.launch_file_mapping)

        # Another way is
        # cmd=["roslaunch",userdata.launch_folder,userdata.launch_file_mapping]
        # proc = subprocess.Popen(cmd)
        # time.sleep(1)  # maybe needed to wait the process to do something useful
        # proc.terminate()

        # rospy.loginfo('Launch Camera')
        # # launch first file
        # launch_file_camera = rospy.get_param("launch_file_name",'camera_launch.launch')
        # cmd=["roslaunch",userdata.launch_package,launch_file_camera]
        # launch_Camera = subprocess.Popen(cmd)
        # time.sleep(3)

        rospy.loginfo('Launch Mapping')
        launch_file_mapping = rospy.get_param("launch_file_name", 'outside_D.launch')
        cmd = ["roslaunch", userdata.launch_package, launch_file_mapping]
        launch_RealTimeMappingProcess = subprocess.Popen(cmd)
        # out, err = launch_RealTimeMappingProcess.communicate()

        time.sleep(60)

        global xx
        xx = launch_RealTimeMappingProcess

        return 'mapped'


class HumanMapVisualization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['human_readable_map_created'], input_keys=['launch_package'],
                             output_keys=['launch_package'])

    def execute(self, userdata):
        # Should this be executed after we run rosrun pcl_ros pointcloud_to_pcd
        # if that doesn't work you can use subprocess
        # package = 'pcl_ros'
        # executable = 'pointcloud_to_pcd'

        # node = roslaunch.core.Node(package, executable, args='input:=in_pointcloud_to_pcd')
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()

        # rospy.sleep(1)

        # process = launch.launch(node)
        # rospy.sleep(3)
        # process.stop()

        # Save data
        rospy.loginfo('Launch Point Cloud')
        # in_pointcloud_to_pcd = rospy.get_param("input_to_pointcloud_to_pcd",'/octomap_point_cloud_centers')
        cmd = ["rosrun","pcl_ros","pointcloud_to_pcd","input:=/octomap_point_cloud_centers","prefix:=/catkin_igluna_ws/src/state_machines/pcd/"]
        pcl_ros = subprocess.Popen(cmd)
        rospy.sleep(2)
        rospy.loginfo('Terminate Launch Point Cloud')
        pcl_ros.terminate()

        global xx
        rospy.loginfo('Terminate Launch Mapping')
        xx.terminate()

        # launch file
        # rospy.loginfo('Launch Human Visualization')
        # launch_file_vis_human = rospy.get_param("launch_file_human_vis",'read_pcd.launch')
        # launch_HumanVisualization = subprocess.Popen(["roslaunch",userdata.launch_package,launch_file_vis_human])
        # rospy.sleep(10)
        # # When do we need to kill the process?
        # launch_HumanVisualization.terminate()

        return 'human_readable_map_created'


class outlineRemoval(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cleaned_map'], input_keys=['launch_package'],
                             output_keys=['launch_package','denosing_out_file'])

    def execute(self, userdata):
        rospy.loginfo('outline removal server')
        cmd = ["rosrun", userdata.launch_package, "outliers_removal_server"]
        OutlineRemovalServer = subprocess.Popen(cmd)
        rospy.sleep(3)

        rospy.loginfo('Searching the last file')
        # is there a parameter that we have to take or the name of the file is always the same?
        list_of_files = glob.glob('/root/catkin_igluna_ws/src/state_machines/python_node_mapping/*.pcd')  # * means all if need specific format then *.csv
        latest_file = max(list_of_files, key=os.path.getctime)
        rospy.loginfo('Last File:')
        rospy.loginfo(latest_file)

        rospy.sleep(3)

        rospy.loginfo('Outline Removal Client')
        # file_in_OutlineRemoval = rospy.get_param("file_in_OutlineRemoval", '~/home/folder/test.pcd')
        file_out_OutlineRemoval = rospy.get_param("file_out_OutlineRemoval", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/test_outliersx1.pcd')
        radius_search_OutlineRemoval = rospy.get_param("radius_search_OutlineRemoval", '0.06')
        min_neighbors_in_radius_OutlineRemoval = rospy.get_param("min_neighbors_in_radius_OutlineRemoval", '4')

        cmd = ["rosrun", userdata.launch_package, "outliers_removal_client", latest_file,
               file_out_OutlineRemoval, radius_search_OutlineRemoval, min_neighbors_in_radius_OutlineRemoval]
        OutlineRemovalClient = subprocess.Popen(cmd)
        rospy.sleep(5)

        rospy.loginfo('Denoising Service')

        cmd = ["rosrun", userdata.launch_package, "denoising_server"]
        CallDenosingServer = subprocess.Popen(cmd)
        rospy.sleep(5)

        file_in_Denosing = rospy.get_param("file_in_Denosing", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/test_outliersx1.pcd')
        file_out_Denosing = rospy.get_param("file_out_Denosing", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/test_outliersx1_denoisedx1.pcd')

        # file input to the grid map
        userdata.denosing_out_file = 'test_outliersx1_denoisedx1.pcd'


        cmd = ["rosrun", userdata.launch_package, "denoising_client", file_in_Denosing, file_out_Denosing]
        CallDenosingClient = subprocess.Popen(cmd)
        rospy.sleep(5)


        return 'cleaned_map'


class GridMapGeneration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obtained_grid_map'], input_keys=['launch_package','denosing_out_file'],
                             output_keys=['launch_package'])

    def execute(self, userdata):
        rospy.loginfo('Launch GridMapLoader')
        folder_path_gridmap_loader = rospy.get_param("folder_path_gridmap_loader", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/')
        pcd_filename_gridmap_loader = rospy.get_param("pcd_filename_gridmap_loader", userdata.denosing_out_file)

        launch_GridMapLoader = rospy.get_param("launch_GridMapLoader_file", 'grid_map_pcl_loader_node.launch')
        launch_MapfromPointCloud = subprocess.Popen(['roslaunch', userdata.launch_package, launch_GridMapLoader])

        rospy.sleep(5)

        # Should the launch file be running when we launch the service or can I close it?
        rospy.loginfo('Launch Conversion Extraction Server')
        cmd = ["rosrun", userdata.launch_package, "conversion_extraction_server"]
        ExtractionServer = subprocess.Popen(cmd)
        rospy.sleep(5)

        # is there a parameter that we have to take or the name of the file is always the same?
        file_in_ExtractionServer = rospy.get_param("file_in_ExtractionServer", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/elevation_map.bag')
        file_out_ExtractionServer = rospy.get_param("file_out_ExtractionServer", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/extracted_map.pcd')

        rospy.loginfo('Launch Conversion Extraction Client')
        cmd = ["rosrun", userdata.launch_package, "conversion_extraction_client", file_in_ExtractionServer,
               file_out_ExtractionServer]
        ExtractionClient = subprocess.Popen(cmd)
        rospy.sleep(5)

        rospy.loginfo('occupancymap_generator_server')
        cmd = ["rosrun", userdata.launch_package, "occupancymap_generator_server"]
        GridMapGeneratorService = subprocess.Popen(cmd)
        rospy.sleep(5)

        rospy.loginfo('occupancymap_generator_client')
        # is there a parameter that we have to take or the name of the file is always the same?
        file_in_GridMapGenerator = rospy.get_param("file_in_GridMapGenerator", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/extracted_map.pcd')
        file_out_GridMapGenerator = rospy.get_param("file_out_GridMapGenerator", '/root/catkin_igluna_ws/src/state_machines/python_node_mapping/occupancy/2d_final_map')
        frame_id_GridMapGenerator=rospy.get_param("frame_id_GridMapGenerator",'map')
        resolution_GridMapGenerator = rospy.get_param("resolution_GridMapGenerator",'0.08')
        resolution_discretized_GridMapGenerator=rospy.get_param("resolution_discretized_GridMapGenerator",'0.3')


        cmd = ["rosrun", userdata.launch_package, "occupancymap_generator_client", file_in_GridMapGenerator,
               file_out_GridMapGenerator,frame_id_GridMapGenerator,resolution_GridMapGenerator,resolution_discretized_GridMapGenerator]
        GridMapGeneratorClient = subprocess.Popen(cmd)

        rospy.sleep(3)

        return 'obtained_grid_map'


if __name__ == '__main__':
    # State Machine Visualize the steps of the plan
    rospy.init_node('Mapping_Smach')

    # Rate
    rate = rospy.get_param("~rate", 1)

    ''' All paramaters used in the State Machines '''

    # Head visibility limits
    # head_height = rospy.get_param("~head_height", 0.116)
    # head_boarder_distance = rospy.get_param("~head_boarder_distance ", 0.03)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded'])

    with sm:
        smach.StateMachine.add('INIT', InitState(),
                               transitions={'start': 'RealTimeMappingProcess'})

        smach.StateMachine.add('RealTimeMappingProcess', RealTimeMappingProcess(),
                               transitions={'mapped': 'HumanMapVisualization'},
                               remapping={})

        smach.StateMachine.add('HumanMapVisualization', HumanMapVisualization(),
                               transitions={'human_readable_map_created': 'outlineRemoval'},
                               remapping={})

        smach.StateMachine.add('outlineRemoval', outlineRemoval(),
                               transitions={'cleaned_map': 'GridMapGeneration'},
                               remapping={})
        #
        smach.StateMachine.add('GridMapGeneration', GridMapGeneration(),
                               transitions={'obtained_grid_map': 'succeeded'},
                               remapping={})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    time.sleep(5)

    sis.stop()

    rospy.signal_shutdown('All done.')



