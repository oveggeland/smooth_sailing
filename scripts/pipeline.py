#! /bin/python3 

import subprocess
import rospy

def launch_node(package, executable, *args):
    command = ['rosrun', package, executable] + list(args)
    return subprocess.Popen(command)

if __name__ == "__main__":
    rospy.init_node("pipeline_node")
    
    if rospy.get_param("run_workspace_preparation", False):
        print("Run workspace preparation")
        node1 = launch_node('smooth_sailing' ,'prepare_workspace.py')
        node1.wait()
        print("Finished workspace preperation")
    
    if rospy.get_param("run_navigation", False):
        print("Running navigation")
        node2 = launch_node('smooth_sailing' ,'navigation')
        node2.wait()
        print("Finished navigation")
        
    if rospy.get_param("run_navigation_evaluation", False):
        print("Running navigation evaluation")
        node2 = launch_node('smooth_sailing' ,'evaluate_navigation.py')
        node2.wait()
        print("Finished navigation evaluation")
        
    if rospy.get_param("run_ship_data_extraction", False):
        print("Running ship data extraction")
        node2 = launch_node('smooth_sailing' ,'ship_data_extraction.py')
        node2.wait()
        print("Finished ship data extraction")
        
    if rospy.get_param("run_mapping", False):
        print("Running mapping")
        node2 = launch_node('smooth_sailing' ,'mapping')
        node2.wait()
        print("Finished mapping")
    
    if rospy.get_param("run_post_processing", False):
        print("Running post processing")
        node2 = launch_node('smooth_sailing' ,'post_processing.py')
        node2.wait()
        print("Finished post processing")
        
    print("Pipeline finished for ws:", rospy.get_param("/ws", "UNDEFINED"))