#! /bin/python3 

import subprocess
import rospy

def launch_node(package, executable, *args):
    command = ['rosrun', package, executable] + list(args)
    return subprocess.Popen(command)

if __name__ == "__main__":
    rospy.init_node("pipeline_node")
    
    if rospy.get_param("run_cookbook", False):
        print("Running cookbook")
        node1 = launch_node('smooth_sailing' ,'cookbook.py')
        node1.wait()
        print("Finished cookbook")
    
    if rospy.get_param("run_navigation", False):
        print("Running navigation")
        node2 = launch_node('smooth_sailing' ,'navigation')
        node2.wait()
        print("Finished navigation")
    
    print("Pipeline finished for ws:", rospy.get_param("/ws", "UNDEFINED"))