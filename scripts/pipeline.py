#! /bin/python3 

import subprocess
import rospy

def run_node(executable):
    print("Running", executable)
    command = ['rosrun', 'smooth_sailing', executable]
    subprocess.Popen(command).wait()
    print("Finished", executable)

if __name__ == "__main__":
    rospy.init_node("pipeline_node")
    
    print("Running pipeline in workspace:", rospy.get_param("/ws", "UNDEFINED"))
    
    if rospy.get_param("run_workspace_preparation", False):
        run_node('prepare_workspace.py')

    if rospy.get_param("run_navigation", False):
        run_node('navigation')
        
    if rospy.get_param("run_navigation_evaluation", False):
        run_node('evaluate_navigation.py')
        
    if rospy.get_param("run_ship_data_extraction", False):
        run_node('ship_data_extraction.py')
        
    if rospy.get_param("run_alignment_estimator", False):
        run_node('estimate_alignment_matrix')
        
    if rospy.get_param("run_mapping", False):
        run_node('mapping')
        
    if rospy.get_param("run_fov_generator", False):
        run_node('generate_fov_masks.py')
    
    if rospy.get_param("run_post_processing", False):
        run_node('post_processing.py')
        
    if rospy.get_param("run_image_reconstruction", False):
        run_node('image_reconstruction.py')
        
    print("Pipeline finished")