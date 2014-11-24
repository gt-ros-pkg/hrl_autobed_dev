#!/usr/bin/python
import pickle as pkl
import sys

def main(pose_id):
    '''Takes the pickle file that we have already stored with a dictionary of poses in it.
    Then it deletes a line in the ragdoll_model2_plugin.cc and replaces the line with a 
    line taken from the dictionary corresponding to the POSE_ID. This line is just a C++ declaration
    of the array from which the initial pose of the sleeping human is defined. By changing this line in
    the plugin, we are changing the initial pose in which the model starts.
    Of course, the shell script start_autobed_training.sh will then compile the above .cc file before it
    uses it in the gazebo simulation.
    REMEMBER TO REMOVE THE OLD FILE AND RENAME  THE newly created model3_plugin.cc using the shell script.
    '''
    poses_dict = pkl.load(open("/home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/autobed_pose_estimator/src/human_poses_list.p", "rb"))
    plugin_file = open('/home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/ros_ragdoll_model2_plugin.cc', "r")
    sdf_file = open('/home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_original.sdf')

    plugin_lines = plugin_file.readlines()
    sdf_lines = sdf_file.readlines()

    plugin_file.close()
    sdf_file.close()
    
    plugin_file_to_write = open('/home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/ros_ragdoll_model3_plugin.cc', "wb")
    sdf_file_to_write = open('/home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_temporary.sdf', "wb")

    plugin_string_to_search = "float joint_angles"
    sdf_string_to_search = "<pose>"

    count = 0


    for line in plugin_lines:
        if not plugin_string_to_search in line:
            plugin_file_to_write.write(line)
        else:
            plugin_file_to_write.write(poses_dict[pose_id][1]+'\n')

    plugin_file_to_write.close()
 
    for line in sdf_lines:
        if (not sdf_string_to_search in line) or (count >= 1):
            sdf_file_to_write.write(line)
        else:
            sdf_file_to_write.write(poses_dict[pose_id][0]+'\n')
            count = count + 1

    sdf_file_to_write.close() 


if __name__ == "__main__":
    main(int(sys.argv[1]))
