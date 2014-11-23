#!/usr/bin/python
import pickle as pkl

def main():
    '''Makes a pickle file that contains a dictionary of pos_id matched with strings
    that are to be put into the gazebo_ragdoll_plugin.cc'''
    thigh_angles_supine = [0.0, 0.139, 0.279, 0.419]
    thigh_angles_lateral = [0.785, 0.959, 1.134]
    knee_angles_lateral = [0.785, 0.959, 1.134]
    elbow_angles_lateral = [0.174, 0.523, 1.047, 1.57]
    pos_id = 0
    poses_dict = {}
    '''Supine Position'''
    for thigh_l in thigh_angles_supine:
        for thigh_r in thigh_angles_supine:
            pos_id = pos_id + 1
            plugin_entry = "float joint_angles[17] = {0.0, 0.0,"+str(thigh_l)+", "+str(-thigh_r)+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};"
            poses_dict[pos_id] = plugin_entry

    '''Right Lateral Position'''
    for thighs in thigh_angles_lateral:
        for knees in knee_angles_lateral:
            for elbows in elbow_angles_lateral:
                pos_id = pos_id + 1
                plugin_entry = "float joint_angles[17] = {"+str(thighs)+", "+str(thighs)+", 0.0, 0.0,"+str(knees)+", "+str(knees)+", 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0, "+str(elbows)+", "+str(elbows)+", 0.0, 0.0};"
                poses_dict[pos_id] = plugin_entry


    '''Left Lateral Position'''
    for thighs in thigh_angles_lateral:
        for knees in knee_angles_lateral:
            for elbows in elbow_angles_lateral:
                pos_id = pos_id + 1
                plugin_entry = "float joint_angles[17] = {"+str(thighs)+", "+str(thighs)+", 0.0, 0.0,"+str(knees)+", "+str(knees)+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "+str(elbows)+", "+str(elbows)+", 0.0, 0.0};"
                poses_dict[pos_id] = plugin_entry



    print poses_dict
    pkl.dump(poses_dict, open("human_poses_list.p", "wb"))


if __name__ == "__main__":
    main()
