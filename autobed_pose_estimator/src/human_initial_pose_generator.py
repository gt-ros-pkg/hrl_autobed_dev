#!/usr/bin/python
import pickle as pkl

def main():
    '''Makes a pickle file that contains a dictionary of pos_id matched with strings
    that are to be put into the gazebo_ragdoll_plugin.cc'''
    postures_supine = [3.14]#[2.965, 3.14, 3.315]
    thigh_angles_supine = [0.0, 0.139, 0.279]
    arm_angles_supine = [0.139, 0.279]#[0.0, 0.139, 0.279]
    thigh_angles_lateral = [-0.785]#, -0.959, -1.134]
    knee_angles_lateral = [0.785]#, 0.959, 1.134]
    elbow_angles_lateral = [-0.174]#, -0.523, -1.047, -1.57]
    shoulder_lateral = -1.57
    pos_id = 0
    poses_dict = {}
    '''Supine Position'''
    for whole_body in postures_supine:
        for thigh_l in thigh_angles_supine:
            for thigh_r in thigh_angles_supine:
                for arm_l in arm_angles_supine:
                    for arm_r in arm_angles_supine:
                        pos_id = pos_id + 1
                        posture = "<pose> 2.10 0.0 0.9 1.57 0.0 "+str(whole_body)+" </pose>"
                        plugin_entry = "float joint_angles[18] = {0.0, 0.0,"+str(thigh_l)+", "+str(-thigh_r)+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "+str(arm_l)+", "+str(-arm_r)+", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};"
                        poses_dict[pos_id] = [posture, plugin_entry]

    '''Right Lateral Position'''
    '''
    posture = "<pose> 2.10 0.0 0.95 -0.3 0.0 3.14 </pose>"
    for thighs in thigh_angles_lateral:
        for knees in knee_angles_lateral:
            for elbows in elbow_angles_lateral:
                pos_id = pos_id + 1
                plugin_entry = "float joint_angles[17] = {"+str(thighs)+", "+str(thighs)+", 0.0, 0.0,"+str(knees)+", "+str(knees)+", 0.0, 0.0,"+str(shoulder_lateral)+", "+str(shoulder_lateral)+", 0.0, 0.0, 0.0, "+str(elbows)+", "+str(elbows)+", 0.0, 0.0};"
                poses_dict[pos_id] = [posture, plugin_entry]

    '''
    '''Left Lateral Position'''
    '''
    posture = "<pose> 2.10 0.0 0.95 3.44 0.0 3.14 </pose>"
    for thighs in thigh_angles_lateral:
        for knees in knee_angles_lateral:
            for elbows in elbow_angles_lateral:
                pos_id = pos_id + 1
                plugin_entry = "float joint_angles[17] = {"+str(thighs)+", "+str(thighs)+", 0.0, 0.0,"+str(knees)+", "+str(knees)+", 0.0, 0.0,"+str(shoulder_lateral)+", "+str(shoulder_lateral)+", 0.0, 0.0, 0.0, "+str(elbows)+", "+str(elbows)+", 0.0, 0.0};"
                poses_dict[pos_id] = [posture, plugin_entry]

    '''

    print poses_dict
    pkl.dump(poses_dict, open("human_poses_list.p", "wb"))
    print "Done"

if __name__ == "__main__":
    main()
