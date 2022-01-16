import sys

import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True, threshold=sys.maxsize)

def get_diff_index(state):
    diff=[]
    i = 0
    while(i < state.shape[0]-1):
        start = i
        if state[i] == state[i+1]:
            i+=1
            continue

        while(i < state.shape[0]-1 and state[i] != state[i+1]):
            i+=1
        end = i
        res = [start,end]
        diff.append(res)
    return diff

def seperateStates(path):

    group_index = np.array([0,1])
    new_path = path[0,:]
    for i in range(0,path.shape[0]-2):
        state1 = path[i,:]
        state2 = path[i+1,:]
        diff_index = np.array(np.where((state1==state2)==False))[0]

        intersect = np.intersect1d(diff_index, group_index)
        if intersect.shape[0] >0:
            s_grnew = new_path[-1:,]
            for gr_index in intersect:
                s_grnew[gr_index] = state2[gr_index]
            new_path = np.vstack((new_path, s_grnew))

        for index in diff_index:
            s_new = new_path[-1:,]
            if (index not in group_index):
                s_new[index] = state2[index]
                new_path = np.vstack((new_path, s_new))

    return new_path

def plot_solutionFINAL(filepath):
    with open(filepath) as f:
        lines = f.readlines()
        joints_ = []
        for line in lines:
            joint_input = []
            line = line.split()
            if line == []:
                continue
            for i in range(0, len(line)):
                joint_input.append(float(line[i]))
            joints_.append(joint_input)
    joints_ = np.asarray(joints_)
    #res_path = seperateStates(joints_) already in C
    res_path = joints_
    x = np.arange(0, res_path.shape[0])
    joints = []

    for i in range(res_path.shape[1]):
        joints.append(res_path[:, i])
    intervals = []
    for joint in joints:
        intervals.append(get_diff_index(joint))

    colors = ['blue', 'red', 'orange','green', 'green']
    labels = [ 'door1', 'door2', 'door3','cube', 'cube']
    fig, ax = plt.subplots()
    for i in range(0, len(joints)):
        ax.plot(x, joints[i], label=labels[i], color=colors[i])

    for i in range(0, len(intervals)):
        for intv in intervals[i]:
            ax.axvspan(intv[0], intv[1], facecolor=colors[i], alpha=0.3)
    plt.legend()
    plt.show()


def main():
    plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/maze3doors.txt")
    plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/maze3doors_pathsep.txt")
    #plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/maze_vertical.txt")
if __name__ == "__main__":
    main()