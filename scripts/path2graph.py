import sys

import numpy as np
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True, threshold=sys.maxsize)

def get_diff_index(state):
    diff=[]
    i = 0
    while(i < state.shape[0]-1):
        start = i
        if np.isclose(state[i], state[i + 1]):
            i+=1
            continue

        while i < state.shape[0]-1 and not np.isclose(state[i],state[i+1]):
            i+=1
        end = i
        res = [start,end]
        diff.append(res)
    return diff

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

    colors = ['green', 'green','blue', 'red', 'orange']
    labels = ['cube', 'cube','door1', 'door2', 'door3']

    #colors = ['green', 'green','blue', 'orange', 'orange']
    #labels = ['cube', 'cube','door1', 'obs', 'obs']

    #colors = ['blue', 'red', 'orange','green', 'green']
    #labels = [ 'door1', 'door2', 'door3','cube', 'cube']

    fig, ax = plt.subplots()
    for i in range(0, len(joints)):
        ax.plot(x, joints[i], label=labels[i], color=colors[i])

    for i in range(0, len(intervals)):
        for intv in intervals[i]:
            if i == 1:
                print(intv)
            ax.axvspan(intv[0], intv[1], facecolor=colors[i], alpha=0.3)
    plt.legend()
    plt.show()


def main():
    plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/mazeBEFORE.txt")
    plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/maze3doors.txt")
   # plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/mazeAFTER.txt")
   # plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/maze_sol_pathiso.txt")
 #   plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/REPAIR.txt")

    #plot_solutionFINAL("/home/serboba/rb_ws/devel/lib/robowflex_dart/mazesolution.txt")
if __name__ == "__main__":
    main()
