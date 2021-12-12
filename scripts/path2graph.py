import numpy as np
import matplotlib.pyplot as plt

def read_solution():
    cube1_1 = []
    cube1_2 = []
    cube2_1 = []
    cube2_2 = []
    door = []
    door_lock = []
    with open("/home/serboba/rb_ws/devel/lib/robowflex_dart/xml_files/solution_path.txt") as f:
        lines = f.readlines()
        cube1_1 = [float(line.split()[0]) for line in lines]
        cube1_2 = [float(line.split()[1]) for line in lines]
        cube2_1 = [float(line.split()[2]) for line in lines]
        cube2_2 = [float(line.split()[3] )for line in lines]
        door = [float(line.split()[4]) for line in lines]
        door_lock = [float(line.split()[5]) for line in lines]

    x = np.arange(0,len(door))
    fig, ax = plt.subplots(3)

    ax[0].plot(x,door,label='door')
    ax[0].plot(x,door_lock,label='door_lock')
    ax[0].legend()
    ax[1].plot(x,cube1_1,label='cube_1')
    ax[1].plot(x,cube1_2,label='cube_1_1')
    ax[1].legend()
    ax[2].plot(x,cube2_1,label='cube_2')
    ax[2].plot(x,cube2_2,label='cube_2_1')
    ax[2].legend()
    print(door_lock)
    plt.show()

def read_solution2():
    with open("/home/serboba/rb_ws/devel/lib/robowflex_dart/solution_path_table.txt") as f:
        lines = f.readlines()
        door1 = [float(line.split()[2]) for line in lines]
        door2 = [float(line.split()[3]) for line in lines]
        door3 = [float(line.split()[4]) for line in lines]
        cube1 = [float(line.split()[0]) for line in lines]
        cube1_x = [float(line.split()[1]) for line in lines]

        x = np.arange(0, len(door1))
        fig, ax = plt.subplots(2)
        print(door1)

        ax[0].plot(x, door1, label='door1')
        ax[0].plot(x, door2, label='door2')
        ax[0].plot(x, door3, label='door3')
        ax[0].legend()

        ax[1].plot(x, cube1, label='cube_1')
        ax[1].plot(x, cube1_x, label='cube_1_1')
        ax[1].legend()
        plt.show()


def main():
    read_solution2()

if __name__ == "__main__":
    main()