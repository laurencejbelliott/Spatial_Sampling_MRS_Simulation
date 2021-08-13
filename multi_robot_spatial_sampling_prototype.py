__author__ = "Laurence Roberts-Elliott"
import simpy
import os
import glob
import numpy as np
import random
import matplotlib.pyplot as plt
from astar_python import Astar
from gaussian import makeGaussian

world_width = 5
world_height = 5
gaussian = makeGaussian(world_height)
sampled = np.zeros((world_width, world_height))
movement_matrix = np.ones((world_width, world_height))
a_star = Astar(movement_matrix)
figure_dir = "sim_visualisation/"
# Measured in m/s, max speed of Leo rover
robot_speed = 0.4
sample_locations = [[9, 5], [5, 5], [4, 4], [3, 3]]
robots = []


def draw_map(robots, env):
    fig = plt.figure()
    ax = fig.gca()
    ax.set_xticks(np.arange(0, world_width, 1))
    ax.set_yticks(np.arange(0, world_height, 1))
    plt.xlim([0, world_width])
    plt.ylim([0, world_height])
    plt.imshow(sampled, cmap="gray", interpolation="nearest",
               vmin=0, vmax=1)

    for robot in robots:
        plt.scatter(robot.pos[0], robot.pos[1], c=[robot.color])
    plt.grid()
    plt.title(str(robot.pos) + " at t = " + str(env.now))
    # plt.show()
    plt.savefig(figure_dir + "t_" + str(env.now).replace(".", "_"))


class Robot:
    def __init__(self, start_pos, rob_id):
        self.pos = start_pos
        self.id = rob_id
        self.goal_pos = [0, 0]
        self.color = [random.randrange(50, 256) / 255, random.randrange(50, 256) / 255,
                      random.randrange(50, 256) / 255]

    def move_robot_and_sample(self, env, sample_location):
        path = a_star.run(self.pos, sample_location)
        for s in range(0, len(path)):
            self.pos = path[s]
            # print("\n", str(self.pos), "at t =", str(env.now))
            draw_map(robots, env)

            if s != len(path) - 1:
                if (abs(path[s][0] - path[s + 1][0]) > 0 and
                        abs(path[s][1] - path[s + 1][1]) > 0):
                    yield env.timeout(1.5 * robot_speed)
                else:
                    yield env.timeout(robot_speed)
            else:
                sampled[path[s][1], path[s][0]] = gaussian[path[s][1], path[s][0]]
                draw_map(robots, env)
                print("Robot", str(self.id), "sampled value:",
                      gaussian[path[s][1], path[s][0]], "at", str(self.pos), "at t =", str(env.now))
                yield env.timeout(30)


def main():
    old_figures = glob.glob('sim_visualisation/*')
    for f in old_figures:
        os.remove(f)

    env = simpy.Environment()
    # rob_0 = Robot([0, 0], 0)
    # rob_1 = Robot([world_width - 1, world_height - 1], 1)
    global robots
    robots = [Robot([random.randrange(0, world_width),
                     random.randrange(0, world_height)], r) for r in range(0, 4)]
    for robot in robots:
        env.process(robot.move_robot_and_sample(env, [random.randrange(0, world_width),
                                                      random.randrange(0, world_height)]))
    env.run()
    print("Simulation complete at t =", str(env.now))
    

if __name__ == "__main__":
    main()
