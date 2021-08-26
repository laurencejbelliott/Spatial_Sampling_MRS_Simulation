__author__ = "Laurence Roberts-Elliott"
import simpy
import os
import glob
import cProfile
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
robots = []


def draw_map(robots, env):
    plt.close('all')
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
    plt.savefig(figure_dir + "t_" + str(float(env.now)).replace(".", "_"))


class Robot:
    def __init__(self, start_pos, rob_id, env):
        self.pos = start_pos
        self.id = rob_id
        self.task = None
        self.color = [random.randrange(50, 256) / 255, random.randrange(50, 256) / 255,
                      random.randrange(50, 256) / 255]

    def move_robot_and_sample(self, env, sample_location):
        path = a_star.run(self.pos, sample_location)
        for s in range(0, len(path)):
            self.pos = path[s]
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
    global robots
    num_robots = 5
    robots = [Robot([random.randrange(0, world_width),
                     random.randrange(0, world_height)], r, env) for r in range(0, num_robots)]

    env.process(multi_robot_sampling(env))
    env.run()

    # for robot in robots:
    #     sample_location = [random.randrange(0, world_width), random.randrange(0, world_height)]
    #     while sample_location in sample_locations:
    #         sample_location = [random.randrange(0, world_width), random.randrange(0, world_height)]
    #     sample_locations.append(sample_location)
    #     env.process(robot.move_robot_and_sample(env, sample_location))


def multi_robot_sampling(env):
    sample_locations = []

    while len(sample_locations) < (world_width*world_height) / 4:
        for robot in robots:
            sample_location = [random.randrange(0, world_width), random.randrange(0, world_height)]
            while sample_location in sample_locations:
                sample_location = [random.randrange(0, world_width), random.randrange(0, world_height)]
            if robot.task is None:
                sample_locations.append(sample_location)
                robot.task = env.process(robot.move_robot_and_sample(env, sample_location))
                yield robot.task
            elif robot.task is not None:
                # print(robot.task.processed, env.now)
                if robot.task.processed:
                    sample_locations.append(sample_location)
                    robot.task = env.process(robot.move_robot_and_sample(env, sample_location))
                    yield robot.task
                else:
                    continue


if __name__ == "__main__":
    main()
    # cProfile.run('main()')
