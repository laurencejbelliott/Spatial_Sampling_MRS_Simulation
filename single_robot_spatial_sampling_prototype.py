__author__ = "Laurence Roberts-Elliott"
import simpy
import numpy as np
import random
import matplotlib.pyplot as plt
from astar_python import Astar
from gaussian import makeGaussian

world_width = 10
world_height = 10
gaussian = makeGaussian(world_height)
sampled = np.zeros((world_width, world_height))
movement_matrix = np.ones((world_width, world_height))
a_star = Astar(movement_matrix)
# Measured in m/s, max speed of Leo rover
robot_speed = 0.4
robot_pos = [0, 0]
sample_locations = [[9, 5], [5, 5], [4, 4], [3, 3]]


def main():
    env = simpy.Environment()
    env.process(move_robot_and_sample(env))
    env.run(until=120)
    print("Simulation complete at t=", str(env.now), "final robot pos:", str(robot_pos))


def move_robot_and_sample(env):
    global robot_pos
    for sample_location in sample_locations:
        path = a_star.run(robot_pos, sample_location)
        for s in range(0, len(path)):
            robot_pos = path[s]
            print("\n", str(robot_pos), "at t =", str(env.now))
            fig = plt.figure()
            ax = fig.gca()
            ax.set_xticks(np.arange(0, world_width, 1))
            ax.set_yticks(np.arange(0, world_height, 1))
            plt.xlim([0, world_width])
            plt.ylim([0, world_height])
            plt.imshow(sampled, cmap="hot", interpolation="nearest",
                       vmin=0, vmax=1)
            plt.scatter([path[s][0]], [path[s][1]])
            plt.grid()
            plt.title(str(path[s]) + " at t = " + str(env.now))
            plt.show()

            if s != len(path) - 1:
                if (abs(path[s][0] - path[s + 1][0]) > 0 and
                        abs(path[s][1] - path[s + 1][1]) > 0):
                    yield env.timeout(1.5 * robot_speed)
                else:
                    yield env.timeout(robot_speed)
            else:
                sampled[path[s][1], path[s][0]] = gaussian[path[s][1], path[s][0]]
                fig = plt.figure()
                ax = fig.gca()
                ax.set_xticks(np.arange(0, world_width, 1))
                ax.set_yticks(np.arange(0, world_height, 1))
                plt.xlim([0, world_width])
                plt.ylim([0, world_height])
                plt.imshow(sampled, cmap="hot", interpolation="nearest",
                           vmin=0, vmax=1)
                plt.scatter([path[s][0]], [path[s][1]])
                plt.grid()
                plt.title(str(path[s]) + " at t = " + str(env.now))
                plt.show()
                print("Sampled value:", gaussian[path[s][1], path[s][0]], "at", str(robot_pos))
                yield env.timeout(30)


if __name__ == "__main__":
    main()
