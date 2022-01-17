__author__ = "Laurence Roberts-Elliott"
import os
import glob
import random

import pandas as pd
from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import numpy as np
import matplotlib.pyplot as plt
from astar_python import Astar
from gaussian import makeGaussian
from kriging_utils.kriging import predict_by_kriging


class Robot(Agent):
    def __init__(self, pos, model):
        # Inherits position and model attributes from Agent superclass
        super().__init__(pos, model)
        self.pos = pos
        # The robot is assigned a randomly generated colour for visualisation
        r = lambda: self.model.random.randint(0, 255)
        # MESA represents colours in Hex, so we convert to this from RGB
        self.color = ("#%02X%02X%02X" % (r(), r(), r()))
        # Each robot has its own cost map for cost based avoidance of collision with other robots
        self.movement_matrix = np.ones((self.model.width, self.model.height))
        # The Astar class from `astar-python` provides the A* implementation
        self.astar = Astar(self.movement_matrix)
        self.goal = []
        self.path = []
        self.path_step = 0
        self.type = 0  # Agent type 0 is a robot
        self.previous_cell = None
        self.deadlocked = False
        self.distance_travelled = 0
        self.visited = np.zeros((self.model.width, self.model.height))
        self.visited[self.pos[1], self.pos[0]] += 1
        self.trajectory = [self.pos]
        self.idle_time = 0
        self.waiting_time = 0

    # Each robot will execute this method at the start of a new time step in the simulation
    def step(self):
        self.deadlocked = False
        if self.path is not None:
            if 0 < self.path_step < len(self.path) - 1:
                # Comment the following line to disable cost based collision avoidance. Updates the robot's cost map
                self.astar = Astar(self.movement_matrix)
                self.path = self.astar.run(self.pos, self.goal)

                # Check if next cell in robot path contains another robot
                if self.path is not None:
                    if len(self.path) > 1:
                        if self.path[self.path_step] is not None:
                            cell_contains_robot = False
                            # Check if the next cell in the robot's path contains a robot
                            for agent in self.model.grid.get_cell_list_contents(tuple(self.path[self.path_step])):
                                if agent.type == 0:
                                    cell_contains_robot = True

                            # Check if the next cell in the robot's path is also the next cell in another robot's path
                            # a.k.a. a deadlock
                            deadlock_cells = []
                            if not cell_contains_robot:
                                for agent in self.model.schedule.agents:
                                    if agent.type == 0:  # True if the agent is a robot
                                        if agent.path is not None and len(agent.path) > 1:
                                            if list(self.pos) == agent.path[agent.path_step]:
                                                self.deadlocked = True
                                                deadlock_cells.append(tuple(agent.path[agent.path_step]))

                                # If a deadlock or other robot is detected, move this robot to another free neighbouring
                                # cell
                                if self.deadlocked:
                                    print("Deadlock detected!")
                                    print(deadlock_cells, "\n")
                                    neighbours = []
                                    neighbour_free = False
                                    for cell in self.model.grid.iter_neighborhood(self.pos, False, False):
                                        neighbours.append(cell)
                                    while not neighbour_free:
                                        neighbour_cell_i = random.randrange(len(neighbours))
                                        neighbour_cell = neighbours[neighbour_cell_i]
                                        robot_in_cell = False
                                        for agent in self.model.grid.get_cell_list_contents(neighbour_cell):
                                            if agent.type == 0:  # If agent is a robot
                                                robot_in_cell = True
                                        if not robot_in_cell:
                                            if neighbour_cell not in deadlock_cells:
                                                neighbour_free = True
                                    print("Moving robot", self.unique_id, "from", self.pos, "to", neighbour_cell)
                                    self.model.grid.move_agent(self, neighbour_cell)
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[neighbour_cell[1], neighbour_cell[0]] += 1
                                    self.trajectory.append(neighbour_cell)

                                # If the next cell is not deadlocked, simply move the robot there
                                else:
                                    self.model.grid.move_agent(self, tuple(self.path[self.path_step]))
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[self.path[self.path_step][1], self.path[self.path_step][0]] += 1
                                    self.trajectory.append(self.path[self.path_step])
                            else:
                                print("Cell", tuple(self.path[self.path_step]), "in robot", str(self.unique_id)+"'s",
                                      "path occupied")
                                neighbours = []
                                for cell in self.model.grid.iter_neighborhood(self.pos, False, False):
                                    neighbours.append(cell)
                                free_neighbours = []
                                for neighbour_cell in neighbours:
                                    if neighbour_cell not in deadlock_cells:
                                        robot_in_cell = False
                                        for agent in self.model.grid.get_cell_list_contents(neighbour_cell):
                                            if agent.type == 0:  # If agent is a robot
                                                robot_in_cell = True
                                        if not robot_in_cell:
                                            free_neighbours.append(neighbour_cell)
                                print("Free neighbours:", free_neighbours)
                                if len(free_neighbours) > 0:
                                    free_cell = free_neighbours[random.randrange(len(free_neighbours))]

                                    print("Moving from", self.pos, "to", free_cell)
                                    self.model.grid.move_agent(self, free_cell)
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[free_cell[1], free_cell[0]] += 1
                                    self.trajectory.append(free_cell)

            # If the robot has reached the end of its path, and thus its goal, sample a value from the underlying
            # distribution
            elif self.path_step == len(self.path) - 1 or len(self.path) == 1:
                if len(self.path) == 1:
                    self.path_step = 0
                self.model.sampled[self.path[self.path_step][1], self.path[self.path_step][0]] = \
                    self.model.gaussian[self.path[self.path_step][0], self.path[self.path_step][1]]

                # Check goal cell for SampledCell agent (i.e. already sampled)

                for a in self.model.grid.get_cell_list_contents((self.path[self.path_step][0],
                                                                 self.path[self.path_step][1])):
                    if a.type == 2:
                        self.model.grid.remove_agent(a)

                # For the purpose of visualisation in MESA, the sampled cell is instantiated as an agent and placed on
                # the grid to shade the cell according to the sampled value
                agent = SampledCell(self.goal, self.model, self.model.gaussian[self.path[self.path_step][0],
                                                                               self.path[self.path_step][1]])
                self.model.grid.place_agent(agent, self.goal)
                print("Robot", self.unique_id, "sampled value", agent.value, "at", self.goal)
                self.goal = []
                self.path = []
                self.path_step = 0

                # Perform kriging interpolation from sampled values
                # Define parameters (Explanations are in kriging.py)
                xgrid = np.arange(0, self.model.width, 1)
                ygrid = np.arange(0, self.model.height, 1)

                # print(np.shape(self.model.sampled))
                sampled_cells = np.where(np.array(self.model.sampled) != -1)
                sampled_cells = list(zip(sampled_cells[1], sampled_cells[0]))
                self.model.num_samples = len(sampled_cells)

                if len(sampled_cells) >= len(self.model.robots):
                    x_arr = []
                    y_arr = []
                    o_arr = []
                    for sampled_cell in sampled_cells:
                        # print(sampled_cell)
                        x_arr.append(sampled_cell[0])
                        y_arr.append(sampled_cell[1])
                        val = self.model.sampled[sampled_cell[1], sampled_cell[0]]
                        # print(val, "\n")
                        o_arr.append(val)

                    # print(x_arr)
                    # print(y_arr)
                    # print(o_arr)

                    variogram = 'gaussian'

                    # Run prediction
                    m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

                    self.model.RMSE = np.mean(np.power(np.array(self.model.gaussian) - m, 2))
                    self.model.avg_variance = np.mean(v)
                    # print("RMSE:", self.model.RMSE)

                    # Export as images
                    plt.figure('Mean')
                    plt.title("Values Predicted by Kriging Interpolation")
                    plt.imshow(m, origin="lower")
                    plt.colorbar()
                    plt.savefig(self.model.visualisation_dir+'Mean.png')
                    plt.close()
                    plt.figure('Variance')
                    plt.title("Kriging Variance")
                    plt.imshow(v, origin="lower")
                    plt.colorbar()
                    plt.savefig(self.model.visualisation_dir+'Variance.png')
                    plt.close()

                    # Sort indices of variance into ascending order
                    v_ind_sorted = np.unravel_index(np.argsort(v, axis=None), v.shape)

                    # Print indices of x highest variance cells (candidate goals)
                    # where x is the number of robots
                    self.model.candidate_goals = [[v_ind_sorted[1][-r],
                                                  v_ind_sorted[0][-r]] for r in range(1, len(self.model.robots) + 1)]
                    print("Candidate goals:", self.model.candidate_goals)

        # print("Robot", str(self.unique_id), "current cell:", str(self.pos))
        # print("Robot", str(self.unique_id), "previous cell:", str(self.trajectory[len(self.trajectory) - 2]))
        # print("Robot", str(self.unique_id), "trajectory length:", len(self.trajectory), "\n")
        if not self.goal:
            self.idle_time += 1
        elif self.pos == tuple(self.trajectory[len(self.trajectory) - 2]):
            self.waiting_time += 1
            print(self.waiting_time)

    # Assigns a goal to the robot to sample a value at the given goal position
    def sample_pos(self, goal_pos):
        self.goal = goal_pos
        print("Robot", self.unique_id, "assigned task at", self.goal, "at step", self.model.step_num)
        self.path = self.astar.run(self.pos, self.goal)
        self.path_step = 1


# A class to represent a sampled cell as a MESA agent to enable visualisation of the sampled value on the grid
class SampledCell(Agent):
    def __init__(self, pos, model, value=0):
        super().__init__(pos, model)
        self.pos = pos
        self.value = value
        self.color = "#%02x%02x%02x" % (int(value * 255), int(value * 255), int(value * 255))
        self.type = 1  # Agent type 1 is a sampled cell


class UnsampledCell(SampledCell):
    def __init__(self, pos, model):
        super().__init__(pos, model)
        self.pos = pos
        self.color = "BlanchedAlmond"
        self.type = 2  # Agent type 2 is an unsampled cell


class SpatialSamplingModel(Model):
    def __init__(self, height=20, width=20, num_robots=2):
        self.height = height
        self.width = width
        self.num_robots = num_robots
        self.robots = []
        self.RMSE = 999
        self.avg_variance = 999
        self.RMSEs = []
        self.variance_avgs = []
        self.num_samples = 0
        self.num_samples_col = []
        self.robot_travel_distances = {}
        self.robot_idle_times = {}
        self.robot_waiting_times = {}
        self.task_allocation = "RR"  # 'random' or 'RR' (Round Robin)
        self.trial_num = 10
        self.visualisation_dir = "./results/RRTA_3robs_20x20/"+str(self.trial_num)+"/"

        # Delete old figures and data
        # old_figures = glob.glob("visited_cells_vis/*")
        old_figures = glob.glob(self.visualisation_dir+"*")
        for f in old_figures:
            os.remove(f)

        self.data_collector = DataCollector(model_reporters={"RMSE": "RMSE"})

        # Agents are instantiated simultaneously
        self.schedule = SimultaneousActivation(self)
        # The grid is multi-layered, and does not loop at the edges
        self.grid = MultiGrid(width, height, torus=False)

        # An underlying 2D Gaussian distribution is created for the robots to sample values from
        # At present this can only be a square matrix, hence only using the height
        self.gaussian = makeGaussian(height)
        self.sampled = np.ones((width, height)) * -1
        self.visited = np.zeros((width, height))
        self.step_num = 0
        self.num_goals = 0
        self.candidate_goals = []
        self.all_cells_assigned = False

        # Place UnsampledCell agents for visualisation of unsampled cells
        for x in range(self.width):
            for y in range(self.height):
                self.grid.place_agent(UnsampledCell(x, y), (x, y))

        # Place robots in grid at random locations, continuously re-assigning these when the location already contains
        # another robot
        starting_positions = []
        for rob_id in range(self.num_robots):
            x = self.random.randrange(1, width)
            y = self.random.randrange(1, height)
            while [x, y] in starting_positions:
                x = self.random.randrange(1, width)
                y = self.random.randrange(1, height)
            starting_positions.append([x, y])
            agent = Robot((x, y), self)
            self.robots.append(agent)
            self.grid.place_agent(agent, (x, y))
            # Adds the robot to the scheduler, so that its step function will be called each time step
            self.schedule.add(agent)

        for agent in self.schedule.agents:
            if agent.type == 0:  # True if the agent is a robot
                self.robot_travel_distances[str(agent.unique_id)] = []
                self.robot_idle_times[str(agent.unique_id)] = []
                self.robot_waiting_times[str(agent.unique_id)] = []

        # Start running the simulation
        self.running = True

    def step(self):
        self.step_num += 1

        # Create costs in the neighbourhoods of other robots in each robot's movement cost map
        for agent in self.schedule.agents:
            if agent.type == 0:  # True if the agent is a robot
                current_agent_id = agent.unique_id
                current_agent_pos = agent.pos
                for other_agent in self.schedule.agents:
                    if other_agent.type == 0:  # True if the agent is a robot
                        if other_agent.unique_id != current_agent_id:
                            for cell in self.grid.iter_neighborhood((current_agent_pos[0], current_agent_pos[1]), False,
                                                                    True, radius=2):
                                other_agent.movement_matrix[cell[1], cell[0]] = 10

                if self.step_num == 1:
                    agent.sample_pos(agent.pos)
                elif not agent.goal:  # If the current robot in the iteration has no goal
                    # assign it one at an unsampled cell
                    goal_ind = 0

                    if len(self.candidate_goals) < 1 or self.task_allocation == "random":
                        goal_pos = (random.randrange(0, self.width), random.randrange(0, self.height))
                    else:
                        goal_pos = tuple(self.candidate_goals[goal_ind])

                    # Create a list of all other robots' goals to check for and reassign any duplicate goals
                    current_goal_cells = []
                    for a in self.schedule.agents:
                        if a.type == 0 and a.goal is not None and a.unique_id != agent.unique_id and a.goal != []:
                            current_goal_cells.append(a.goal)

                    while goal_pos in current_goal_cells:
                        goal_ind += 1
                        if len(self.candidate_goals) < 1 or self.task_allocation == "random":
                            goal_pos = (random.randrange(0, self.width), random.randrange(0, self.height))
                        else:
                            goal_pos = tuple(self.candidate_goals[goal_ind])

                        current_goal_cells = []
                        for a in self.schedule.agents:
                            if a.type == 0 and a.goal is not None and a.unique_id != agent.unique_id and a.goal != []:
                                current_goal_cells.append(a.goal)

                        # Finally assign the goal to the robot, after the goal has been validated, i.e. it is for an as
                        # yet unsampled cell, does not contain another robot, and is not already assigned to another
                        # robot
                    agent.sample_pos(goal_pos)
        print("")

        if self.RMSE < 999:
            self.RMSEs.append(self.RMSE)
        else:
            self.RMSEs.append(pd.NA)

        if self.avg_variance < 999:
            self.variance_avgs.append(self.avg_variance)
        else:
            self.variance_avgs.append(pd.NA)

        sampled_cells = np.where(np.array(self.sampled) != -1)
        sampled_cells = list(zip(sampled_cells[1], sampled_cells[0]))
        self.num_samples = len(sampled_cells)
        self.num_samples_col.append(self.num_samples)

        # Get total distance travelled by each robot
        for agent in self.schedule.agents:
            if agent.type == 0:  # True if the agent is a robot
                self.robot_travel_distances[str(agent.unique_id)].append(agent.distance_travelled)
                self.robot_idle_times[str(agent.unique_id)].append(agent.idle_time)
                self.robot_waiting_times[str(agent.unique_id)].append(agent.waiting_time)

        self.data_collector.collect(self)

        # Stop the simulation when all cells have been sampled by the robots
        # or Root Mean Square Error is below a given value
        # if -1 not in self.sampled or self.RMSE < 0.008:
        if self.num_samples >= 50:
            metrics = pd.DataFrame({
                "Time step": range(1, len(self.RMSEs) + 1),
                "RMSE": self.RMSEs,
                "Average variance": self.variance_avgs,
                "Number of cells sampled": self.num_samples_col
            })
            metrics.set_index("Time step")

            for robot in self.robot_travel_distances.keys():
                metrics["Robot " + robot + " distance travelled"] = self.robot_travel_distances[robot]
                metrics["Robot " + robot + " idle time"] = self.robot_idle_times[robot]
                metrics["Robot " + robot + " waiting time"] = self.robot_waiting_times[robot]

            print(metrics)
            metrics.to_csv("./results/random_sampling_3robs_20x20/"+str(self.trial_num)+"/"+str(self.trial_num)+".csv")

            # Export visited cells as images
            combined_cells_visited = np.zeros((self.width, self.height))
            trajectory_plot_info = {}
            for agent in self.schedule.agents:
                if agent.type == 0:  # True if the agent is a robot
                    ax = plt.figure("Robot " + str(agent.unique_id) + " visited cells").gca()
                    ax.yaxis.get_major_locator().set_params(integer=True)
                    ax.xaxis.get_major_locator().set_params(integer=True)
                    plt.imshow(agent.visited, origin="lower", cmap="gray")

                    trajectory_plot_info[agent.unique_id] = {
                        "x": [agent.trajectory[t][0] for t in range(len(agent.trajectory))],
                        "y": [agent.trajectory[t][1] for t in range(len(agent.trajectory))],
                        "c": agent.color
                    }

                    plt.plot(trajectory_plot_info[agent.unique_id]["x"],
                             trajectory_plot_info[agent.unique_id]["y"],
                             c=trajectory_plot_info[agent.unique_id]["c"])

                    plt.colorbar()
                    plt.title("Map of Cells Visited by Robot " + str(agent.unique_id))
                    # plt.savefig("visited_cells_vis/robot_" + str(agent.unique_id) + "_visited_cells.png")
                    plt.savefig(self.visualisation_dir + str(agent.unique_id) + "_visited_cells.png")
                    plt.close()
                    combined_cells_visited += agent.visited

            ax = plt.figure("Combined visited cells").gca()
            ax.yaxis.get_major_locator().set_params(integer=True)
            ax.xaxis.get_major_locator().set_params(integer=True)
            plt.imshow(combined_cells_visited, origin="lower", cmap="gray")
            for robot_id in trajectory_plot_info.keys():
                plt.plot(trajectory_plot_info[robot_id]["x"],
                         trajectory_plot_info[robot_id]["y"],
                         c=trajectory_plot_info[robot_id]["c"])
            plt.title("Combined Map of Visited Cells")
            plt.legend(["Robot "+str(robot_id) for robot_id in trajectory_plot_info.keys()])
            plt.colorbar()
            plt.savefig(self.visualisation_dir+"combined_visited_cells.png")
            plt.close()

            self.running = False

        # Run one step of the model
        self.schedule.step()
