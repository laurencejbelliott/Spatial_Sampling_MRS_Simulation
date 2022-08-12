__author__ = "Laurence Roberts-Elliott"

import os
import glob
import random
import re
import copy
import pickle
import math
import shutil

import pandas as pd
from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import numpy as np
import matplotlib.pyplot as plt
from astar_python import Astar
from kriging_utils.kriging import predict_by_kriging
from scipy.cluster.hierarchy import fclusterdata


class Robot(Agent):
    def __init__(self, pos, model):
        # Inherits position and model attributes from Mesa's Agent superclass
        super().__init__(pos, model)
        self.pos = pos
        # The robot is assigned a randomly generated colour for visualisation
        r = lambda: self.model.random.randint(0, 255)

        # MESA represents colours in Hex, so we convert to this from RGB
        self.color = ("#%02X%02X%02X" % (r(), r(), r()))
        # Each robot has its own cost map for cost based avoidance of collision with other robots
        # self.movement_matrix = np.ones((self.model.width, self.model.height))
        self.movement_matrix = np.ones((self.model.height, self.model.width))
        # The Astar class from `astar-python` provides the A* implementation
        self.astar = Astar(self.movement_matrix)
        self.goal = []
        self.goals = []
        self.bid = -1
        self.path = []
        self.path_step = 0
        self.type = 0  # Agent type 0 is a robot
        self.previous_cell = None
        self.deadlocked = False
        self.distance_travelled = 0
        # Increase visits to cell at the robot's initial position by 1
        self.visited = np.zeros((self.model.width, self.model.height))
        self.visited[self.pos[0], self.pos[1]] += 1
        self.trajectory = [self.pos]
        self.idle_time = 0
        self.waiting_time = 0
        self.task_completion_times = []
        self.current_task_completion_time = 0
        self.sampled_cells_x = []
        self.sampled_cells_y = []

    # Each robot will execute this method at the start of a new time step in the simulation
    def step(self):
        self.deadlocked = False
        # If the robot has no goal, and only 1 goal in its queue, assign that as its current goal
        if not self.goal and len(self.goals) == 1:
            goal_pos = self.goals[0]
            self.goals.remove(goal_pos)
            self.sample_pos(goal_pos)
            self.model.allocated_tasks.add(tuple(goal_pos))
        elif not self.goal and len(self.goals) > 0:
            goal_pos = self.goals[0]
            self.goals.remove(goal_pos)
            self.sample_pos(goal_pos)

        # If the robot has a path to its goal
        if self.path is not None:
            # If the robot has reached its current goal position, sample a value at the goal position
            if list(self.pos) == list(self.goal):
                if len(self.path) == 1:
                    self.path_step = 0
                self.model.sampled[self.path[self.path_step][0], self.path[self.path_step][1]] = \
                    self.model.gaussian[self.path[self.path_step][1], self.path[self.path_step][0]]

                # Check goal cell for SampledCell agent (i.e. already sampled)
                sampled_cells_to_del = []
                for a in self.model.grid.get_cell_list_contents((int(self.path[self.path_step][0]),
                                                                 int(self.path[self.path_step][1]))):
                    if a.type == 2:
                        sampled_cells_to_del.append(a)
                for a in sampled_cells_to_del:
                    self.model.grid.remove_agent(a)

                # For the purpose of visualisation in MESA, the sampled cell is instantiated as an agent and placed on
                # the grid to shade the cell according to the sampled value
                agent = SampledCell(self.goal, self.model, self.model.gaussian[self.path[self.path_step][1],
                                                                               self.path[self.path_step][0]])
                self.model.grid.place_agent(agent, tuple(self.goal))
                if self.model.verbose:
                    print("Robot", self.unique_id, "sampled value", agent.value, "at", self.goal)
                self.goal = []
                self.path = []
                self.path_step = 0
                self.task_completion_times.append(self.current_task_completion_time)
                self.current_task_completion_time = 0
                self.sampled_cells_x.append(self.pos[0])
                self.sampled_cells_y.append(self.pos[1])

                # Perform kriging interpolation from sampled values
                # Define parameters (Explanations of these are in kriging_utils/kriging.py)
                xgrid = np.arange(0, self.model.width, 1)
                ygrid = np.arange(0, self.model.height, 1)

                print(np.shape(self.model.sampled))

                sampled_cells = np.where(np.array(self.model.sampled) != -1)
                sampled_cells = list(zip(sampled_cells[1], sampled_cells[0]))
                self.model.num_samples = len(sampled_cells)

                if len(sampled_cells) >= len(self.model.robots):
                    x_arr = []
                    y_arr = []
                    o_arr = []
                    for sampled_cell in sampled_cells:
                        x_arr.append(sampled_cell[1])
                        y_arr.append(sampled_cell[0])
                        val = self.model.sampled[sampled_cell[1], sampled_cell[0]]
                        o_arr.append(val)

                    # After a set number of samples, cross-validation of interpolation is performed to determine the
                    # best variogram model
                    if self.model.num_samples == 20:
                        variogram_models = ["linear", "power", "gaussian", "spherical"]
                        model_scores = []

                        for variogram_model in variogram_models:
                            m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram_model)

                            model_score = np.sqrt(np.mean(np.power(np.array(self.model.gaussian) - m,
                                                                   2)))

                            model_scores.append(model_score)
                            print(variogram_model, model_score)

                        best_model = variogram_models[np.argmin(model_scores)]
                        self.model.variogram = best_model
                        print("Best variogram model:", best_model)

                    # Run prediction
                    self.model.m, self.model.v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr,
                                                                    variogram=self.model.variogram)

                    self.model.RMSE = np.sqrt(np.mean(np.power(np.array(self.model.gaussian) - self.model.m, 2)))
                    self.model.avg_variance = np.mean(self.model.v)

                    if self.model.sampling_strategy == "Dynamic":

                        # Cluster unsampled cells
                        # Get unsampled cells and format as N by M matrix (N observations, M dimensions)
                        self.model.unsampled_cells = np.where(np.array(self.model.sampled) >= -1)
                        # self.model.unsampled_cells = np.where(np.array(self.model.sampled) >= -1)
                        self.model.unsampled_cells = np.array(list(zip(self.model.unsampled_cells[1],
                                                                       self.model.unsampled_cells[0])))

                        if self.model.step_num == 1:
                            self.model.unsampled_clusters = fclusterdata(self.model.unsampled_cells,
                                                              t=math.sqrt(self.model.width*self.model.height)/4,
                                                              criterion='distance',
                                                              metric='euclidean',
                                                              depth=1,
                                                              method='complete')

                        # Split variance cells array into sub-arrays based on cluster
                        self.model.v_clustered = [{} for cluster in range(1, len(set(self.model.unsampled_clusters)) + 1)]
                        for cell_ix in range(0, len(self.model.unsampled_cells)):
                            cell = self.model.unsampled_cells[cell_ix]
                            cell_cluster = self.model.unsampled_clusters[cell_ix]

                            cell_variance = self.model.v[cell[0], cell[1]]
                            self.model.v_clustered[cell_cluster - 1][str(cell)] = cell_variance

                        self.model.candidate_goals = []
                        print("Number of clusters:", len(self.model.v_clustered))
                        print("Number of cells:", self.model.width * self.model.height)

                        if self.model.verbose:
                            print("Allocated cluster indices:", self.model.clusters_sampled_ix)

                        # Clusters which have had tasks allocated within them are eliminated from task generation.
                        # The number of goals generated from each round of interpolation is limited to 2 times the
                        # number of robots, creating a task in the x clusters with the highest mean kriging
                        # variance, where x is 2 times the number of robots.

                        # Get mean kriging variance for each cluster
                        cluster_count = 0
                        cluster_mean_variances = {}
                        for cluster in self.model.v_clustered:
                            cluster_count += 1
                            cluster_size = len(cluster)
                            cluster_total_variance = 0
                            for cell in cluster.keys():
                                cluster_total_variance += cluster[cell]

                            cluster_mean_variance = cluster_total_variance / cluster_size
                            if self.model.verbose:
                                print("Mean kriging variance in cluster", cluster_count, ":", cluster_mean_variance)
                            cluster_mean_variances[cluster_count] = cluster_mean_variance

                        # Sort clusters by mean kriging variance
                        cluster_ids_by_variance = sorted(cluster_mean_variances, key=cluster_mean_variances.get,
                                                         reverse=True)

                        # Print cluster IDs sorted by mean variance, and their mean variance values
                        if self.model.verbose:
                            print("Cluster IDs sorted by kriging variance (highest first):")
                            for cluster_id in cluster_ids_by_variance:
                                print(cluster_id, cluster_mean_variances[cluster_id])

                        # Filter out IDs of clusters which have had a task allocated within them
                        cluster_ids_by_variance = [cluster_id for cluster_id in cluster_ids_by_variance if cluster_id
                                                   not in self.model.clusters_sampled_ix]
                        print("Cluster IDs by variance, excluding already allocated clusters:", cluster_ids_by_variance)
                        print("Number of clusters excluding allocated clusters:", len(cluster_ids_by_variance))
                        clusters_for_task_generation = cluster_ids_by_variance[:2*len(self.model.robots)]
                        print("Clusters for task generation:", clusters_for_task_generation)

                        cluster_count = 0
                        for cluster in self.model.v_clustered:
                            cluster_sampled = False
                            cluster_count += 1
                            if cluster_count in self.model.clusters_sampled_ix or \
                                    cluster_count not in clusters_for_task_generation:
                                continue

                            cluster_total_variance = 0
                            for cell in cluster.keys():
                                cluster_total_variance += cluster[cell]

                            for cell in cluster.keys():
                                cell = [re.sub("[^0-9]", "", word) for word in cell.split()]
                                for word in cell:
                                    if not word.isdigit():
                                        cell.remove(word)

                                cell = [int(word) for word in cell]
                                cell = [cell[1], cell[0]]
                                if tuple(cell) in self.model.allocated_tasks:
                                    cluster_sampled = True
                                    self.model.clusters_sampled_ix.add(cluster_count)
                                    break

                            if cluster_sampled:
                                continue
                            if not cluster_sampled:
                                cluster_max_v_cell_str = max(cluster, key=cluster.get)
                                cluster_max_v_cell_str = [re.sub("[^0-9]", "", word) for
                                                          word in cluster_max_v_cell_str.split()]
                                for word in cluster_max_v_cell_str:
                                    if not word.isdigit():
                                        cluster_max_v_cell_str.remove(word)

                                cluster_max_v_cell = [int(word) for word in cluster_max_v_cell_str]
                                cluster_max_v_cell = [cluster_max_v_cell[1], cluster_max_v_cell[0]]
                                cluster_sampled = True
                                self.model.clusters_sampled_ix.add(cluster_count)
                                self.model.candidate_goals.append(cluster_max_v_cell)
                                continue

                        if self.model.verbose:
                            print("No. of new task goals:", len(self.model.candidate_goals))

                    elif self.model.sampling_strategy == "Random":
                        # Set goals as x random unsampled cells (candidate goals)
                        # where x is the number of robots
                        self.model.candidate_goals = []
                        if self.model.all_cells_assigned:
                            pass
                        else:
                            for x in range(len(self.model.robots)):
                                goal_pos = (random.randrange(0, self.model.width),
                                            random.randrange(0, self.model.height))
                                while goal_pos in self.model.allocated_tasks:
                                    unallocated_cells = [[x, y] for x in range(self.model.width) for
                                                         y in range(self.model.height) if
                                                         [x, y] not in self.model.allocated_tasks]
                                    # print("Unallocated cells:", unallocated_cells)
                                    if unallocated_cells:
                                        goal_pos = random.choice(unallocated_cells)
                                    else:
                                        self.model.all_cells_assigned = True
                                        break
                                if not self.model.all_cells_assigned:
                                    self.model.candidate_goals.append(goal_pos)

                    if self.model.verbose:
                        print("Candidate goals:", self.model.candidate_goals)

                    if self.model.task_allocation == "Sequential Single Item (SSI) auction":
                        self.model.SSI_TA()
                    elif self.model.task_allocation == "Round Robin":
                        self.model.RR_TA()

            # If the robot hasn't reached its next sampling goal
            elif 0 < self.path_step < len(self.path):
                # Comment the following line to disable cost based collision avoidance. Updates the robot's cost map
                self.astar = Astar(self.movement_matrix)
                self.path = self.astar.run(self.pos, self.goal)
                self.current_task_completion_time += 1

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
                                    if self.model.verbose:
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
                                    if self.model.verbose:
                                        print("Moving robot", self.unique_id, "from", self.pos, "to", neighbour_cell)
                                    self.model.grid.move_agent(self, neighbour_cell)
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[neighbour_cell[0], neighbour_cell[1]] += 1
                                    self.trajectory.append(neighbour_cell)

                                # If the next cell is not deadlocked, simply move the robot there
                                else:
                                    self.model.grid.move_agent(self, tuple(self.path[self.path_step]))
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[self.path[self.path_step][0], self.path[self.path_step][1]] += 1
                                    self.trajectory.append(self.path[self.path_step])
                            else:
                                if self.model.verbose:
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
                                if self.model.verbose:
                                    print("Free neighbours:", free_neighbours)
                                if len(free_neighbours) > 0:
                                    free_cell = free_neighbours[random.randrange(len(free_neighbours))]

                                    if self.model.verbose:
                                        print("Moving from", self.pos, "to", free_cell)
                                    self.model.grid.move_agent(self, free_cell)
                                    self.path_step = 1
                                    self.distance_travelled += 1
                                    self.visited[free_cell[0], free_cell[1]] += 1
                                    self.trajectory.append(free_cell)

        elif len(self.goals) > 0:
            self.path = self.astar.run(self.pos, self.goal)
            print(self.goals)

        if not self.goal:
            self.idle_time += 1
        elif self.pos == tuple(self.trajectory[len(self.trajectory) - 2]):
            self.waiting_time += 1
            if self.model.verbose:
                print(self.waiting_time)

    # Assigns a goal to the robot to sample a value at the given goal position
    def sample_pos(self, goal_pos):
        self.goal = goal_pos
        if self.model.verbose:
            print("Robot", self.unique_id, "assigned task at", self.goal, "at step", self.model.step_num)
        # self.path = self.astar.run(np.array(self.pos) - [1, 1], np.array(self.goal) - [1, 1])
        self.path = self.astar.run(np.array(self.pos), np.array(self.goal))

        self.path_step = 1
        self.current_task_completion_time = 0

    def queue_sampling(self, goal_pos):
        # Check if goal_pos is in a cluster that has already been allocated for sampling
        goal_pos_in_allocated_cluster = False
        for cluster_ix in range(len(self.model.v_clustered)):
            if str(goal_pos) in list(self.model.v_clustered[cluster_ix].keys()):
                goal_pos_in_allocated_cluster = True
                break
        if goal_pos not in list(self.model.allocated_tasks) and goal_pos not in self.goals and \
                not goal_pos_in_allocated_cluster:
            if self.model.verbose:
                print("Robot", self.unique_id, "added task at", goal_pos, " to queue at step", self.model.step_num)
            # # Add goal to queue with value equal to path cost

            if len(self.goals) > 0:
                # Insertion heuristic
                # (insert task where it least increases the cost of navigating to the queued tasks)
                best_queue_cost = -1
                best_queue = []
                for i in range(len(self.goals)+1):
                    try:
                        candidate_goal_queue = copy.copy(self.goals)
                        candidate_goal_queue.insert(i, goal_pos)

                        queue_cost = 0
                        for j in range(len(candidate_goal_queue)):
                            if j > 0:
                                prev_goal = candidate_goal_queue[j-1]
                                queue_cost += np.sum([self.movement_matrix[step[1], step[0]] for
                                                      step in self.astar.run(prev_goal, candidate_goal_queue[j])])
                            else:
                                queue_cost += np.sum([self.movement_matrix[step[1], step[0]] for step in self.astar.run(
                                    self.pos, goal_pos)])
                        if best_queue_cost == -1 or best_queue_cost > queue_cost:
                            best_queue_cost = queue_cost
                            best_queue = candidate_goal_queue
                    except TypeError:
                        pass
                self.goals = best_queue
            else:
                # Add goal to queue
                self.goals.append(goal_pos)

            self.model.allocated_tasks.add(tuple(goal_pos))
            if self.model.verbose:
                print("Allocated tasks:", self.model.allocated_tasks)


# A class to represent a sampled cell as a MESA agent to enable visualisation of the sampled value on the grid
class SampledCell(Agent):
    def __init__(self, pos, model, value=0):
        super().__init__(pos, model)
        self.pos = pos
        self.value = value
        # Normalise value to 0-1 range for grayscale representation of value
        self.color = "#%02x%02x%02x" % (int(value * 255), int(value * 255), int(value * 255))
        if isinstance(self.model, SpatialSamplingModel):
            val_normalised = (self.value - self.model.min_sample_value) /\
                             (self.model.max_sample_value - self.model.min_sample_value)
            self.color = "#%02x%02x%02x" % (int(val_normalised * 255), int(val_normalised * 255), int(val_normalised * 255))
        self.type = 1  # Agent type 1 is a sampled cell


class UnsampledCell(SampledCell):
    def __init__(self, pos, model):
        super().__init__(pos, model)
        self.pos = pos
        self.color = "BlanchedAlmond"
        self.type = 2  # Agent type 2 is an unsampled cell


class SpatialSamplingModel(Model):
    def __init__(self, height=20, width=20, num_robots=2, task_allocation="Sequential Single Item (SSI) auction",
                 trial_num=1, max_steps=100,
                 sampling_strategy="Dynamic",
                 results_dir="./results/default/",
                 verbose=True, vis_freq=1):
        super(SpatialSamplingModel, self).__init__(seed=trial_num)
        self.random.seed(trial_num)
        random.seed(trial_num)
        self.step_num = 0
        with open(r"interpolated_jaime_compaction_0cm_kpas.pickle", "rb") as input_file:
            self.gaussian = np.array(pickle.load(input_file))

        # Delete old files and figures
        self.visualisation_dir = results_dir+str(trial_num)+"/"
        old_figures = glob.glob(self.visualisation_dir+"*")
        for f in old_figures:
            try:
                os.remove(f)
            except IsADirectoryError:
                shutil.rmtree(f)

        # Make visualisation data folder if one doesn't exist in current directory
        self.vis_data_dir = self.visualisation_dir+"/visualisation_data/"
        if not os.path.exists(self.vis_data_dir):
            os.makedirs(self.vis_data_dir)

        plt.imshow(self.gaussian)
        plt.xlabel("Cell X co-ordinate")
        plt.ylabel("Cell Y co-ordinate")
        pickle.dump(self.gaussian, open(self.vis_data_dir + "ground_truth_distribution.pickle", "wb"))
        plt.title("Underlying soil compaction data")
        plt.colorbar()
        plt.savefig("ground_truth_distribution.png", dpi=300)
        plt.close()
        self.min_sample_value = np.min(self.gaussian)
        self.max_sample_value = np.max(self.gaussian)

        self.width = width
        self.height = height

        self.max_steps = max_steps

        self.variogram = "gaussian"
        self.num_robots = num_robots
        self.robots = []
        self.RMSE = -1
        self.avg_variance = -1
        self.RMSEs = []
        self.variance_avgs = []
        self.num_samples = 0
        self.num_samples_col = []
        self.robot_travel_distances = {}
        self.robot_idle_times = {}
        self.robot_waiting_times = {}
        self.task_allocation = task_allocation  # "Round Robin" or "Sequential Single Item (SSI) auction"
        self.sampling_strategy = sampling_strategy  # "Dynamic" or "Random"
        self.allocated_tasks = set()
        self.RR_rob_index = 0
        self.RR_task_index = 0
        self.trial_num = trial_num
        self.combined_cells_visited = np.zeros((self.width, self.height))
        self.clusters_sampled = False
        self.verbose = verbose
        self.vis_freq = vis_freq
        self.v_clustered = []
        self.clusters_sampled_ix = set()

        self.data_collector = DataCollector(model_reporters={"RMSE": "RMSE"})

        # Agents are instantiated simultaneously
        self.schedule = SimultaneousActivation(self)
        # The grid is multi-layered, and does not loop at the edges
        self.grid = MultiGrid(self.width, self.height, torus=False)

        # An underlying 2D Gaussian distribution is created for the robots to sample values from
        # At present this can only be a square matrix, hence only using the height
        # self.gaussian = makeGaussian(height)
        self.sampled = np.ones((self.width, self.height)) * -1
        self.visited = np.zeros((self.width, self.height))
        self.num_goals = 0
        self.candidate_goals = []
        self.all_cells_assigned = False

        # Placeholders for kriging means and variance
        self.m = np.zeros((self.height, self.width))
        self.v = np.zeros((self.height, self.width))

        # Placeholders for unsampled cells and clusters
        self.unsampled_cells = []
        self.unsampled_clusters = []

        # Place UnsampledCell agents for visualisation of unsampled cells
        for x in range(self.width):
            for y in range(self.height):
                self.grid.place_agent(UnsampledCell(x, y), (x, y))

        # Place robots in grid at random locations, continuously re-assigning these when the location already contains
        # another robot
        starting_positions = []
        for rob_id in range(self.num_robots):
            x = self.random.randrange(1, self.width)
            y = self.random.randrange(1, self.height)
            while [x, y] in starting_positions:
                x = self.random.randrange(1, self.width)
                y = self.random.randrange(1, self.height)
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
                                other_agent.movement_matrix[cell[1], cell[0]] = 2  # 10

                if self.step_num == 1:
                    agent.queue_sampling(agent.pos)
        if self.verbose:
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

        if self.step_num % self.vis_freq == 0:
            print("Saving visualisations for step {}".format(self.step_num))
            # Export visited cells as images
            self.combined_cells_visited = np.zeros((self.width, self.height))
            trajectory_plot_info = {}
            for agent in self.schedule.agents:
                if agent.type == 0:  # True if the agent is a robot
                    ax = plt.figure("Robot " + str(agent.unique_id) + " visited cells").gca()
                    ax.yaxis.get_major_locator().set_params(integer=True)
                    ax.xaxis.get_major_locator().set_params(integer=True)
                    plt.imshow(np.swapaxes(agent.visited, 0, 1), origin="lower", cmap="gray")
                    plt.xlabel("Cell X co-ordinate")
                    plt.ylabel("Cell Y co-ordinate")

                    trajectory_plot_info[agent.unique_id] = {
                        "x": [agent.trajectory[t][0] for t in range(len(agent.trajectory))],
                        "y": [agent.trajectory[t][1] for t in range(len(agent.trajectory))],
                        "c": agent.color
                    }

                    # Dump agent's trajectory to pickle file
                    with open(self.vis_data_dir + str(self.step_num) + "_" + "trajectory_" + str(agent.unique_id) +
                              ".pickle", "wb") as f:
                        pickle.dump(trajectory_plot_info, f)

                    plt.plot(trajectory_plot_info[agent.unique_id]["x"],
                             trajectory_plot_info[agent.unique_id]["y"],
                             c=trajectory_plot_info[agent.unique_id]["c"],
                             linewidth=0.6)

                    plt.colorbar()
                    plt.scatter(x=agent.sampled_cells_x, y=agent.sampled_cells_y, marker="o",
                                c=trajectory_plot_info[agent.unique_id]["c"], zorder=1)

                    plt.title(
                        "Map of Cells Visited by Robot " + str(agent.unique_id) + " at Step " + str(self.step_num))
                    plt.xlabel("Cell X co-ordinate")
                    plt.ylabel("Cell Y co-ordinate")
                    plt.savefig(
                        self.visualisation_dir + str(self.step_num) + "_" + str(agent.unique_id) + "_visited_cells.png",
                        dpi=300)
                    plt.close()

                    # Dump agent's visited cells to pickle file
                    with open(self.vis_data_dir + str(self.step_num) + "_" + "visited_cells_" + str(agent.unique_id) + ".pickle", "wb") as f:
                        pickle.dump(agent.visited, f)

                    self.combined_cells_visited += agent.visited

            # Dump combined visited cells to pickle file
            with open(self.vis_data_dir + str(self.step_num) + "_" + "combined_cells_visited.pickle", "wb") as f:
                pickle.dump(self.combined_cells_visited, f)

            ax = plt.figure("Combined visited cells").gca()
            ax.yaxis.get_major_locator().set_params(integer=True)
            ax.xaxis.get_major_locator().set_params(integer=True)

            plt.imshow(np.swapaxes(self.combined_cells_visited, 0, 1), origin="lower", cmap="gray")
            plt.colorbar(ticks=np.linspace(0, np.max(self.combined_cells_visited), dtype=int))
            plt.xlabel("Cell X co-ordinate")
            plt.ylabel("Cell Y co-ordinate")
            for robot_id in trajectory_plot_info.keys():
                plt.plot(trajectory_plot_info[robot_id]["x"],
                         trajectory_plot_info[robot_id]["y"],
                         c=trajectory_plot_info[robot_id]["c"],
                         linewidth=0.6)
            for agent in self.schedule.agents:
                if agent.type == 0 and len(agent.sampled_cells_x) > 0:
                    plt.scatter(x=agent.sampled_cells_x[1:-1], y=agent.sampled_cells_y[1:-1], marker="o",
                                c=trajectory_plot_info[agent.unique_id]["c"])

                    plt.scatter(x=[agent.sampled_cells_x[0]], y=[agent.sampled_cells_y[0]], marker="s",
                                c=trajectory_plot_info[agent.unique_id]["c"], zorder=10)

                    plt.scatter(x=[agent.sampled_cells_x[-1]], y=[agent.sampled_cells_y[-1]], marker="X",
                                c=trajectory_plot_info[agent.unique_id]["c"], zorder=10)

                plt.title("Combined Map of Visited Cells  at Step " + str(self.step_num))
                plt.xlabel("Cell X co-ordinate")
                plt.ylabel("Cell Y co-ordinate")
            plt.legend(["Robot " + str(i) for i in range(1, len(self.robots) + 1)])
            plt.savefig(self.visualisation_dir + str(self.step_num) + "_" + "combined_visited_cells.png",
                        dpi=300)
            plt.close()

            # Export as images
            plt.figure('Mean')
            plt.title("Values Predicted by Kriging Interpolation at Step " + str(self.step_num))
            # Get ground truth min and mix
            gt_min = np.min(self.gaussian)
            gt_max = np.max(self.gaussian)

            # Normalise predicted values in range of ground truth min and max for visualisation
            plt.imshow(self.m, origin="lower", vmin=gt_min, vmax=gt_max)
            plt.xlabel("Cell X co-ordinate")
            plt.ylabel("Cell Y co-ordinate")
            plt.colorbar()
            plt.savefig(self.visualisation_dir + str(self.step_num) + "_" + 'Interpolation',
                        dpi=300)
            plt.close()

            # Dump predicted values to pickle file
            with open(self.vis_data_dir + str(self.step_num) + "_" + "mean.pickle", "wb") as f:
                pickle.dump(self.m, f)

            plt.figure('Variance')
            plt.title("Kriging Variance at Step " + str(self.step_num))
            plt.imshow(self.v, origin="lower", vmin=1, vmax=600)
            plt.xlabel("Cell X co-ordinate")
            plt.ylabel("Cell Y co-ordinate")
            plt.colorbar()
            plt.savefig(self.visualisation_dir + str(self.step_num) + "_" + 'Variance.png',
                        dpi=300)
            plt.close()

            # Dump predicted values to pickle file
            with open(self.vis_data_dir + str(self.step_num) + "_" + "variance.pickle", "wb") as f:
                pickle.dump(self.v, f)

        if self.sampling_strategy == "Dynamic":
            if len(self.unsampled_cells) > 0 and len(self.unsampled_clusters) > 0:
                if self.step_num == 2:
                    # Plot clusters of unsampled cells
                    plt.scatter(self.unsampled_cells[:, 1], self.unsampled_cells[:, 0],
                                c=self.unsampled_clusters)
                    plt.xlabel("Cell X co-ordinate")
                    plt.ylabel("Cell Y co-ordinate")
                    # plt.title("Unsampled Cells Clustered by Distance at Step " + str(self.step_num))
                    plt.title("Cells Clustered by Distance")
                    plt.savefig("cell_clusters.png", dpi=300)
                    plt.close()

                    # Dump unsampled cells to pickle file
                    with open(self.vis_data_dir + str(self.step_num) + "_" + "unsampled_cells.pickle", "wb") as f:
                        pickle.dump(self.unsampled_cells, f)

                    # Dump unsampled cell clusters to pickle file
                    with open(self.vis_data_dir + str(self.step_num) + "_" + "unsampled_cell_clusters.pickle", "wb") as f:
                        pickle.dump(self.unsampled_clusters, f)

        # Stop the simulation when all cells have been sampled by the robots
        # or Root Mean Square Error is below a given value
        if self.step_num >= self.max_steps or -1 not in self.sampled or self.clusters_sampled:
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
            metrics.to_csv(self.visualisation_dir+str(self.trial_num)+".csv")

            self.running = False

        # Run one step of the model
        self.schedule.step()

    def SSI_TA(self):
        for task in self.candidate_goals:
            if task not in list(self.allocated_tasks):
                if self.verbose:
                    print("Bidding on task at", task)
                bids = {}
                for agent in self.schedule.agents:
                    if agent.type == 0:  # True if the agent is a robot
                        # Insertion heuristic
                        # (insert task where it least increases the cost of navigating to the queued tasks)
                        best_queue_cost = -1
                        for i in range(len(agent.goals) + 1):
                            candidate_goal_queue = copy.copy(agent.goals)
                            candidate_goal_queue.insert(i, task)

                            queue_cost = 0
                            for j in range(len(candidate_goal_queue)):
                                if j > 0:
                                    prev_goal = candidate_goal_queue[j - 1]
                                    queue_cost += np.sum([agent.movement_matrix[step[1], step[0]] for
                                                          step in
                                                          agent.astar.run(prev_goal, candidate_goal_queue[j])])
                                else:
                                    queue_cost += np.sum(
                                        [agent.movement_matrix[step[1], step[0]] for step in agent.astar.run(
                                            agent.pos, task)])
                            if best_queue_cost == -1 or best_queue_cost > queue_cost:
                                best_queue_cost = queue_cost
                                bids[str(agent.unique_id)] = best_queue_cost
                winning_agent = min(bids, key=bids.get)

                for agent in self.schedule.agents:
                    if agent.type == 0:  # True if the agent is a robot
                        if str(agent.unique_id) == winning_agent:
                            if self.verbose:
                                print("Winning agent:", str(agent.unique_id))
                            agent.queue_sampling(task)

    def RR_TA(self):
        for task in self.candidate_goals:
            self.robots[self.RR_rob_index].queue_sampling(task)
            if self.RR_rob_index == len(self.robots) - 1:
                self.RR_rob_index = 0
            else:
                self.RR_rob_index += 1

    def getRMSE(self):
        return self.RMSE

    def getAvgVariance(self):
        return self.avg_variance

    def getNumSamples(self):
        return self.num_samples

    def getStepNum(self):
        return self.step_num

    def getTotalDistance(self):
        current_robot_distances = []
        for distances in self.robot_travel_distances.values():
            current_robot_distances.append(distances[-1])

        return np.sum(current_robot_distances)

    def getTotalIdleTime(self):
        current_robot_idle_times = []
        for idle_times in self.robot_idle_times.values():
            current_robot_idle_times.append(idle_times[-1])

        return np.sum(current_robot_idle_times)

    def getTotalWaitingTime(self):
        current_robot_waiting_times = []
        for waiting_times in self.robot_waiting_times.values():
            current_robot_waiting_times.append(waiting_times[-1])

        return np.sum(current_robot_waiting_times)

    def getMaxVisits(self):
        return np.max(self.combined_cells_visited)

    def getTotalTaskCompletionTime(self):
        task_completion_times = []
        for agent in self.schedule.agents:
            if agent.type == 0:
                task_completion_times.append(np.sum(agent.task_completion_times))

        return np.sum(task_completion_times)

    def getAvgTaskCompletionTime(self):
        task_completion_times = []
        for agent in self.schedule.agents:
            if agent.type == 0:
                task_completion_times.append(np.mean(agent.task_completion_times))

        return np.mean(task_completion_times)
