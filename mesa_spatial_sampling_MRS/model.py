__author__ = "Laurence Roberts-Elliott"
# TODO: Document dependencies in README
import os
import glob
import random

from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import numpy as np
from astar_python import Astar
from gaussian import makeGaussian
from matplotlib import pyplot as plt

# TODO: Comment code where necessary for readability


class Robot(Agent):
    # 1 Initialisation
    def __init__(self, pos, model):
        super().__init__(pos, model)
        self.pos = pos
        r = lambda: self.model.random.randint(0, 255)
        self.color = ("#%02X%02X%02X" % (r(), r(), r()))
        self.movement_matrix = np.ones((self.model.width, self.model.height))
        self.astar = Astar(self.movement_matrix)
        self.goal = []
        self.path = []
        self.path_step = 0
        self.type = 0  # Agent type 0 is a robot
        self.previous_cell = None
        self.deadlocked = False

    # Step function
    def step(self):
        self.deadlocked = False
        if self.path is not None:
            if 0 < self.path_step < len(self.path) - 1:
                # TODO: Detect deadlock (two agents have the same next cell in their path) and stop one robot
                # Comment the following line to disable cost based collision avoidance
                self.astar = Astar(self.movement_matrix)
                self.path = self.astar.run(self.pos, self.goal)

                # Check if next cell in robot path contains another robot
                if self.path is not None:
                    if self.path[self.path_step] is not None:
                        cell_contains_robot = False
                        for agent in self.model.grid.get_cell_list_contents(tuple(self.path[self.path_step])):
                            if agent.type == 0:
                                cell_contains_robot = True

                        deadlock_cells = []
                        if self.model.grid.is_cell_empty(tuple(self.path[self.path_step])) or not cell_contains_robot:
                            for agent in self.model.schedule.agents:
                                if agent.type == 0:  # True if the agent is a robot
                                    if agent.path is not None and len(agent.path) > 0:
                                        if list(self.pos) == agent.path[agent.path_step]:
                                            self.deadlocked = True
                                            deadlock_cells.append(tuple(agent.path[agent.path_step]))
                                        # try:
                                        #     if list(self.pos) == agent.path[agent.path_step]:
                                        #         self.deadlocked = True
                                        #         deadlock_cells.append(tuple(agent.path[agent.path_step]))
                                        # except IndexError as e:
                                        #     print(e)
                                        #     # pass

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
                                print("Moving from", self.pos, "to", neighbour_cell)
                                self.model.grid.move_agent(self, neighbour_cell)
                                self.path_step = 1
                            else:
                                self.model.grid.move_agent(self, tuple(self.path[self.path_step]))
                                self.path_step = 1
                        else:
                            print("Cell in path occupied")
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
                            free_cell = free_neighbours[random.randrange(len(free_neighbours))]

                            # while not neighbour_free:
                            #     neighbour_cell_i = random.randrange(len(neighbours))
                            #     neighbour_cell = neighbours[neighbour_cell_i]
                            #     robot_in_cell = False
                            #     for agent in self.model.grid.get_cell_list_contents(neighbour_cell):
                            #         if agent.type == 0:  # If agent is a robot
                            #             robot_in_cell = True
                            #     if not robot_in_cell:
                            #         if neighbour_cell not in deadlock_cells:
                            #             neighbour_free = True

                            print("Moving from", self.pos, "to", free_cell)
                            self.model.grid.move_agent(self, free_cell)
                            self.path_step = 1

            elif self.path_step == len(self.path) - 1:
                self.model.sampled[self.path[self.path_step][1], self.path[self.path_step][0]] = \
                    self.model.gaussian[self.path[self.path_step][0], self.path[self.path_step][1]]
                agent = SampledCell(self.goal, self.model, self.model.gaussian[self.path[self.path_step][0],
                                                                               self.path[self.path_step][1]])
                self.model.grid.place_agent(agent, self.goal)
                print("Robot", self.unique_id, "sampled value", agent.value, "at", self.goal)
                self.goal = []
                self.path = []
                self.path_step = 0

    def sample_pos(self, goal_pos):
        self.goal = goal_pos
        print("Robot", self.unique_id, "assigned task at", self.goal, "at step", self.model.step_num)
        self.path = self.astar.run(self.pos, self.goal)
        self.path_step = 1


class SampledCell(Agent):
    # 1 Initialisation
    def __init__(self, pos, model, value=0):
        super().__init__(pos, model)
        self.pos = pos
        self.value = value
        self.color = "#%02x%02x%02x" % (int(value * 255), int(value * 255), int(value * 255))
        self.type = 1  # Agent type 1 is a sampled cell


class SpatialSamplingModel(Model):
    def __init__(self, height=20, width=20, num_robots=2):
        self.height = height
        self.width = width
        self.num_robots = num_robots
        self.robots = []
        self.figure_dir = "../sim_visualisation/"
        self.schedule = SimultaneousActivation(self)
        self.grid = MultiGrid(width, height, torus=False)
        self.gaussian = makeGaussian(height)
        self.sampled = np.ones((width, height)) * -1
        self.step_num = 0
        self.num_goals = 0

        old_figures = glob.glob(self.figure_dir + '*')
        for f in old_figures:
            os.remove(f)
        # Measured in m/s, max speed of Leo rover
        # self.robot_speed = 0.4

        # self.datacollector = DataCollector(
        #     # {"happy": "happy"},  # Model-level count of happy agents
        #     # For testing purposes an agent's individual x and y
        #     {"x": lambda a: a.pos[0], "y": lambda a: a.pos[1]}
        # )

        # Set up agents
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
            self.schedule.add(agent)

        self.running = True
        # self.datacollector.collect(self)

    def draw_map(self):
        plt.close('all')
        fig = plt.figure()
        ax = fig.gca()
        ax.set_xticks(np.arange(0, self.width, 1))
        ax.set_yticks(np.arange(0, self.height, 1))
        plt.xlim([0, self.width])
        plt.ylim([0, self.height])
        plt.imshow(self.sampled, cmap="gray", interpolation="nearest",
                   vmin=0, vmax=1)

        for robot in self.robots:
            plt.scatter(robot.pos[0], robot.pos[1], c=[robot.color])
        plt.grid()
        plt.title("Step " + str(self.schedule.time))
        # plt.show()
        # plt.savefig(self.figure_dir + "t_" + str(float(env.now)).replace(".", "_"))
        plt.savefig(self.figure_dir + str(self.schedule.time))

    # TODO: Fix robots being assigned a goal in a cell that is already assigned to another robot
    def step(self):
        self.step_num += 1
        for agent in self.schedule.agents:
            if agent.type == 0:  # True if the agent is a robot
                agent.movement_matrix = np.ones((self.width, self.height))

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
                if not agent.goal:
                    goal_pos = self.grid.find_empty()
                    if goal_pos is not None:
                        while self.sampled[goal_pos[1], goal_pos[0]] != -1:
                            goal_pos = self.grid.find_empty()
                        agent.sample_pos(goal_pos)
                print("Robot", str(agent.unique_id)+"'s path:", agent.path)
        print("")

        if -1 not in self.sampled:
            self.running = False

        # Run one step of the model.
        self.schedule.step()

        # Collect data
        # self.datacollector.collect(self)

        # if self.happy == self.schedule.get_agent_count():
        #     self.running = False
