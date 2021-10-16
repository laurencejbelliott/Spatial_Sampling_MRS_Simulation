__author__ = "Laurence Roberts-Elliott"
import os
import glob
import random

import mesa
from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import numpy as np
from astar_python import Astar
from gaussian import makeGaussian


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
                                # If the next cell is not deadlocked, simply move the robot there
                                else:
                                    self.model.grid.move_agent(self, tuple(self.path[self.path_step]))
                                    self.path_step = 1
                            else:
                                print("Cell", tuple(self.path[self.path_step]), "in robot", str(self.unique_id)+"'s", "path occupied")
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

            # If the robot has reached the end of its path, and thus its goal, sample a value from the underlying
            # distribution
            elif self.path_step == len(self.path) - 1:
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
        # Agents are instantiated simultaneously
        self.schedule = SimultaneousActivation(self)
        # The grid is multi-layered, and does not loop at the edges
        self.grid = MultiGrid(width, height, torus=False)

        # An underlying 2D Gaussian distribution is created for the robots to sample values from
        # At present this can only be a square matrix, hence only using the height
        self.gaussian = makeGaussian(height)
        self.sampled = np.ones((width, height)) * -1
        self.step_num = 0
        self.num_goals = 0
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

        # Start running the simulation
        self.running = True

    def step(self):
        self.step_num += 1
        for agent in self.schedule.agents:
            if agent.type == 0:  # True if the agent is a robot
                agent.movement_matrix = np.ones((self.width, self.height))

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

                # If the current robot in the iteration has no goal, assign it one at an unsampled cell
                if not agent.goal:
                    goal_pos = (random.randrange(0, self.width), random.randrange(0, self.height))
                    if goal_pos is not None:
                        # Create a list of all other robots' goals to check for and reassign any duplicate goals
                        current_goal_cells = []
                        for a in self.schedule.agents:
                            if a.type == 0 and a.goal is not None and a.unique_id != agent.unique_id and a.goal != []:
                                current_goal_cells.append(a.goal)
                        goal_val_loop_runs = 0

                        # Check goal cell for SampledCell agent (i.e. already sampled)
                        goal_cell_already_sampled = True
                        for a in self.grid.get_cell_list_contents(goal_pos):
                            if a.type == 2:
                                goal_cell_already_sampled = False

                        if goal_cell_already_sampled:
                            print("Goal cell already sampled")

                        # Ensure that the goal cell has not been sampled or assigned to another robot
                        while self.sampled[goal_pos[1], goal_pos[0]] != -1 or goal_pos in current_goal_cells or\
                                goal_cell_already_sampled:
                            goal_val_loop_runs += 1

                            # For debugging, informs the user if the goal validation loop has run an excessive number of
                            # times and therefore may be stuck. Can be useful to set a breakpoint here if the loop gets
                            # stuck
                            if goal_val_loop_runs > (self.width * self.height) * 10:
                                print("Consecutive goal validation loop runs: ", goal_val_loop_runs)
                                print("Loop stuck?")
                            if agent.goal in current_goal_cells:
                                print("Goal", agent.goal, "already assigned in", current_goal_cells)

                            goal_pos = (random.randrange(0, self.width), random.randrange(0, self.height))

                            num_unsampled_cells = np.count_nonzero(self.sampled == -1)
                            if num_unsampled_cells == 0:
                                break

                            # Check goal cell for SampledCell agent (i.e. already sampled)
                            goal_cell_already_sampled = True
                            for a in self.grid.get_cell_list_contents(goal_pos):
                                if a.type == 2:
                                    goal_cell_already_sampled = False
                            if goal_cell_already_sampled:
                                continue

                            # Prevents assignment of goals when all remaining unsampled cells have already been assigned
                            # as sampling goals to the robots
                            if num_unsampled_cells <= len(self.robots):
                                print(num_unsampled_cells, "unsampled cells remaining for", len(self.robots), "robots")

                                unsampled_cells = np.where(self.sampled == -1)
                                unsampled_cells = set(zip(unsampled_cells[1], unsampled_cells[0]))
                                print("Goal cells:", set(current_goal_cells), "len:", len(set(current_goal_cells)))
                                print("Unsampled cells:", unsampled_cells, "len:", len(unsampled_cells))
                                print("Goal cells are a subset of unsampled cells:",
                                      set(current_goal_cells).issubset(unsampled_cells))
                                if unsampled_cells == set(current_goal_cells):
                                    print("All unsampled cells assigned")

                                print("Halting assignment of additional goal cells to robot", agent.unique_id)
                                self.all_cells_assigned = True
                                break
                        # Finally assign the goal to the robot, after the goal has been validated, i.e. it is for an as
                        # yet unsampled cell, does not contain another robot, and is not already assigned to another
                        # robot
                        agent.sample_pos(goal_pos)
        print("")

        # Stop the simulation when all cells have been sampled by the robots
        if -1 not in self.sampled:
            self.running = False

        # Run one step of the model
        self.schedule.step()
