__author__ = "Laurence Roberts-Elliott"
from mesa import Model, Agent
from mesa.time import SimultaneousActivation
from mesa.space import SingleGrid
from mesa.datacollection import DataCollector
import numpy as np
from astar_python import Astar
from gaussian import makeGaussian


class Robot(Agent):
    # 1 Initialisation
    def __init__(self, pos, model):
        super().__init__(pos, model)
        self.pos = pos
        r = lambda: self.model.random.randint(0, 255)
        self.color = ("#%02X%02X%02X" % (r(), r(), r()))
        self.goal = []
        self.path = []
        self.path_step = 0

    # Step function
    def step(self):
        if 0 < self.path_step < len(self.path) - 1:
            # Commented line enables rudimentary collision avoidance (TODO: Currently bugged)
            # self.model.astar = Astar(self.model.movement_matrix)
            self.path = self.model.astar.run(self.pos, self.goal)
            if self.model.grid.is_cell_empty(tuple(self.path[self.path_step])):
                self.model.grid.move_agent(self, tuple(self.path[self.path_step]))
                self.path_step = 1

        elif self.path_step == len(self.path) - 1:
            self.goal = []
            self.path = []
            self.path_step = 0

    def sample_pos(self, goal_pos):
        self.goal = goal_pos
        self.path = self.model.astar.run(self.pos, self.goal)
        self.path_step = 1


class SpatialSamplingModel(Model):
    def __init__(self, height=20, width=20, num_robots=2):
        self.height = height
        self.width = width
        self.num_robots = num_robots

        self.schedule = SimultaneousActivation(self)
        self.grid = SingleGrid(width, height, torus=False)
        self.gaussian = makeGaussian(height)  # TODO: Add sampling and visualisation of Gaussian distribution
        self.sampled = np.zeros((width, height))
        self.movement_matrix = np.ones((width, height))
        self.astar = Astar(self.movement_matrix)
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
            self.grid.position_agent(agent, (x, y))
            self.schedule.add(agent)

        self.running = True
        # self.datacollector.collect(self)

    def step(self):
        self.movement_matrix = np.ones((self.width, self.height))
        for robot in self.schedule.agents:
            for cell in self.grid.iter_neighborhood((robot.pos[0], robot.pos[1]), False, True, 1):
                self.movement_matrix[cell[0], cell[1]] = 99
            if not robot.goal:
                robot.sample_pos(self.grid.find_empty())

        # Run one step of the model. If all agents are happy, halt the model.
        self.schedule.step()

        # Collect data
        # self.datacollector.collect(self)

        # if self.happy == self.schedule.get_agent_count():
        #     self.running = False
