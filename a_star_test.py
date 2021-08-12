__author__ = "Laurence Roberts-Elliott"
import numpy as np
from astar_python import Astar

movement_matrix = np.ones((10, 10))
# print(movement_matrix)

a_star = Astar(movement_matrix)
result = a_star.run([0, 0], [5, 5])

for step in result:
    bot_map = np.zeros(movement_matrix.shape)
    bot_map[step[0], step[1]] = 1
    print(bot_map)
