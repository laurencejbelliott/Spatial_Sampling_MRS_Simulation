from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import CanvasGrid, ChartModule
from mesa.visualization.UserParam import UserSettableParameter
from model import SpatialSamplingModel
import numpy as np

import pickle


# Draws robots and sampled cells (both modelled as agents) in their respective shapes and layers of the grid
def draw(agent):
    if agent is None:
        return
    if agent.type == 0:
        portrayal = {"Shape": "circle", "r": 4, "Filled": "true", "Layer": 1, "Color": [agent.color, agent.color]}
    else:
        portrayal = {"Shape": "rect", "w": 1.0, "h": 1.0, "Filled": "true", "Layer": 0,
                     "Color": [agent.color, agent.color]}
    return portrayal


# Grid defined and instantiated
with open(r"interpolated_ADR.pickle", "rb") as f:
    interpolated_compaction_data = pickle.load(f)
# with open(r"interpolated_jaime_compaction_0cm_kpas.pickle", "rb") as f:
#     interpolated_compaction_data = pickle.load(f)


# width = np.shape(interpolated_compaction_data)[0]
# height = np.shape(interpolated_compaction_data)[1]
width = np.shape(interpolated_compaction_data)[1]
height = np.shape(interpolated_compaction_data)[0]

# canvas_element = CanvasGrid(draw, width, height, width*20, height*20)
canvas_width = width * 10
canvas_height = height * 10
canvas_element = CanvasGrid(draw, width, height, canvas_width, canvas_height)

RMSE_chart = ChartModule([{"Label": "RMSE",
                           "Color": "Black"}],
                         data_collector_name="data_collector",
                         canvas_height=200, canvas_width=canvas_width)

max_steps_slider = UserSettableParameter("slider", name="Maximum Time Steps for Sampling", value=240, min_value=1,
                                         max_value=240, step=1)

num_robots_slider = UserSettableParameter("slider", name="Number of Robots", value=3, min_value=2,
                                          max_value=20, step=1)

task_allocation_choice = UserSettableParameter("choice", "Task Allocation Algorithm",
                                               value="Sequential Single Item (SSI) auction",
                                               choices=["Sequential Single Item (SSI) auction", "Round Robin"])

sampling_choice = UserSettableParameter("choice", "Sampling Strategy",
                                        value="Dynamic",
                                        choices=["Dynamic", "Random"])

# Model parameterised and instantiated
model_params = {
    "max_steps": max_steps_slider,
    "height": height,
    "width": width,
    "num_robots": num_robots_slider,
    "vis_freq": 5,
    "task_allocation": task_allocation_choice,
    "sampling_strategy": sampling_choice
}

vis_elements = [
    canvas_element,
    RMSE_chart
]

server = ModularServer(
    SpatialSamplingModel, vis_elements, "Spatial Sampling MRS", model_params
)
