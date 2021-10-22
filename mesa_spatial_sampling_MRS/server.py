from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import CanvasGrid, ChartModule, TextElement
from mesa.visualization.UserParam import UserSettableParameter
from model import SpatialSamplingModel


# Draws robots and sampled cells (both modelled as agents) in their respective shapes and layers of the grid
def draw(agent):
    if agent is None:
        return
    if agent.type == 0:
        portrayal = {"Shape": "circle", "r": 0.5, "Filled": "true", "Layer": 1, "Color": [agent.color, agent.color]}
    else:
        portrayal = {"Shape": "rect", "w": 1.0, "h": 1.0, "Filled": "true", "Layer": 0,
                     "Color": [agent.color, agent.color]}
    return portrayal


# Grid defined and instantiated
width = 50
height = 50
canvas_element = CanvasGrid(draw, width, height, 500, 500)
RMSE_chart = ChartModule([{"Label": "RMSE",
                           "Color": "Black"}],
                         data_collector_name="data_collector",
                         canvas_height=500)

# Model parameterised and instantiated
model_params = {
    "height": height,
    "width": width,
    "num_robots": 3
}

server = ModularServer(
    SpatialSamplingModel, [canvas_element, RMSE_chart], "Spatial Sampling MRS", model_params
)
