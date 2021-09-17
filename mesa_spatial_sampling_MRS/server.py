from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.modules import CanvasGrid, ChartModule, TextElement
from mesa.visualization.UserParam import UserSettableParameter
from model import SpatialSamplingModel
import random


# class HappyElement(TextElement):
#     def __init__(self):
#         pass
#
#     def render(self, model):
#         return "Happy agents: " + str(model.happy)


def draw(agent):
    if agent is None:
        return
    # r = lambda: random.randint(0, 255)
    # rand_color = ("#%02X%02X%02X" % (r(), r(), r()))
    if agent.type == 0:
        portrayal = {"Shape": "circle", "r": 0.5, "Filled": "true", "Layer": 1, "Color": [agent.color, agent.color]}
    else:
        portrayal = {"Shape": "rect", "w": 1.0, "h": 1.0, "Filled": "true", "Layer": 0,
                     "Color": [agent.color, agent.color]}
    return portrayal


width = 10
height = 10
# happy_element = HappyElement()
canvas_element = CanvasGrid(draw, width, height, 500, 500)
# happy_chart = ChartModule([{"Label": "happy", "Color": "Black"}])

model_params = {
    "height": height,
    "width": width,
    "num_robots": 10
}

server = ModularServer(
    SpatialSamplingModel, [canvas_element], "Spatial Sampling MRS", model_params
)
