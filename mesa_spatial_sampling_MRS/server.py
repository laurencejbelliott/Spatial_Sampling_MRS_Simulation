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
    portrayal = {"Shape": "circle", "r": 0.5, "Filled": "true", "Layer": 0, "Color": [agent.color, agent.color]}
    return portrayal


# happy_element = HappyElement()
canvas_element = CanvasGrid(draw, 20, 20, 500, 500)
# happy_chart = ChartModule([{"Label": "happy", "Color": "Black"}])

model_params = {
    "height": 20,
    "width": 20,
    "num_robots": 5
}

server = ModularServer(
    SpatialSamplingModel, [canvas_element], "Spatial Sampling MRS", model_params
)
