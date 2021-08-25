__author__ = "Laurence Roberts-Elliott"
import glob
import ntpath

import matplotlib.pyplot as plt

old_figures = set(glob.glob('sim_visualisation/*'))
print(old_figures)

old_figures = [int(path[18:].split(".")[0]) for path in old_figures]
print(old_figures)

old_figures = sorted(old_figures)
print(old_figures)

old_figures = ["sim_visualisation/" + str(step) + ".png" for step in old_figures]
print(old_figures)

for f in old_figures:
    # plt.close('all')
    fig = plt.figure()
    image = plt.imread(f)
    plt.imshow(image)
    plt.draw()
    plt.pause(1)
    plt.close(fig)
