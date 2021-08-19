__author__ = "Laurence Roberts-Elliott"
import glob
import ntpath

import matplotlib.pyplot as plt

old_figures = set(glob.glob('sim_visualisation/*'))
print(old_figures)
old_figures = [float(".".join(path[:-4].split("_")[2:])) for path in old_figures]
print(old_figures)

old_figures = sorted(old_figures)
print(old_figures)

old_figures = ["sim_visualisation/t_"+str(path).replace(".", "_")+".png" for path in old_figures]
print(old_figures)

for f in old_figures:
    # plt.close('all')
    fig = plt.figure()
    image = plt.imread(f)
    plt.imshow(image)
    plt.draw()
    plt.pause(1)
    plt.close(fig)
