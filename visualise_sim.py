__author__ = "Laurence Roberts-Elliott"
import glob
import matplotlib.pyplot as plt

old_figures = set(glob.glob('sim_visualisation/*'))

old_figures = sorted(old_figures, key=lambda item: (int(item.partition(' ')[0])
                                                    if item[0].isdigit() else float('inf'), item))
print(old_figures)
for f in old_figures:
    # plt.close('all')
    fig = plt.figure()
    image = plt.imread(f)
    plt.imshow(image)
    plt.draw()
    plt.pause(1)
    plt.close(fig)
