# This script is used to animate the graphics output of the mesa_spatial_sampling_MRS script.
__author__ = "Laurence Roberts-Elliott"

import glob
import os
from animate_graphical_output import convert_to_gif


def animate_batch(batch_path, num_trials):
    for i in range(1, num_trials + 1):
        path_to_png_files = batch_path + str(i) + "/"

        print("Animating png files in: " + path_to_png_files)

        # Animate predicted values
        convert_to_gif(path_to_png_files,
                       path_to_png_files + 'mean_animation.gif', text_filter="Mean")

        # Animate kriging variance
        convert_to_gif(path_to_png_files,
                       path_to_png_files + 'variance_animation.gif', text_filter="Variance")

        # Animate combined visited cells and robot trajectories
        convert_to_gif(path_to_png_files,
                       path_to_png_files + 'combined_visited_cells_animation.gif',
                       text_filter="combined_visited_cells")

        # # Animate unsampled cell clustering
        # convert_to_gif(path_to_png_files,
        #                path_to_png_files + 'clustering_animation.gif', text_filter="clusters")

        # # Delete png files
        # os.system('rm ' + path_to_png_files + '*.png')


if __name__ == '__main__':
    animate_batch("results/SSI_RS_3robs_20x20/", 1)
