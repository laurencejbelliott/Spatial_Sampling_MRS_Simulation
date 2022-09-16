# This script is used to animate the graphics output of the mesa_spatial_sampling_MRS script.
__author__ = "Laurence Roberts-Elliott"
# Import modules
import os
import glob
import imageio


# Convert png images into single gif
def convert_to_gif(path_to_png_files, gif_name, fps=10, text_filter=None):
    """
    Converts png images into a single gif.
    :param path_to_png_files: Path to png files.
    :param gif_name: Name of gif file.
    :param fps: Frames per second.
    :return: None
    """

    # Get all png files in directory
    png_files = glob.glob(path_to_png_files + '*.png')

    # Sort png files
    png_files = sorted(png_files, key=lambda x: int(x.split('/')[3].split('_')[0]))
    # print(png_files)

    # Create gif
    images = []
    for file_name in png_files:
        if text_filter is None:
            images.append(imageio.imread(file_name))
        elif text_filter in file_name:
            # print(file_name)
            images.append(imageio.imread(file_name))
    imageio.mimsave(gif_name, images, fps=fps)


if __name__ == '__main__':
    path_to_png_files = "results/default/1/"

    png_files = glob.glob(path_to_png_files + '*.png')

    # Animate predicted values
    convert_to_gif(path_to_png_files,
                   path_to_png_files+'interpolation_animation.gif', text_filter="Interpolation")

    # Animate kriging variance
    convert_to_gif(path_to_png_files,
                   path_to_png_files+'variance_animation.gif', text_filter="Variance")

    # Animate combined visited cells and robot trajectories
    convert_to_gif(path_to_png_files,
                   path_to_png_files+'combined_visited_cells_animation.gif',
                   text_filter="combined_visited_cells")

    # # Animate unsampled cell clustering
    # convert_to_gif(path_to_png_files,
    #                 path_to_png_files+'clustering_animation.gif', text_filter="clusters")

    # # Animate robot id (4, 8)'s trajectories
    # convert_to_gif('results/SSI_RS_3robs_20x20/1/',
    #                'results/SSI_RS_3robs_20x20/1/(4, 8)_animation.gif',
    #                text_filter="(4, 8)")
    #
    # # Animate robot id (5, 10)'s trajectories
    # convert_to_gif('results/SSI_RS_3robs_20x20/1/',
    #                'results/SSI_RS_3robs_20x20/1/(5, 10)_animation.gif',
    #                text_filter="(5, 10)")
    #
    # # Animate robot id (16, 8)'s trajectories
    # convert_to_gif('results/SSI_RS_3robs_20x20/1/',
    #                'results/SSI_RS_3robs_20x20/1/(16, 8)_animation.gif',
    #                text_filter="(16, 8)")

    # # Delete png files
    # os.system('rm ' + path_to_png_files + '*.png')
