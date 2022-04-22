# From https://stackoverflow.com/a/58610908

import imageio
import numpy as np

gifs_dir = "results/3robs_20x20_grid_sampling_all_cells/1/"

# Create reader object for the gif
gif1 = imageio.get_reader(gifs_dir+'combined_visited_cells_animation.gif')
gif2 = imageio.get_reader(gifs_dir+'mean_animation.gif')
gif3 = imageio.get_reader(gifs_dir+'variance_animation.gif')
gif4 = imageio.get_reader(gifs_dir+'clustering_animation.gif')

print("Gif 1 metadata: ", gif1.get_meta_data(), "\n", "Gif 2 metadata: ", gif2.get_meta_data(), "\n",
      "Gif 3 metadata: ", gif3.get_meta_data(), "\n", "Gif 4 metadata: ", gif4.get_meta_data())

# If they don't have the same number of frame take the shorter
number_of_frames = min(gif1.get_length(), gif2.get_length(), gif3.get_length(), gif4.get_length())

# Create writer object
new_gif = imageio.get_writer(gifs_dir+'visited_cells_and_variance.gif')

for frame_number in range(number_of_frames):
    img1 = gif1.get_next_data()
    img2 = gif2.get_next_data()
    img3 = gif3.get_next_data()
    img3 = np.squeeze(img3)

    img4 = gif4.get_next_data()
    # img4 = np.squeeze(img4)

    # Here is the magic
    new_image1 = np.hstack((img1, img2))
    print(img3.shape, img4.shape)
    new_image2 = np.hstack((img3, img4))
    new_image = np.vstack((new_image1, new_image2))
    new_gif.append_data(new_image)

gif1.close()
gif2.close()
new_gif.close()
