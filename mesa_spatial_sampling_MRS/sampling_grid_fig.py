__author__ = "Loz Elliott"

import pickle
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, floor

# Draw sampling grid for square 2D Gaussian mapping environment
grid_height = 20
grid_width = 20

sampling_cell_size_denominator = 6
sampling_grid_cell_size = floor(sqrt((grid_height * grid_width)) / sampling_cell_size_denominator)
print(sampling_grid_cell_size)

# Create figure showing environment grid and sampling grid
fig, ax = plt.subplots()
ax.set_xticks(np.arange(0, grid_width + 1, 1))
ax.set_yticks(np.arange(0, grid_height + 1, 1))
ax.set_xticks(np.arange(0.5, grid_width, 1), minor=True)
ax.set_yticks(np.arange(0.5, grid_height, 1), minor=True)
for i in range(0, grid_width, sampling_grid_cell_size):
   ax.axvline(i, color='r', linestyle='--', linewidth=1)
for j in range(0, grid_height, sampling_grid_cell_size):
   ax.axhline(j, color='r', linestyle='--', linewidth=1)
ax.set_xlim(0, grid_width)
ax.set_ylim(0, grid_height)

plt.gca().set_aspect('equal', adjustable='box')
plt.title('Environment with Sampling Grid (red)')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.show()

# Save figure
fig.savefig('sampling_grid_gaussian.png', dpi=300)



# Draw sampling grid for soil compaction mapping environment
# Get the shape of the soil compaction data from the pickle file
 

with open('mesa_spatial_sampling_MRS/interpolated_jaime_compaction_0cm_kpas.pickle', 'rb') as f:
    soil_compaction_data = pickle.load(f)

grid_height = soil_compaction_data.shape[0] # meters
grid_width = soil_compaction_data.shape[1] # meters

sampling_cell_size_denominator = 8
sampling_grid_cell_size = floor(sqrt((grid_height * grid_width)) / sampling_cell_size_denominator)
print(sampling_grid_cell_size)

# Create figure showing environment grid and sampling grid
fig, ax = plt.subplots()
ax.set_xticks(np.arange(0, grid_width + 1, 10))
ax.set_yticks(np.arange(0, grid_height + 1, 10))
ax.set_xticks(np.arange(0.5, grid_width, 1), minor=True)
ax.set_yticks(np.arange(0.5, grid_height, 1), minor=True)
for i in range(0, grid_width, sampling_grid_cell_size):
   ax.axvline(i, color='r', linestyle='--', linewidth=1)
for j in range(0, grid_height, sampling_grid_cell_size):
   ax.axhline(j, color='r', linestyle='--', linewidth=1)
ax.set_xlim(0, grid_width)
ax.set_ylim(0, grid_height)

plt.gca().set_aspect('equal', adjustable='box')
plt.title('Environment with Sampling Grid (red)')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.show()

# Save figure
fig.savefig('sampling_grid_soil_compaction.png', dpi=300)