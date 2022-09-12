import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from kriging_utils.kriging import predict_by_kriging
import pickle

compaction_df = pd.read_csv("jaime_compaction_0cm_kpas.csv")
print(compaction_df)

print("Min. lat:", np.min(compaction_df["lat"]))
print("Max. lat:", np.max(compaction_df["lat"]))  # 220m
print("Min. lon:", np.min(compaction_df["lon"]))
print("Max. lon:", np.max(compaction_df["lon"]))  # 324m

# Set environment dimensions
env_width = 324  # m converted from lon
env_height = 220  # m converted from lat

# Scaling down environment resolution due to memory limitations
env_width /= 2
env_height /= 2
print("Env. width:", env_width)
print("Env. height:", env_height)

lat_range = np.max(compaction_df["lat"]) - np.min(compaction_df["lat"])
lon_range = np.max(compaction_df["lon"]) - np.min(compaction_df["lon"])
lon_to_lat_ratio = np.max(compaction_df["lon"]) / np.max(compaction_df["lat"])
print("Lon to lat ratio:", lon_to_lat_ratio)

# Perform kriging interpolation from sampled values in dataset
# Define parameters (Explanations are in kriging.py)

# Set kriging grid dimensions to those the environment
xgrid = np.arange(np.min(compaction_df["lon"]), np.max(compaction_df["lon"]), lon_range/env_width)
ygrid = np.arange(np.min(compaction_df["lat"]), np.max(compaction_df["lat"]), lat_range/env_height)

# Define location and data of sampled points
x_arr = np.array(compaction_df["lon"])
y_arr = np.array(compaction_df["lat"])
o_arr = np.array(compaction_df["0.0 cm"])

# Plot sampled points to be compared later with interpolated values
plt.style.use('dark_background')
plt.scatter(x=x_arr, y=y_arr, c=o_arr, cmap="gray")
plt.colorbar()
plt.title("Ground truth compaction (kPas) data at 0.0 cm")
plt.savefig("ground_truth_compaction_0cm.png")
plt.show()
plt.close()

print(x_arr.shape)
print(y_arr.shape)
print(o_arr.shape)

print(x_arr[0])
print(y_arr[0])
print(o_arr[0])

# Use Gaussian variogram model
variogram = 'gaussian'

# Run interpolation of sampled points so that the ground truth data can be sampled at any point in the sim. environment
m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

# Generate heatmap of interpolated values to be compared with ground truth point data
print(np.shape(m))
plt.figure('Mean')
plt.title("Compaction (kPas) Predicted by Kriging Interpolation")
plt.imshow(m, origin="lower", cmap="gray")
plt.colorbar()
plt.savefig("interpolated_compaction_0cm.png")
plt.show()
print(m)

# Generate interpolated values with default colormap to be compared later with interpolation of simulated sampling
plt.style.use('default')
plt.figure('Mean')
plt.title("Compaction (kPas) Predicted by Kriging Interpolation")
plt.imshow(m, origin="lower")
plt.colorbar()
plt.savefig("interpolated_compaction_0cm_colour.png")
plt.show()

# Save interpolated values to be sampled in simulation
with open(r"interpolated_jaime_compaction_0cm_kpas.pickle", "wb") as output_file:
    pickle.dump(m, output_file)

