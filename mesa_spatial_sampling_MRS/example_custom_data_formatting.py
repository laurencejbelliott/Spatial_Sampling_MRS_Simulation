import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from kriging_utils.kriging import predict_by_kriging
import pickle

custom_df = pd.read_csv("chernobyl_radionuclide_contamination_dataset/data/1_Spatial_dataset.csv")
custom_df = custom_df[custom_df["ADR"].notna()]

# Define bounds of subregion of the Chernobyl exclusion zone to sample in simulation
custom_df = custom_df[custom_df["Longitude"] <= 30.0986]  # WGS-84 degrees
custom_df = custom_df[custom_df["Longitude"] >= 30.0383]

custom_df = custom_df[custom_df["Latitude"] <= 51.4714]
custom_df = custom_df[custom_df["Latitude"] >= 51.4216]

print(custom_df)

print("Min. lat:", np.min(custom_df["Latitude"]))
print("Max. lat:", np.max(custom_df["Latitude"]))  # x width: ~6692m (from QGIS)
print("Min. lon:", np.min(custom_df["Longitude"]))
print("Max. lon:", np.max(custom_df["Longitude"]))  # y height: ~8894m (from QGIS)
#
# # Set environment dimensions
env_width = 6692  # m converted from lon
env_height = 8894  # m converted from lat
#
# Scaling down environment resolution due to memory limitations
env_width /= 100
env_height /= 100
print("Env. width:", env_width)
print("Env. height:", env_height)
#
lat_range = np.max(custom_df["Latitude"]) - np.min(custom_df["Latitude"])
lon_range = np.max(custom_df["Longitude"]) - np.min(custom_df["Longitude"])

# Perform kriging interpolation from sampled values in dataset
# Define parameters (Explanations are in kriging.py)

# Set kriging grid dimensions to those the environment
xgrid = np.arange(np.min(custom_df["Longitude"]), np.max(custom_df["Longitude"]), lon_range / env_width)
ygrid = np.arange(np.min(custom_df["Latitude"]), np.max(custom_df["Latitude"]), lat_range / env_height)

# Define location and data of sampled points
x_arr = np.array(custom_df["Longitude"])
y_arr = np.array(custom_df["Latitude"])
o_arr = np.array(custom_df["ADR"])

# Plot sampled points to be compared later with interpolated values
plt.style.use('dark_background')
plt.scatter(x=x_arr, y=y_arr, c=o_arr, cmap="gray")
plt.colorbar()
plt.title("Ground truth ADR (micro Gray per hour) data")
plt.savefig("ground_truth_adr.png")
plt.show()
plt.close()
#
# print(x_arr.shape)
# print(y_arr.shape)
# print(o_arr.shape)
#
# print(x_arr[0])
# print(y_arr[0])
# print(o_arr[0])
#
# Use Gaussian variogram model
variogram = 'gaussian'

# Run interpolation of sampled points so that the ground truth data can be sampled at any point in the sim. environment
m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

# Generate heatmap of interpolated values to be compared with ground truth point data
print(np.shape(m))
plt.figure('Mean')
plt.title("ADR (micro Gray per hour) Predicted by Kriging")
plt.imshow(m, origin="lower", cmap="gray")
plt.colorbar()
plt.savefig("interpolated_ADR.png")
plt.show()
print(m)

# Generate interpolated values with default colormap to be compared later with interpolation of simulated sampling
plt.style.use('default')
plt.figure('Mean')
plt.title("ADR (micro Gray per hour) Predicted by Kriging")
plt.imshow(m, origin="lower")
plt.colorbar()
plt.savefig("interpolated_ADR_colour.png")
plt.show()

# Save interpolated values to be sampled in simulation
with open(r"interpolated_ADR.pickle", "wb") as output_file:
    pickle.dump(m, output_file)

