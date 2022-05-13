import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from kriging_utils.kriging import predict_by_kriging
import random

random.seed(2)
sample_ratio = 0.05

compaction_df = pd.read_csv("jaime_compaction_0cm_kpas.csv")
print(compaction_df)

# plt.scatter(x=compaction_df["lon"], y=compaction_df["lat"])
# plt.show()

print("Min. lat:", np.min(compaction_df["lat"]))
print("Max. lat:", np.max(compaction_df["lat"]))  # 220m
print("Min. lon:", np.min(compaction_df["lon"]))
print("Max. lon:", np.max(compaction_df["lon"]))  # 325m

# set environment res. to res. of sampling
# env_width = 17
# env_height = 13

env_width = 324  # m converted from lon
env_height = 220  # m converted from lat

# Scaling down environment resolution due to memory limitations
env_width /= 8
env_height /= 8
print("Env. width:", env_width)
print("Env. height:", env_height)

# env_width = int(env_width / 4)
# env_height = int(env_height / 4)


# Perform kriging interpolation from sampled values
# Define parameters (Explanations are in kriging.py)
lat_range = np.max(compaction_df["lat"]) - np.min(compaction_df["lat"])
lon_range = np.max(compaction_df["lon"]) - np.min(compaction_df["lon"])
lon_to_lat_ratio = np.max(compaction_df["lon"]) / np.max(compaction_df["lat"])
print("Lon to lat ratio:", lon_to_lat_ratio)
# xgrid = np.arange(np.min(compaction_df["lon"]), np.max(compaction_df["lon"]), lon_range/18)
# ygrid = np.arange(np.min(compaction_df["lat"]), np.max(compaction_df["lat"]), lat_range/13)
# xgrid = np.arange(np.min(compaction_df["lon"]), np.max(compaction_df["lon"]), lon_range/env_width)
# ygrid = np.arange(np.min(compaction_df["lat"]), np.max(compaction_df["lat"]), lat_range/env_height)
xgrid = np.arange(np.min(compaction_df["lon"]), np.max(compaction_df["lon"]), lon_range/env_width)
ygrid = np.arange(np.min(compaction_df["lat"]), np.max(compaction_df["lat"]), lat_range/env_height)


# Sample random points from the dataframe to test the kriging interpolation
random_indices = random.sample(range(len(compaction_df["0.0 cm"])), round(len(list(compaction_df["0.0 cm"])) *
                                                                          sample_ratio))
print("Number of random samples:", len(random_indices))

x_arr = np.array(compaction_df["lon"][random_indices])
y_arr = np.array(compaction_df["lat"][random_indices])
o_arr = np.array(compaction_df["0.0 cm"][random_indices])

plt.style.use('dark_background')
plt.scatter(x=x_arr, y=y_arr, c=o_arr, cmap="gray")
plt.colorbar()
plt.title("Ground truth compaction (kPas) data at 0.0 cm")
plt.savefig("dummy_data_points.png")
plt.show()
plt.close()
# for sampled_cell in sampled_cells:
#     # print(sampled_cell)
#     x_arr.append(sampled_cell[0])
#     y_arr.append(sampled_cell[1])
#     val = self.model.sampled[sampled_cell[1], sampled_cell[0]]
#     # print(val, "\n")
#     o_arr.append(val)
#
print(x_arr.shape)
print(y_arr.shape)
print(o_arr.shape)

print(x_arr[0])
print(y_arr[0])
print(o_arr[0])

variogram = 'gaussian'

# Run prediction
m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

print(np.shape(m))
plt.figure('Mean')
plt.title("Compaction (kPas) Predicted by Kriging Interpolation")
plt.imshow(m, origin="lower", cmap="gray")
plt.colorbar()
plt.savefig("kriging_test_means.png")
plt.show()
print(m)

print(np.shape(v))
plt.figure('Variance')
plt.title("Kriging Variance")
plt.imshow(v, origin="lower", cmap="gray")
plt.colorbar()
plt.savefig("kriging_test_variance.png")
plt.show()
print(m)
