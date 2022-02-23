import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from kriging_utils.kriging import predict_by_kriging
import pickle

compaction_df = pd.read_csv("jaime_compaction_0cm_kpas.csv")
print(compaction_df)

plt.scatter(x=compaction_df["lon"], y=compaction_df["lat"])
plt.show()


# Perform kriging interpolation from sampled values
# Define parameters (Explanations are in kriging.py)
lat_range = np.max(compaction_df["lat"]) - np.min(compaction_df["lat"])
lon_range = np.max(compaction_df["lon"]) - np.min(compaction_df["lon"])
lon_to_lat_ratio = np.max(compaction_df["lon"]) / np.max(compaction_df["lat"])
print("Lon to lat ratio:", lon_to_lat_ratio)
xgrid = np.arange(np.min(compaction_df["lon"]), np.max(compaction_df["lon"]), lat_range/20)
ygrid = np.arange(np.min(compaction_df["lat"]), np.max(compaction_df["lat"]), lon_range/20)

x_arr = np.array(compaction_df["lon"])
y_arr = np.array(compaction_df["lat"])
o_arr = np.array(compaction_df["0.0 cm"])
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
plt.title("Values Predicted by Kriging Interpolation")
plt.imshow(m, origin="lower")
plt.colorbar()
plt.show()
print(m)
with open(r"interpolated_jaime_compaction_0cm_kpas.pickle", "wb") as output_file:
    pickle.dump(m, output_file)

