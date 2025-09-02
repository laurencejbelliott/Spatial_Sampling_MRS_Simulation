__author__ = "Loz Elliott"

import numpy as np
import pandas as pd
import pickle

max_variance = 0
min_variance = 9999 # Start with value higher than any expected variance
conditions = [
    'RR_DS_3robs_20x20',
    'RR_RS_3robs_20x20',
    'SSI_DS_3robs_20x20',
    'SSI_RS_3robs_20x20'
]
num_trials = 10

# Iterate through conditions and trials
for condition in conditions:
    for trial in range(1, num_trials + 1):
        # Get last time step from results.csv
        with open(f"results/{condition}/{trial}/{trial}.csv", "r") as f:
            df = pd.read_csv(f)
            last_time_step = int(df.iloc[-1]["Time step"])
            print(f"Condition: {condition}, Trial: {trial}, Last Time Step: {last_time_step}")

        # Calculate max variance from last step pickle in results/x/y/visualisation_data/s_variance.pickle
        with open(f"results/{condition}/{trial}/visualisation_data/{last_time_step}_variance.pickle", "rb") as f:
            data = pickle.load(f)

            print(data)
            print(type(data))
            current_trial_max_variance = np.max(data)
            current_trial_min_variance = np.min(data)
            if current_trial_max_variance > max_variance:
                max_variance = current_trial_max_variance
            if current_trial_min_variance < min_variance:
                min_variance = current_trial_min_variance
print("Min Variance: ", min_variance)
print("Max Variance: ", max_variance)
