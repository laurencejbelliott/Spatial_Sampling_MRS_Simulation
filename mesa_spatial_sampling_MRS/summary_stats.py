__author__ = "Laurence Roberts-Elliott"
import pandas as pd
import numpy as np

RS_trial_results = []
total_distances_travelled = []
total_idle_times = []
total_waiting_times = []
for i in range(1, 11):
    print(i)
    trial_results = pd.read_csv(
        "results/random_sampling_3robs_20x20/"+str(i)+"/"+str(i)+".csv").iloc[-1]
    # Create totals from per robot metrics
    distances_travelled = [trial_results[5], trial_results[8], trial_results[11]]
    idle_times = [trial_results[6], trial_results[9], trial_results[12]]
    waiting_times = [trial_results[7], trial_results[10], trial_results[13]]
    total_distances_travelled.append(np.sum(distances_travelled))
    total_idle_times.append(np.sum(idle_times))
    total_waiting_times.append(np.sum(waiting_times))

    RS_trial_results.append(trial_results)

df_combined = pd.DataFrame(RS_trial_results)
df_combined["total distance travelled"] = total_distances_travelled
df_combined["total idle time"] = total_idle_times
df_combined["total waiting time"] = total_waiting_times

print(df_combined)
df_combined.to_csv("RS_combined.csv")
print("Mean of Random Sampling metrics:\n", df_combined.mean())
print("Median of Random Sampling metrics:\n", df_combined.median())


RR_trial_results = []
total_distances_travelled = []
total_idle_times = []
total_waiting_times = []
for i in range(1, 11):
    print(i)
    trial_results = pd.read_csv(
        "results/RRTA_3robs_20x20/"+str(i)+"/"+str(i)+".csv").iloc[-1]
    # Create totals from per robot metrics
    distances_travelled = [trial_results[5], trial_results[8], trial_results[11]]
    idle_times = [trial_results[6], trial_results[9], trial_results[12]]
    waiting_times = [trial_results[7], trial_results[10], trial_results[13]]
    total_distances_travelled.append(np.sum(distances_travelled))
    total_idle_times.append(np.sum(idle_times))
    total_waiting_times.append(np.sum(waiting_times))

    RR_trial_results.append(trial_results)

df_combined = pd.DataFrame(RR_trial_results)
df_combined["total distance travelled"] = total_distances_travelled
df_combined["total idle time"] = total_idle_times
df_combined["total waiting time"] = total_waiting_times

print(df_combined)
df_combined.to_csv("RS_combined.csv")
print("Mean of Random Sampling metrics:\n", df_combined.mean())
print("Median of Random Sampling metrics:\n", df_combined.median())
