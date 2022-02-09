__author__ = "Laurence Roberts-Elliott"
import pandas as pd

# Calculate summary statistics for random sampling trials
RS_df = pd.read_csv("results/random_sampling_3robs_20x20/results.csv")

RS_df_mean = RS_df.mean()
print("Mean of Random Sampling metrics:\n", RS_df_mean)
RS_df_mean.to_csv("results/random_sampling_3robs_20x20/mean.csv")

RS_df_med = RS_df.median()
print("Median of Random Sampling metrics:\n", RS_df_med)
RS_df_med.to_csv("results/random_sampling_3robs_20x20/median.csv")

RS_df_std = RS_df.std()
print("STD of Random Sampling metrics:\n", RS_df_std)
RS_df_std.to_csv("results/random_sampling_3robs_20x20/std.csv")


# Calculate summary statistics for Round Robin trials
RR_df = pd.read_csv("results/RRTA_3robs_20x20/results.csv")

RR_df_mean = RR_df.mean()
print("Mean of Round Robin metrics:\n", RR_df_mean)
RR_df_mean.to_csv("results/RRTA_3robs_20x20/mean.csv")

RR_df_med = RR_df.median()
print("Median of Round Robin metrics:\n", RR_df_med)
RR_df_med.to_csv("results/RRTA_3robs_20x20/median.csv")

RR_df_std = RR_df.std()
print("STD of Round Robin metrics:\n", RR_df_std)
RR_df_std.to_csv("results/RRTA_3robs_20x20/std.csv")