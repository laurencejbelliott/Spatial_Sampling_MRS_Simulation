__author__ = "Laurence Roberts-Elliott"
import pandas as pd

# Calculate summary statistics for RR DS trials
df = pd.read_csv("results/RR_DS_3robs_20x20/results.csv")

df_mean = df.mean()
print("Mean of RR DS metrics:\n", df_mean)
df_mean.to_csv("results/RR_DS_3robs_20x20/mean.csv")

df_med = df.median()
print("Median of RR DS metrics:\n", df_med)
df_med.to_csv("results/RR_DS_3robs_20x20/median.csv")

df_std = df.std()
print("STD of RR DS metrics:\n", df_std)
df_std.to_csv("results/RR_DS_3robs_20x20/std.csv")

df_max = df.max()
print("Max of RR DS metrics:\n", df_max)
df_max.to_csv("results/RR_DS_3robs_20x20/max.csv")


# Calculate summary statistics for RR RS trials
df = pd.read_csv("results/RR_RS_3robs_20x20/results.csv")

df_mean = df.mean()
print("Mean of RR RS metrics:\n", df_mean)
df_mean.to_csv("results/RR_RS_3robs_20x20/mean.csv")

df_med = df.median()
print("Median of RR RS metrics:\n", df_med)
df_med.to_csv("results/RR_RS_3robs_20x20/median.csv")

df_std = df.std()
print("STD of RR RS metrics:\n", df_std)
df_std.to_csv("results/RR_RS_3robs_20x20/std.csv")

df_max = df.max()
print("Max of RR RS metrics:\n", df_max)
df_max.to_csv("results/RR_RS_3robs_20x20/max.csv")


# Calculate summary statistics for SSI DS trials
df = pd.read_csv("results/SSI_DS_3robs_20x20/results.csv")

df_mean = df.mean()
print("Mean of SSI DS metrics:\n", df_mean)
df_mean.to_csv("results/SSI_DS_3robs_20x20/mean.csv")

df_med = df.median()
print("Median of SSI DS metrics:\n", df_med)
df_med.to_csv("results/SSI_DS_3robs_20x20/median.csv")

df_std = df.std()
print("STD of SSI DS metrics:\n", df_std)
df_std.to_csv("results/SSI_DS_3robs_20x20/std.csv")

df_max = df.max()
print("Max of SSI DS metrics:\n", df_max)
df_max.to_csv("results/SSI_DS_3robs_20x20/max.csv")

# Calculate summary statistics for SSI RS trials
df = pd.read_csv("results/SSI_RS_3robs_20x20/results.csv")

df_mean = df.mean()
print("Mean of SSI RS metrics:\n", df_mean)
df_mean.to_csv("results/SSI_RS_3robs_20x20/mean.csv")

df_med = df.median()
print("Median of SSI RS metrics:\n", df_med)
df_med.to_csv("results/SSI_RS_3robs_20x20/median.csv")

df_std = df.std()
print("STD of SSI RS metrics:\n", df_std)
df_std.to_csv("results/SSI_RS_3robs_20x20/std.csv")

df_max = df.max()
print("Max of SSI RS metrics:\n", df_max)
df_max.to_csv("results/SSI_RS_3robs_20x20/max.csv")

