__author__ = "Laurence Roberts-Elliott"

import pandas as pd
import numpy as np

# Calculate max. RMSE from RR_DS trials
RR_DS_RMSEs = []
for i in range(1, 11):
    trial_results = pd.read_csv(
        "results/RR_DS_3robs_20x20/"+str(i)+"/"+str(i)+".csv")
    for j in range(len(trial_results)):
        
        RR_DS_RMSEs.append(trial_results.iloc[j]["RMSE"])
print("\nRR DS max. RMSE:", np.max(RR_DS_RMSEs))


# Calculate max. RMSE from RR_RS trials
RR_RS_RMSEs = []
for i in range(1, 11):
    trial_results = pd.read_csv(
        "results/RR_RS_3robs_20x20/"+str(i)+"/"+str(i)+".csv")
    for j in range(len(trial_results)):
        
        RR_RS_RMSEs.append(trial_results.iloc[j]["RMSE"])
print("\nRR RS max. RMSE:", np.max(RR_RS_RMSEs))


# Calculate max. RMSE from SSI_DS trials
SSI_DS_RMSEs = []
for i in range(1, 11):
    trial_results = pd.read_csv(
        "results/SSI_DS_3robs_20x20/"+str(i)+"/"+str(i)+".csv")
    for j in range(len(trial_results)):
        
        SSI_DS_RMSEs.append(trial_results.iloc[j]["RMSE"])
print("\nSSI DS max. RMSE:", np.max(SSI_DS_RMSEs))


# Calculate max. RMSE from SSI_RS trials
SSI_RS_RMSEs = []
for i in range(1, 11):
    trial_results = pd.read_csv(
        "results/SSI_RS_3robs_20x20/"+str(i)+"/"+str(i)+".csv")
    for j in range(len(trial_results)):
        SSI_RS_RMSEs.append(trial_results.iloc[j]["RMSE"])
print("\nSSI RS max. RMSE:", np.max(SSI_RS_RMSEs))

# Highest RMSE was reported as 23.412 for all conditions (same trial no. seed, RMSE from first round of interpolation
# from same initial positions across conditions), so skyrocketing RMSE issue seems to be resolved.
