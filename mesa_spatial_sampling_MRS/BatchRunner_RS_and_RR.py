__author__ = "Laurence Roberts-Elliott"
from model import *
from mesa.batchrunner import BatchRunnerMP

RSBatchRunner = BatchRunnerMP(SpatialSamplingModel,
                            fixed_parameters={
                                "width": 20, "height": 20, "task_allocation": "RR", "num_robots": 3,
                                "results_dir": "./results/RRTA_3robs_20x20/"},
                            variable_parameters={
                                "trial_num": range(1, 11, 1)},  # Define number of iterations for output in correct dirs
                            iterations=1,
                            max_steps=99999,
                            model_reporters={"Step number": SpatialSamplingModel.getNumSamples,
                                             "RMSE": SpatialSamplingModel.getRMSE,
                                             "Average Variance": SpatialSamplingModel.getAvgVariance,
                                             "Total cells sampled": SpatialSamplingModel.getNumSamples
                                             },
                            nr_processes=10
                            )

RSBatchRunner.run_all()
RSBatchData = RSBatchRunner.get_model_vars_dataframe()
print(RSBatchData)
RSBatchData.to_csv("results/random_sampling_3robs_20x20/results.csv")
