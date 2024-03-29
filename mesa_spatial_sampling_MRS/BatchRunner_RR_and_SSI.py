__author__ = "Laurence Roberts-Elliott"
from model import *
from mesa.batchrunner import BatchRunnerMP


def main():
    # Automated batch of trials for Round Robin wih random sampling, 3 robots, Jaime's soil compaction data at 0cm depth
    results_dir = "./results/SSI_RS_3robs_20x20/"
    BatchRunner = BatchRunnerMP(SpatialSamplingModel,
                                fixed_parameters={
                                    "width": 162, "height": 110,  # Dimensions of the interpolated soil compaction data
                                    "task_allocation": "Sequential Single Item (SSI) auction", "num_robots": 3,
                                    "sampling_strategy": "Random",
                                    "results_dir": results_dir,
                                    "vis_freq": 240
                                },
                                variable_parameters={
                                    "trial_num": range(1, 11, 1)},  # Iterate trial num. for seed & visual outputs
                                iterations=1,
                                max_steps=99999,
                                model_reporters={"Step number": SpatialSamplingModel.getStepNum,
                                                 "RMSE": SpatialSamplingModel.getRMSE,
                                                 "Average Variance": SpatialSamplingModel.getAvgVariance,
                                                 "Total cells sampled": SpatialSamplingModel.getNumSamples,
                                                 "Total distance travelled": SpatialSamplingModel.getTotalDistance,
                                                 "Total idle time": SpatialSamplingModel.getTotalIdleTime,
                                                 "Total waiting time": SpatialSamplingModel.getTotalWaitingTime,
                                                 "Maximum visits to a cell": SpatialSamplingModel.getMaxVisits,
                                                 "Average time to complete a task":
                                                     SpatialSamplingModel.getAvgTaskCompletionTime
                                                 },
                                nr_processes=1
                                )

    BatchRunner.run_all()
    BatchData = BatchRunner.get_model_vars_dataframe()
    print(BatchData)
    BatchData.to_csv(results_dir + "results.csv")

    results_dir = "./results/SSI_DS_3robs_20x20/"
    BatchRunner = BatchRunnerMP(SpatialSamplingModel,
                                fixed_parameters={
                                    "width": 162, "height": 110,
                                    "task_allocation": "Sequential Single Item (SSI) auction", "num_robots": 3,
                                    "sampling_strategy": "Dynamic",
                                    "results_dir": results_dir,
                                    "vis_freq": 240},
                                variable_parameters={
                                    "trial_num": range(1, 11, 1)},
                                iterations=1,
                                max_steps=99999,
                                model_reporters={"Step number": SpatialSamplingModel.getStepNum,
                                                 "RMSE": SpatialSamplingModel.getRMSE,
                                                 "Average Variance": SpatialSamplingModel.getAvgVariance,
                                                 "Total cells sampled": SpatialSamplingModel.getNumSamples,
                                                 "Total distance travelled": SpatialSamplingModel.getTotalDistance,
                                                 "Total idle time": SpatialSamplingModel.getTotalIdleTime,
                                                 "Total waiting time": SpatialSamplingModel.getTotalWaitingTime,
                                                 "Maximum visits to a cell": SpatialSamplingModel.getMaxVisits,
                                                 "Average time to complete a task":
                                                     SpatialSamplingModel.getAvgTaskCompletionTime
                                                 },
                                nr_processes=1
                                )

    BatchRunner.run_all()
    BatchData = BatchRunner.get_model_vars_dataframe()
    print(BatchData)
    BatchData.to_csv(results_dir+"results.csv")

    results_dir = "./results/RR_RS_3robs_20x20/"
    BatchRunner = BatchRunnerMP(SpatialSamplingModel,
                                fixed_parameters={
                                    "width": 162, "height": 110,
                                    "task_allocation": "Round Robin", "num_robots": 3,
                                    "sampling_strategy": "Random",
                                    "results_dir": results_dir,
                                    "vis_freq": 240},
                                variable_parameters={
                                    "trial_num": range(1, 11, 1)},
                                iterations=1,
                                max_steps=99999,
                                model_reporters={"Step number": SpatialSamplingModel.getStepNum,
                                                 "RMSE": SpatialSamplingModel.getRMSE,
                                                 "Average Variance": SpatialSamplingModel.getAvgVariance,
                                                 "Total cells sampled": SpatialSamplingModel.getNumSamples,
                                                 "Total distance travelled": SpatialSamplingModel.getTotalDistance,
                                                 "Total idle time": SpatialSamplingModel.getTotalIdleTime,
                                                 "Total waiting time": SpatialSamplingModel.getTotalWaitingTime,
                                                 "Maximum visits to a cell": SpatialSamplingModel.getMaxVisits,
                                                 "Average time to complete a task":
                                                     SpatialSamplingModel.getAvgTaskCompletionTime
                                                 },
                                nr_processes=1
                                )

    BatchRunner.run_all()
    BatchData = BatchRunner.get_model_vars_dataframe()
    print(BatchData)
    BatchData.to_csv(results_dir+"results.csv")

    results_dir = "./results/RR_DS_3robs_20x20/"
    BatchRunner = BatchRunnerMP(SpatialSamplingModel,
                                fixed_parameters={
                                    "width": 162, "height": 110,
                                    "task_allocation": "Round Robin", "num_robots": 3,
                                    "sampling_strategy": "Dynamic",
                                    "results_dir": results_dir,
                                    "vis_freq": 240},
                                variable_parameters={
                                    "trial_num": range(1, 11, 1)},
                                iterations=1,
                                max_steps=99999,
                                model_reporters={"Step number": SpatialSamplingModel.getStepNum,
                                                 "RMSE": SpatialSamplingModel.getRMSE,
                                                 "Average Variance": SpatialSamplingModel.getAvgVariance,
                                                 "Total cells sampled": SpatialSamplingModel.getNumSamples,
                                                 "Total distance travelled": SpatialSamplingModel.getTotalDistance,
                                                 "Total idle time": SpatialSamplingModel.getTotalIdleTime,
                                                 "Total waiting time": SpatialSamplingModel.getTotalWaitingTime,
                                                 "Maximum visits to a cell": SpatialSamplingModel.getMaxVisits,
                                                 "Average time to complete a task":
                                                     SpatialSamplingModel.getAvgTaskCompletionTime
                                                 },
                                nr_processes=1
                                )

    BatchRunner.run_all()
    BatchData = BatchRunner.get_model_vars_dataframe()
    print(BatchData)
    BatchData.to_csv(results_dir+"results.csv")


if __name__ == "__main__":
    main()
