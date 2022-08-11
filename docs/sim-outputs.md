[Home](https://github.com/laurencejbelliott/Spatial_Sampling_MRS_Simulation/) | [Accessing Simulation Outputs](/docs/sim-outputs.md) | [Sampling Custom Data](/docs/custom-data.md) | [Automating Batches of Experiments](/docs/batch-experiments.md)

# Accessing Simulation Outputs
By default, simulation runs executed via `run.py` will save output data and additional graphical outputs in `\mesa_spatial_sampling_MRS\results\default\1\`.
Within this directory, `1.csv` contains data on the performance of the simulated multi-robot system recorded at each step of the simulated sampling mission.
This file and the directory it resides in are named `1` after the iteration number of the simulated run, used as a seed for
randomised variables such as the starting positions of the robots.

Visual outputs are also saved in this folder. The names of these are prefixed with the simulation time step at which they were generated.