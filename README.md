[Home](https://github.com/laurencejbelliott/Spatial_Sampling_MRS_Simulation/) | [Accessing Simulation Outputs](/docs/sim-outputs.md) | [Sampling Custom Data](/docs/custom-data.md) | [Automating Batches of Experiments](/docs/batch-experiments.md)
# Spatial Sampling Multi-Robot System Simulation
A 2D grid-based simulation of a spatial sampling Multi-Robot System (MRS).
Created using the MESA agent-based modelling framework for Python. 
This simulation is developed as a platform for simulated experiments to compare the
performance of different combinations of task allocation and sampling methods for 
multi-robot mapping of spatial data. Its abstract representation of a MRS
allows many experiments to be conducted quickly, with low computational complexity.
It also includes interactive visualisation with easy control of parameters.
By default, the simulated MRS samples surface soil compaction data, recorded using
an outdoor Thorvald robot at the University of Lincoln's Riseholme campus by 
Jaime Fentanes et al. Further details of their dataset and their work on robotic soil
compaction sampling can be found in ['3D Soil Compaction Mapping through Kriging-based Exploration with a Mobile Robot'](https://arxiv.org/abs/1803.08069).
This data is measured in Kpa, and is interpolated by this simulation using kriging,
so that samples can be taken from ground-truth data at arbitrary locations.

You can provide the simulation with your own geolocated data for the robots to sample, following the instructions
in [Sampling Custom Data](/docs/custom-data.md).

## Dependencies
The simulation runs entirely in Python 3, requiring the following packages to be 
installed with:
`pip install mesa numpy astar-python PyKrige matplotlib`

## Running the simulation
To run the simulation, use the bash command `mesa runserver` with your working directory
set to that of `mesa_spatial_sampling_MRS`.
If the `mesa` command is not found, and you have installed mesa in a virtual environment,
ensure that you have activated this environment before running this command.

Alternatively, you can run the Python script `run.py`.

If all has been set up correctly, running the simulation should open a web page in
your default internet browser for visualisation.
It should resemble this screenshot:

![A screenshot of the 2D grid visualisation of the multi-robot spatial sampling simulation.](./docs/images/spatial_MRS_sim_vis.png "A screenshot of the 2D grid visualisation of the spatial sampling MRS simulation.")

Instructions on using this grid visualisation web UI can be found in the [Grid Visualisation section of MESA's advanced tutorial](https://mesa.readthedocs.io/en/latest/tutorials/adv_tutorial.html#grid-visualization). 