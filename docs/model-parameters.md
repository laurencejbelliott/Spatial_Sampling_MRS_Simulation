[Home](https://github.com/laurencejbelliott/Spatial_Sampling_MRS_Simulation/) | [Accessing Simulation Outputs](/docs/sim-outputs.md) | [Model Parameters](/docs/model-parameters.md) | [Sampling Custom Data](/docs/custom-data.md)

# Model Parameters
## Changing Parameters in the Web UI
The web UI presented when running `mesa runserver` or ['run.py'](/mesa_spatial_sampling_MRS/run.py)
allows you to change some parameters of the model using interactive elements such as
drop-down selections and sliders. For these changes to take effect, you must restart
the simulation by clicking the 'Reset' button in the menu bar. Some parameters
can only be changed by modifying code directly in ['server.py'](/mesa_spatial_sampling_MRS/server.py), or
['BatchRunner_RR_and_SSI.py'](/mesa_spatial_sampling_MRS/BatchRunner_RR_and_SSI.py), for
effecting the web-connected simulation, or headless batches of simulations respectively.

### The following parameters are exposed via the web UI:

| Parameter                                     | Description                                                                                                                                                                                                      | Default Value                            | 
|-----------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------|
| Maximum Time Steps for Sampling (`max_steps`) | Sets a time step at which to stop the simulation and save performance metrics.                                                                                                                                   | `0` (run indefinitely)                   |
| Number of Robots (`num_robots`)               | The number of robots to be deployed for the MRS.                                                                                                                                                                 | `3`                                      |
| Task Allocation Algorithm (`task_allocation`) | The algorithm to be used to allocate sampling tasks to robots. Can be `"Sequential Single Item (SSI) auction"` or `"Round Robin"`.                                                                               | `"Sequential Single Item (SSI) auction"` |
| Sampling Strategy (`sampling_strategy`)       | The strategy to be used to select locations for sampling. Can be `"Random"` or `"Dynamic"`. The dynamic sampling methodlogy is described in the [TAROS '22 paper](https://doi.org/10.1007/978-3-031-15908-4_20). | `"Dynamic"`                              |

## Changing Parameters in Code
The above parameters accessible via the web UI can also be changed within the
['server.py'](/mesa_spatial_sampling_MRS/server.py), or
['BatchRunner_RR_and_SSI.py'](/mesa_spatial_sampling_MRS/BatchRunner_RR_and_SSI.py)
scripts. In both scripts, the values of model parameters, including some not exposed by 
the web UI, are defined within a dictionary. In `server.py` this dictionary
is called `model_params`. An example of defining its values can be seen in `server.py`: 
```python
model_params = {
    "max_steps": max_steps_slider,
    "height": height,
    "width": width,
    "num_robots": num_robots_slider,
    "vis_freq": 5,
    "task_allocation": task_allocation_choice,
    "sampling_strategy": sampling_choice
}
```

In `BatchRunner_RR_and_SSI.py` parameters that are constant across a batch of trials
are set in the `fixed_parameters` argument of instances of `BatchRunnerMP`.

### The following parameters can only be changed within the code:

| Parameter                            | Description                                                                                                                                                                                                                                                     | Default Value         |
|--------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------|
| Height (`height`)                    | Defines the height (in cells) of the grid-world. In `server.py` this is set to the height of the data provided for the robots to sample. In `BatchRunner_RR_and_SSI.py` you will need to manually set this to the height of your data, if sampling custom data. | 20                    |
| Width (`width`)                      | Defines the width (in cells) of the grid-world. In `server.py` this is set to the width of the data provided for the robots to sample. In `BatchRunner_RR_and_SSI.py` you will need to manually set this to the width of your data, if sampling custom data.    | 20                    |
| Visualisation Frequency (`vis_freq`) | The frequency, in simulation time steps, at which to generate visual outputs. Lower frequencies can significantly degrade the simulation's performance.                                                                                                         | 5                     |
| Results Directory (`results_dir`)    | The path to a directory in which data and visual outputs will be saved.                                                                                                                                                                                         | `"./results/default"` |
| Verbose (`verbose`)                  | If True, the simulation prints additional information to the terminal.                                                                                                                                                                                          | True                  |