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

| Parameter                       | Description                                                                                                                                                                                                      | Default Value                            | 
|---------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------|
| Maximum Time Steps for Sampling | Sets a time step at which to stop the simulation and save performance metrics.                                                                                                                                   | `0` (run indefinitely)                   |
| Number of Robots                | The number of robots to be deployed for the MRS.                                                                                                                                                                 | `3`                                      |
| Task Allocation Algorithm       | The algorithm to be used to allocate sampling tasks to robots. Can be `"Sequential Single Item (SSI) auction"` or `"Round Robin"`.                                                                               | `"Sequential Single Item (SSI) auction"` |
| Sampling Strategy               | The strategy to be used to select locations for sampling. Can be `"Random"` or `"Dynamic"`. The dynamic sampling methodlogy is described in the [TAROS '22 paper](https://doi.org/10.1007/978-3-031-15908-4_20). | `"Dynamic"`                              |

### The following parameters can only be changed within the code:

To be completed...