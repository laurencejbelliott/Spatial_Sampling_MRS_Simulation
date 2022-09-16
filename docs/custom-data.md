[Home](https://github.com/laurencejbelliott/Spatial_Sampling_MRS_Simulation/) | [Accessing Simulation Outputs](/docs/sim-outputs.md) | [Model Parameters](/docs/model-parameters.md) | [Sampling Custom Data](/docs/custom-data.md)

# Sampling Custom Data
## Preparing Your Data for the Simulation

Although the simulation defaults to simulating a MRS sampling surface soil compaction data,
you can provide the simulation with your own geolocated data for the robots to sample.
This data should be provided in a CSV file, with each record containing the following fields:
* The latitude of the sample location, in decimal degrees.
* The longitude of the sample location, in decimal degrees.
* The value of the sample at the location.

The names of these fields can be set to match those in your dataset, changing the default keys
in `compaction_data_formatting.py` from `lat`, `lon`, and `0cm` to the names of your fields. It
is recommended that you create a copy of `compaction_data_formatting.py` to modify for preparing
your own data for the simulation, e.g. [example_custom_data_formatting.py](https://github.com/laurencejbelliott/Spatial_Sampling_MRS_Simulation/blob/custom_data_docs/mesa_spatial_sampling_MRS/example_custom_data_formatting.py)
as in the `custom_data_docs` branch of this repository. This branch is provided as an example of modifying the simulation 
to sample a different dataset, and is not intended to be merged into the main branch. This data used in this example 
comes from Kashparov et al.'s (2017) dataset ['Spatial datasets of radionuclide contamination in the Ukrainian Chernobyl Exclusion Zone'](https://doi.org/10.5285/782ec845-2135-4698-8881-b38823e533bf).

In your copy of `compaction_data_formatting.py`, change 
```python
custom_df = pd.read_csv("...")
```
to 
```python
custom_df = pd.read_csv("path/to/your/data.csv")
```
and change the default keys to match the names of your fields. The easiest way method is to find and replace `"lat"`,
`"lon"`, and `"0cm"` with the names of your fields as they appear in your dataset.

The script outputs the extents of your data in decimal degrees, so that you can calculate the dimensions of the sampled
environment. The simulation assumes this environment is rectangular, so a bounding box should be drawn around the 
sample locations. The width and height of this bounding box should be measured in meters as longitudinal and
latitudinal distances, respectively. This can be done using measurement tools in GIS software or
some online map services such as Google Maps. Set the values of `env_width` and `env_height` according to these
measurements.

Next the script will show you a plot of your data points, and subsequently attempt to perform kriging
interpolation on your data. This is done to generate a continuous surface of your data, which can be sampled at
arbitrary locations in the simulation. This can fail with an error reporting that the interpolation tried to reserve too
much memory. If this occurs, you may need to lower the resolution of the interpolation. E.g.:

```python
# Scaling down environment resolution due to memory limitations
env_width /= 100
env_height /= 100
```

Try different downscaling factors until the interpolation succeeds. Please note that reducing the interpolation
resolution increases the area that each cell represents in the output surface, and in-turn in the simulation's
grid-world.

You may alternatively, or in addition, select a smaller subregion of your data to interpolate and sample. E.g.:
```python
# Define bounds of subregion of the Chernobyl exclusion zone to sample in simulation
custom_df = custom_df[custom_df["Longitude"] <= 30.0986]  # WGS-84 degrees
custom_df = custom_df[custom_df["Longitude"] >= 30.0383]

custom_df = custom_df[custom_df["Latitude"] <= 51.4714]
custom_df = custom_df[custom_df["Latitude"] >= 51.4216]
```

In this example, this subregion was selected as it was observed to have high variance in the data, when viewing IDW
interpolation of the data in QGIS.

You may wish to change the titles and filenames of the visual outputs of the script to refer to your own data, changing
the arguments given in calls to `plt.title()` and `plt.savefig()`.

Lastly, it is also recommended that you change the pickle file storing the interpolated surface to a unique name, e.g.:
```python
# Save interpolated values to be sampled in simulation
with open(r"my_interpolated_data.pickle", "wb") as output_file:
    pickle.dump(m, output_file)
```

## Modifying the Simulation to Sample Your Data
### In `model.py`:
```python
with open(r"interpolated_jaime_compaction_0cm_kpas.pickle", "rb") as input_file:
    self.gaussian = np.array(pickle.load(input_file))
```
should be modified to load your interpolated surface, e.g.:
```python
with open(r"my_interpolated_data.pickle", "rb") as input_file:
    self.gaussian = np.array(pickle.load(input_file))
```

### In `server.py`:
Repeat the process for `model.py` in `server.py`, and also change the multiplier of `canvas_width` and `canvas_height`
e.g.

```python
canvas_width = width * 10
canvas_height = height * 10
```
Be sure to set the same multiplier for both `canvas_width` and `canvas_height`, and try different values until
the grid-world is displayed at a reasonable scale in the Mesa web UI.

With all of these changes made, you should be able to run the simulation with the bash command `python run.py`, or by
calling `mesa runserver` in the terminal with `mesa_spatial_sampling_MRS` as your working directory,
while you have sourced a virtual environment containing the required dependencies.