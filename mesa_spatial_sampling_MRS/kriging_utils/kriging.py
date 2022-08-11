import numpy as np
from pykrige.ok import OrdinaryKriging


def predict_by_kriging(gridx, gridy, x_arr, y_arr, o_arr, variogram='spherical'):
    """
    Predict values at unsampled locations based on observations from other locations.
    Use Ordinary Kriging with a particular variogram implemented in PyKrige. 
    Note that at least three observations are recommended as input because otherwise the 
    OrdinaryKriging function generates some errors.
    Tutorial: https://geostat-framework.readthedocs.io/projects/pykrige/en/stable/examples/00_ordinary.html
    
    Inputs: 
    - gridx, gridy: 1D array or list of x,y coordinates to print in the output 
    - x_arr, y_arr: 1D array or list of x,y coordinates of observed locations 
    - o_arr: 1D array or list of observed values 
    - variogram: Type of variogram model to use, i.e., one of 'gaussian', 'exponential', 'spherical', or 'linear'
      - Ref: https://geostat-framework.readthedocs.io/projects/pykrige/en/stable/variogram_models.html
      
    Outputs:
    - m: 2D arrays of the means from Kriging predictions
    - v: 2D arrays of the variances from Kriging predictions
    """
    
    assert len(x_arr) == len(y_arr) and len(y_arr) == len(o_arr), 'Lengths must be equivalent in x_arr, y_arr, and o_arr'
    
    gridx = np.asarray(gridx, dtype=np.float32)
    gridy = np.asarray(gridy, dtype=np.float32)
    x_arr = np.asarray(x_arr, dtype=np.float32)
    y_arr = np.asarray(y_arr, dtype=np.float32)
    o_arr = np.asarray(o_arr, dtype=np.float32)
    
    OK = OrdinaryKriging(x_arr, y_arr, o_arr, 
        variogram_model=variogram, enable_statistics=True,
        verbose=0, enable_plotting=False)
    m, v = OK.execute("grid", gridx, gridy)

#     return OK, m, v, OK.delta 
    return m, v