from kriging import predict_by_kriging
import numpy as np

# Define parameters (Explanations are in kriging.py)
xgrid = np.arange(1, 10, 1)
ygrid = np.arange(1, 10, 1)
x_arr = [1, 1,   2, 2,   3, 4,   5, 6, 7, 9  ]
y_arr = [1, 9,   2, 5,   3, 6,   4, 5, 6, 4  ]
o_arr = [3, 4.2, 4, 4.5, 5, 5.5, 5, 2, 1, 3.3]

variogram = 'gaussian'

# Run prediction 
m, v = predict_by_kriging(xgrid, ygrid, x_arr, y_arr, o_arr, variogram=variogram)

# Export as images for instruction purpose 
import matplotlib.pyplot as plt
plt.figure('Mean')
plt.imshow(m)
plt.savefig('Mean.png')
plt.figure('Variance')
plt.imshow(v)
plt.savefig('Variance.png')