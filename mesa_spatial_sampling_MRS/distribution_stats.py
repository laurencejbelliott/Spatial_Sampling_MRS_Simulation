__author__ = "Laurence Roberts-Elliott"
from gaussian import makeGaussian
from matplotlib import pyplot as plt
import numpy as np

size = 20

distribution = makeGaussian(size)

plt.imshow(distribution, interpolation='nearest')
plt.colorbar()
plt.title("Ground-truth distribution")
plt.xlabel("X-axis")
plt.ylabel("Y-axis")
plt.show()
plt.savefig("ground_truth.png")
print("Ground-truth distribution saved as ground_truth.png")
plt.close()

# Calculate STD of the distribution
std = np.std(distribution)
print(f"Standard deviation of the distribution: {std}")