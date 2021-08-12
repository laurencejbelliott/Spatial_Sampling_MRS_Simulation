import numpy as np
import matplotlib.pyplot as plt


def makeGaussian(size, fwhm = 3, center=None):
    """ Make a square gaussian kernel.

    size is the length of a side of the square
    fwhm is full-width-half-maximum, which
    can be thought of as an effective radius.
    """

    x = np.arange(0, size, 1, float)
    y = x[:,np.newaxis]

    if center is None:
        x0 = y0 = size // 2
    else:
        x0 = center[0]
        y0 = center[1]

    return np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)


if __name__ == "__main__":
    gaussian = makeGaussian(10)
    # sampled = np.ones((10, 10)) * 255
    sampled = np.zeros((10, 10))
    sampled[3:5, 3:5] = gaussian[3:5, 3:5]
    plt.imshow(sampled, cmap='gray', interpolation='nearest')
    plt.show()
