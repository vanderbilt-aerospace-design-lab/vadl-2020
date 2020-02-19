
import numpy as np
from matplotlib import pyplot as plt
from math import *

def compute_densities(altitude):

    # Constants
    D = np.linspace(0, 80, 80)
    PIX_PER_DEG = 3.5

    # Compute distance along ground for each angle
    g_dist = altitude * np.tan(np.radians(D))

    # Compute distance between each angle
    gap = g_dist.tolist()
    gap = np.array([gap[i+1] - gap[i] for i in range(len(gap)-1)])

    # Compute pixels per foot over each span
    ppu = PIX_PER_DEG / gap

    # Compute pixels for full tarp
    ppt = np.square(ppu * 10)

    return ppt


def get_distances(altitude):
    return altitude * np.tan(np.radians(np.linspace(1, 80, 79)))



if __name__ == "__main__":

    # Plot angle vs. density for various altitudes
    legend = []

    for alt in [25, 50, 75, 100]:
        density = compute_densities(alt)
        plt.plot(density, linewidth=1)
        legend += [str(alt) + " ft."]

    plt.title("Pixels spanned by 10ft. square tarp at various altitudes")
    plt.xlabel("Angle to tarp")
    plt.ylabel("Pixels spanned by tarp")
    plt.legend(legend)
    plt.xlim(0, 80)
    plt.ylim(0, 1000)
    plt.show()

    # Plot distance vs. density for various altitudes
    for alt in [25, 50, 75, 100]:
        density = compute_densities(alt)
        plt.plot(get_distances(alt), density, linewidth=1)
        legend += [str(alt) + " ft."]

    plt.title("Pixels spanned by 10ft. square tarp at various altitudes")
    plt.xlabel("Distance to tarp along ground")
    plt.ylabel("Pixels spanned by tarp")
    plt.xlim(0, 600)
    plt.ylim(0, 500)
    plt.legend(legend)
    plt.show()


