
from matplotlib import pyplot as plt
from matplotlib import patches
from math import *


def plot_hemisphere(ppd, ax, show=False):

    # Compute diameter of hemisphere in pixels
    diam = ppd * 180
    center = (diam / 2, diam / 2)

    for deg in range(0, 100, 10):

        # Radians
        rad = radians(deg)

        # Create 2 ellipses, 1 horizontally oriented and 1 vertically oriented
        e1 = patches.Ellipse(center, diam * sin(rad), diam)
        e2 = patches.Ellipse(center, diam, diam * sin(rad))

        # Set appearance
        e1.fill = False
        e1.set_linewidth(0.5)
        e2.fill = False
        e2.set_linewidth(0.5)

        # Add to ax
        ax.add_artist(e1)
        ax.add_artist(e2)

        # Set bounds around hemisphere
        ax.set_xlim(0, diam)
        ax.set_ylim(0, diam)

    # Show (optional)
    if show:
        plt.show()


def calc_gnd_pt(roll, pitch, ppd):

    # Find center of hemisphere in pixels
    diam = ppd * 180
    center = [diam / 2, diam / 2]

    # Precompute some values
    t = diam / 2
    t_sq = t**2
    sin_2_p = sin(radians(pitch)) ** 2
    sin_2_r = sin(radians(roll)) ** 2

    # Compute x^2, x
    x_sq = (t_sq * sin_2_r * (1 - sin_2_p)) / (1 - sin_2_r * sin_2_p + 0.00000001)
    x = sqrt(x_sq)

    # Compute y
    y = sqrt(t_sq * sin_2_p * (1 - x_sq / t_sq))

    return [center[0] + x, center[1] + y]

# Params
DEG_PER_PIX = 160 / 640
PIX_PER_DEG = 1 / DEG_PER_PIX
ROLL = 20
PITCH = 30

if __name__ == "__main__":

    # Create figure
    fig, ax = plt.subplots(subplot_kw={'aspect': 'equal'})

    # Draw hemisphere
    plot_hemisphere(PIX_PER_DEG, ax)

    # Compute ground point and plot
    gp = calc_gnd_pt(ROLL, PITCH, PIX_PER_DEG)
    plt.plot(gp[0], gp[1], 'o')





    # # Draw pitch and roll ellipses
    # pitch = -60
    # roll = 20
    # ell_test_p = patches.Ellipse((0,0), 2, 2*sin(radians(pitch)))
    # ell_test_r = patches.Ellipse((0,0), 2*sin(radians(roll)), 2)
    # ell_test_p.fill = False
    # ell_test_p.set_linewidth(1)
    # ell_test_r.fill = False
    # ell_test_r.set_linewidth(1)
    # ell_test_p.set_color('b')
    # ell_test_r.set_color('r')
    # ax.add_artist(ell_test_p)
    # ax.add_artist(ell_test_r)


    # p_rad = radians(pitch)
    # r_rad = radians(roll)
    #
    # # Compute intersection point
    # num = sin(r_rad)**2 * (1 - sin(p_rad)**2)
    # den = 1 - sin(r_rad)**2 * sin(p_rad) ** 2
    # x = sqrt(num / den)
    # y = sqrt(sin(p_rad)**2 * (1 - x**2))
    # x = -x if roll < 0 else x
    # y = -y if pitch < 0 else y
    #
    # plt.plot(x,y,'o')

    # Plot the points and show
    plt.show()




