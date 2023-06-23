from scipy import interpolate
import matplotlib.pyplot as plt
import numpy as np
import math
import math
import numpy as np
from scipy import interpolate
from collections import OrderedDict


def distance_between_points(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def points_on_line(x1, y1, x2, y2, n):
    dx = (x2 - x1) / (n - 1)
    dy = (y2 - y1) / (n - 1)
    return [(x1 + i * dx, y1 + i * dy) for i in range(n - 1)] + [(x2, y2)]

def smooth_path_bspline(waypoints):
    interpolated_points = []

    for i in range(0, len(waypoints) - 1):
        num_of_points = int(distance_between_points(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][1]))
        new_points = points_on_line(waypoints[i][0], waypoints[i][1], waypoints[i+1][0], waypoints[i+1][1], (num_of_points)+2)
        for p in new_points:
            interpolated_points.append(p)

    unique_dict = OrderedDict.fromkeys(interpolated_points)
    unique_list = list(unique_dict.keys())

    x = []
    y = []

    for point in unique_list:
        x.append(point[0])
        y.append(point[1])
    
    tck, *rest = interpolate.splprep([x, y], s=0.001)
    u = np.linspace(0, 1, num=100)
    smooth= interpolate.splev(u, tck)

    path = []
    for i in range(0, len(smooth[0])):
        path.append((smooth[0][i], smooth[1][i]))


    return path