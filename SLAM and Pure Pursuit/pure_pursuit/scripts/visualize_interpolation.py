import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

# Define some points:
data = np.genfromtxt('waypoints.csv', delimiter=',')
points = data[:,0:2]
points = np.vstack((points, points[0]))

# Linear length along the line:
distance = np.cumsum( np.sqrt(np.sum( np.diff(points, axis=0)**2, axis=1 )) )
distance = np.insert(distance, 0, 0)/distance[-1]

# Interpolation for different methods:
interpolations_methods = ['cubic']
alpha = np.linspace(0, 1, 1020)

interpolator =  interp1d(distance, points, kind='cubic', axis=0)
interpolated_points = interpolator(alpha)

interpolated_points = {}
for method in interpolations_methods:
    interpolator =  interp1d(distance, points, kind=method, axis=0)
    interpolated_points[method] = interpolator(alpha)
    # interpolated_points[method] = np.delete(interpolated_points[method], [0,1,2,3,4,5], 0)

# Graph:
for method_name, curve in interpolated_points.items():
    plt.plot(*curve.T, '-', label=method_name);

plt.plot(*points.T, 'ok', label='original points');
plt.scatter(0,0, c='red')
idx = 711
plt.scatter(interpolated_points['cubic'][idx,0], interpolated_points['cubic'][idx,1], s=200, c='green')
plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y');
plt.show()