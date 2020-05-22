import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from matplotlib import colors, cm
from plot_util import plot_reference_frame

data = np.loadtxt('build/path_se3.txt')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2])

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# ax.scatter(0.5, 0.5, 0.5, "ro")
ax.scatter(0, 0, 0, "ro")

for pose in data:
    plot_reference_frame(ax, pose[:3], pose[3:])
    # quaternion norm?
    print(np.linalg.norm(pose[3:]))

plt.show()