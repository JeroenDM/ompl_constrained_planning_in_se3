import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from plot_util import plot_reference_frame, plot_unit_sphere

data = np.loadtxt('build/path_se3.txt')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2])
plot_unit_sphere(ax)
for pose in data:
    plot_reference_frame(ax, pose[:3], pose[3:])

# cosmetics
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel("X", fontsize=16)
ax.set_ylabel("Y", fontsize=16)
ax.set_zlabel("Z", fontsize=16)

ax.xaxis.pane.fill = False
ax.yaxis.pane.fill = False
ax.zaxis.pane.fill = False

ax.set_title("Constrained SE3 plannig on sphere", fontsize=20)

plt.show()