import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
from matplotlib import colors, cm
#from plot_util import plot_reference_frame

data = np.loadtxt('build/path.txt')

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(data[:, 0], data[:, 1], data[:, 2])
plt.show()

# get link positions from data
# add zeros column for origin
# N = data.shape[0]
# x = np.vstack(( np.zeros(N), data[:, 0] )).T
# y = np.vstack(( np.zeros(N), data[:, 1] )).T
# z = np.vstack(( np.zeros(N), data[:, 2] )).T

# # p = data[:, :3]
# # q = data[:, 3:]

# # setup color scale to represent index along path
# cMap = cm.ScalarMappable(
#             norm=colors.Normalize(vmin=0, vmax=N - 1),
#             cmap=plt.get_cmap('winter'))

# # plot the robot links for each path point
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')


# c1 = p[:, 0] - 1.0 * (q[:, 3]**2 - q[:, 2]**2)
# c2 = p[:, 1] - 1.0 * 2 * q[:, 3] * q[:, 2]
# c3 = p[:, 2]
# print(c1)
# print(c2)
# print(c3)

# # q = (rx, ry, rz, w)
# # rotation around z
# # for i in range(N):
# #     rz =  np.arctan2(q[i][2], q[i][3]) * 2.0
# #     print("Y: {:.2f} RZ: {:.2f}".format(p[i][1], rz))

# for i in range(N):
#     ax.plot(x[i], y[i], z[i], '-o', c=cMap.to_rgba(i))
#     #plot_reference_frame(ax, p[i], q[i])


# # ax.axis('equal')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_zlim([0, 1])
# plt.show(block=True)