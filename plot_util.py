import numpy as np
from pyquaternion import Quaternion

def plot_reference_frame(ax, p, q, l=0.2):
    """ Plot xyz-axes on axes3d object
    
    Parameters
    ----------
    ax : mpl_toolkits.mplot3d.Axes3D
        Axes object for 3D plotting.
    tf : np.array of float
        Transform to specify location of axes. Plots in origin if None.
    l : float
        The length of the axes plotted.
    
    """
    # different convertion in pyquaternion q = (w, rx, ry, rz)
    pyq = Quaternion(w=q[3], x=q[0], y=q[1], z=q[2])
    R = pyq.rotation_matrix

    x_axis = np.array([[0, l], [0, 0], [0, 0]])
    y_axis = np.array([[0, 0], [0, l], [0, 0]])
    z_axis = np.array([[0, 0], [0, 0], [0, l]])
    
    # rotation
    x_axis = np.dot(R, x_axis)
    y_axis = np.dot(R, y_axis)
    z_axis = np.dot(R, z_axis)

    # translation [:, None] to change shape (add axis)
    x_axis = x_axis + p[:, None]
    y_axis = y_axis + p[:, None]
    z_axis = z_axis + p[:, None]

    ax.plot(x_axis[0], x_axis[1], x_axis[2], '-', c='r')
    ax.plot(y_axis[0], y_axis[1], y_axis[2], '-', c='g')
    ax.plot(z_axis[0], z_axis[1], z_axis[2], '-', c='b')


def plot_unit_sphere(ax):
    """
    from  https://stackoverflow.com/questions/32424670/python-matplotlib-drawing-3d-sphere-with-circumferences
    """
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = 1 * np.outer(np.cos(u), np.sin(v))
    y = 1 * np.outer(np.sin(u), np.sin(v))
    z = 1 * np.outer(np.ones(np.size(u)), np.cos(v))

    ax.plot_surface(x, y, z,  rstride=4, cstride=4, color='y', linewidth=0, alpha=0.2)