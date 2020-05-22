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
