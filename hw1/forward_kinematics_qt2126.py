import math

import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def form_homo_trans(a, alpha, d, theta):
    """ Returns a homogeneous transformation from DH parameters """
    ctheta, stheta = math.cos(theta), math.sin(theta)
    calpha, salpha = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ctheta, -stheta * calpha, stheta * salpha, a * ctheta],
        [stheta, ctheta * calpha, -ctheta * salpha, a * stheta],
        [0, salpha, calpha, d],
        [0, 0, 0, 1]
    ])

def fk(dh_params):
    """ Computes the forward kinematics with a list of DH parameters """
    # dh_params: List[List]
    # Returns: 4*4 numpy array
    result = np.identity(4) # identity array
    for dh_param in dh_params:
        result = np.matmul(result, form_homo_trans(*dh_param))
    return result

def form_homo_rep(x, y, z):
    """ Returns a homogeneous representation from coordinates """
    return np.array([[x], [y], [z], [1]])

def convert_frame(coords, trans):
    """ Transforms coordinates in one frame to another frame using trans """
    # coords: 4*1 numpy array
    # trans: 4*4 numpy array
    # Returns: 4*1 numpy array
    return np.matmul(trans, coords)

def plot_drawing(xs, ys, zs):
    """ Plot the drawing """
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(xs, ys, zs)
    plt.show()

def main():
    xs, ys, zs, output = [], [], [], []

    with open('JointData.txt', 'r') as f:
        lines = f.readlines()
    # print(lines)

    for line in lines:
        line = line[:-1] # remove trailing newline
        t1, t2, t3, t4, t5, t6, t7 = line.split(' ')
        p1 = (0, -math.pi / 2, 0, float(t1))
        p2 = (0, math.pi / 2, 0, float(t2))
        p3 = (45, -math.pi / 2, 550, float(t3))
        p4 = (-45, math.pi / 2, 0, float(t4))
        p5 = (0, -math.pi / 2, 300, float(t5))
        p6 = (0, math.pi / 2, 0, float(t6))
        p7 = (0, 0, 60, float(t7))
        ptool = (0, 0, 120, 0)
        fk_map = fk((p1, p2, p3, p4, p5, p6, p7, ptool))
        # print(fk_map)
        coords = convert_frame(form_homo_rep(0, 0, 0), fk_map)
        # print(coords)
        xs.append(coords[0][0])
        ys.append(coords[1][0])
        zs.append(coords[2][0])
        output.append("{} {} {}\n".format(coords[0][0], coords[1][0], coords[2, 0]))

    with open('MarkerTipData.txt', 'w') as f:
        f.writelines(output)

    plot_drawing(xs, ys, zs)

if __name__ == "__main__":
    main()
