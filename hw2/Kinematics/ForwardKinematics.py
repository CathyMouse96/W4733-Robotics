import math

from sympy import *

def form_homo_trans(a, alpha, d, theta):
    """ Returns a homogeneous transformation from DH parameters """
    ctheta, stheta = cos(theta), sin(theta)
    calpha, salpha = cos(alpha), sin(alpha)
    return Matrix([
        [ctheta, -stheta * calpha, stheta * salpha, a * ctheta],
        [stheta, ctheta * calpha, -ctheta * salpha, a * stheta],
        [0, salpha, calpha, d],
        [0, 0, 0, 1]
    ])

def fk(dh_params):
    """ Computes the forward kinematics with a list of DH parameters """
    # dh_params: List[List]
    # Returns: 4*4 sympy array
    result = eye(4) # identity array
    for dh_param in dh_params:
        result = result * form_homo_trans(*dh_param)
    return result

def form_homo_rep(x, y, z):
    """ Returns a homogeneous representation from coordinates """
    return Matrix([[x], [y], [z], [1]])

def convert_frame(coords, trans):
    """ Transforms coordinates in one frame to another frame using trans """
    # coords: 4*1 sympy array
    # trans: 4*4 sympy array
    # Returns: 4*1 sympy array
    return trans * coords

def element_at(matrix, row, col):
    """ Get the scalar at some row and col """
    return matrix.row(row).col(col).dot(eye(1))

def get_e_pos(fk_map):
    """ Get end effector position from fk map """
    coords = convert_frame(form_homo_rep(0, 0, 0), fk_map)
    return element_at(coords, 0, 0), element_at(coords, 1, 0), element_at(coords, 2, 0)
