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
    # Returns: List of (n+1) 4*4 sympy arrays
    results = []
    result = eye(4) # identity array, T00
    results.append(result)
    for dh_param in dh_params:
        result *= form_homo_trans(*dh_param) # T10 to Tn0
        results.append(result)
    return results

def form_homo_rep(x, y, z):
    """ Returns a homogeneous representation from coordinates """
    return Matrix([[x], [y], [z], [1]])

def convert_frame(coords, trans):
    """ Transforms coordinates in one frame to another frame using trans """
    # coords: 4*1 sympy array
    # trans: 4*4 sympy array
    # Returns: 4*1 sympy array
    return trans * coords

def get_e_pos(fk_map):
    """ Get end effector position from fk map """
    coords = convert_frame(form_homo_rep(0, 0, 0), fk_map)
    return coords[0, 0], coords[1, 0], coords[2, 0]
