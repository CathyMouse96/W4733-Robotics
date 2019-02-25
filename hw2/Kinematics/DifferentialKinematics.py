from sympy import *

def get_linear_velocity_jacobian(pos, joints):
    """ Takes end effector position and computes JP """
    x, y, z = pos
    # TODO: Remove hardcode
    t1, t2, t3, t4, t5, t6, t7 = joints
    return Matrix([
        [diff(x, t1), diff(x, t2), diff(x, t3), diff(x, t4), diff(x, t5), diff(x, t6), diff(x, t7)],
        [diff(y, t1), diff(y, t2), diff(y, t3), diff(y, t4), diff(y, t5), diff(y, t6), diff(y, t7)],
        [diff(z, t1), diff(z, t2), diff(z, t3), diff(z, t4), diff(z, t5), diff(z, t6), diff(z, t7)]
    ])
