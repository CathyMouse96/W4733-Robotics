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

def get_angular_velocity_jacobian(fk_maps):
    """ Takes fk maps (T00 to T(n-1)0) and computes JO """
    jo = Matrix()
    cnt = 0
    for fk_map in fk_maps:
        jo = jo.col_insert(cnt, fk_map[:3, 2:3]) # strange behavior when insert at -1; makes me angry, grrr
        cnt += 1
    return jo

def get_full_jacobian(jp, jo):
    """ Combines JP and JO """
    return jp.row_insert(3, jo)
