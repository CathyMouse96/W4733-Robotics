from sympy import *

from Kinematics import ForwardKinematics as FK
from Kinematics import DifferentialKinematics as DK

t1, t2, t3, t4, t5, t6, t7 = symbols('t1, t2, t3, t4, t5, t6, t7')

dh_params = (
    (0, 0, 0.333, t1),
    (0, -pi/2, 0, t2),
    (0, pi/2, 0.316, t3),
    (0.0825, pi/2, 0, t4),
    (-0.0825, -pi/2, 0.384, t5),
    (0, pi/2, 0, t6),
    (0.088, pi/2, 0, t7),
    # (0, 0, 0.107, 0),
)

fk_map = FK.fk(dh_params)
e_pos = FK.get_e_pos(fk_map) # end effector coordinates

# Linear Velocity Jacobian
jp = DK.get_linear_velocity_jacobian(e_pos, (t1, t2, t3, t4, t5, t6, t7))

# Sanity check
print(jp.evalf(subs={t1: 0.1, t2: 0.2, t3: 0.3, t4: -0.4, t5: 0.5, t6: 0.6, t7:0.7}))
