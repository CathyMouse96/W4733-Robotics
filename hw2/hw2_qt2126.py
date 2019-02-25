import sys

from sympy import *

from Kinematics import ForwardKinematics as FK
from Kinematics import DifferentialKinematics as DK

def main():
    if not len(sys.argv) == 8:
        print("Usage: python hw2_qt2126.py t1 t2 t3 t4 t5 t6 t7")
        exit(1)
    
    v1, v2, v3, v4, v5, v6, v7 = [float(x) for x in sys.argv[1:]]

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

    fk_maps = FK.fk(dh_params)
    e_pos = FK.get_e_pos(fk_maps[-1]) # end effector coordinates

    # Linear Velocity Jacobian
    jp = DK.get_linear_velocity_jacobian(e_pos, (t1, t2, t3, t4, t5, t6, t7))

    # Angular Velocity Jacobian
    jo = DK.get_angular_velocity_jacobian(fk_maps[:-1])

    # Full Jacobian
    j = DK.get_full_jacobian(jp, jo)
  
    # Sanity check
    # pprint(j.evalf(subs={t1: 0.1, t2: 0.2, t3: 0.3, t4: -0.4, t5: 0.5, t6: 0.6, t7:0.7}))

    jv = j.evalf(subs={t1: v1, t2: v2, t3: v3, t4: v4, t5: v5, t6: v6, t7: v7})
    pprint(jv)

if __name__ == "__main__":
    main()
