import numpy as np
from config import load_params_txt, get_robot_config
from forwardKinematic import dh_transform, forward_kinematics

import numpy as np

def inverse_kinematics(target, L, phi=0.0):
    """
    Planar 3R analitik IK
    target: np.ndarray (x, y, z)
    L: np.ndarray (L1, L2, L3)
    phi: end-effector orientation (rad)
    """
    x, y, _ = target
    L1, L2, L3 = L

    # Wrist center
    xw = x - L3*np.cos(phi)
    yw = y - L3*np.sin(phi)

    r2 = xw**2 + yw**2
    D = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(D) > 1.0:
        raise ValueError("Hedef pozisyon erişilemez.")

    solutions = []
    for sgn in (+1, -1):
        q2 = np.arctan2(sgn*np.sqrt(1 - D**2), D)
        q1 = np.arctan2(yw, xw) - np.arctan2(L2*np.sin(q2), L1 + L2*np.cos(q2))
        q3 = phi - q1 - q2
        solutions.append(np.array([q1, q2, q3]))

    return solutions

def ik_test():
    params = load_params_txt("robot_params.txt")
    robot_type, L, q_limits, q_sample, target = get_robot_config(params)

    phi = 0.0  # test için sabit seç
    sols = inverse_kinematics(target, L, phi=0.0)

    for i, q_sol in enumerate(sols, 1):
        p_fk, phi_fk = forward_kinematics(q_sol, L)  # forward_kinematics'in (p,phi) döndürmeli
        err_xy = np.linalg.norm(p_fk[:2] - target[:2])

        print(f"\nSolution {i}: q = {q_sol}")
        print(f"FK p = {p_fk}, phi = {phi_fk}")
        print(f"Target xy = {target[:2]}, err_xy = {err_xy}")
if __name__ == "__main__":
    ik_test()

   