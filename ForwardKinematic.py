import matplotlib.pyplot as plt
import numpy as np
from config import load_params_txt, get_robot_config

def dh_transform(a, alpha, d, tetha):
    """
    Denavit-Hartenberg dönüşüm matrisi oluşturur.
    a: link uzunluğu
    alpha: link açısı
    d: link ofseti
    tetha: eklem açısı
    """

    T = np.array([
        [np.cos(tetha), -np.sin(tetha)*np.cos(alpha),  np.sin(tetha)*np.sin(alpha), a*np.cos(tetha)],
        [np.sin(tetha),  np.cos(tetha)*np.cos(alpha), -np.cos(tetha)*np.sin(alpha), a*np.sin(tetha)],
        [0,                  np.sin(alpha),                   np.cos(alpha),                  d],
        [0,                  0,                                   0,                                  1]
    ])
    return T
    
def forward_kinematics(q, L):
    """
    Verilen eklem açıları (q) ve link uzunlukları (L) için ileri kinematik hesaplar.
    q: np.ndarray (q1, q2, q3)
    L: np.ndarray (L1, L2, L3)
    Çıktı: np.ndarray (x, y, z)
    """
    A1 = dh_transform(L[0], 0, 0, q[0])
    A2 = dh_transform(L[1], 0, 0, q[1])
    A3 = dh_transform(L[2], 0, 0, q[2])
    print("A1:\n", A1)
    print("A2:\n", A2)
    print("A3:\n", A3)
    T03 = A1 @ A2 @ A3
    print("T03:\n", T03)
    position = T03[:3, 3]
    rotation = T03[:3, :3]
    phi = np.arctan2(rotation[1, 0], rotation[0, 0])

    return position, phi


if __name__ == "__main__":
    params= load_params_txt("robot_params.txt")
    robot_type, L, q_limits, q_sample, target = get_robot_config(params)
    position = forward_kinematics(q_sample, L)

    print("=== Robot Configuration ===\n")
    print(f"Robot type: {robot_type}\n")
    print(f"Link lengths: {L}\n")

    print(f"End-effector position: {position}\n")
    print(f"Target position: {target}\n")
    #print(f"Position error: {np.linalg.norm(position - target)}\n")
    
    
    print(f"Joint limits: {q_limits}\n")
    print(f"Sample joint angles: {q_sample}")
