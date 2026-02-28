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
    alpha_rad = np.radians(alpha)
    tetha_rad = np.radians(tetha)

    T = np.array([
        [np.cos(tetha_rad), -np.sin(tetha_rad)*np.cos(alpha_rad),  np.sin(tetha_rad)*np.sin(alpha_rad), a*np.cos(tetha_rad)],
        [np.sin(tetha_rad),  np.cos(tetha_rad)*np.cos(alpha_rad), -np.cos(tetha_rad)*np.sin(alpha_rad), a*np.sin(tetha_rad)],
        [0,                  np.sin(alpha_rad),                   np.cos(alpha_rad),                  d],
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
    T01 = dh_transform(0, 0, 0, q[0])
    T12 = dh_transform(L[0], 0, 0, q[1])
    T23 = dh_transform(L[1], 0, 0, q[2])
    
    T02 = T01 @ T12
    T03 = T02 @ T23
    
    position = T03[:3, 3]
    return position


if __name__ == "__main__":
    params= load_params_txt("robot_params.txt")
    robot_type, L, q_limits, q_sample, target = get_robot_config(params)
    position = forward_kinematics(q_sample, L)

    print("=== Robot Configuration ===\n")
    print(f"Robot type: {robot_type}\n")
    print(f"Link lengths: {L}\n")

    print(f"End-effector position: {position}\n")
    print(f"Target position: {target}\n")
    print(f"Position error: {np.linalg.norm(position - target)}\n")
    
    
    print(f"Joint limits: {q_limits}\n")
    print(f"Sample joint angles: {q_sample}")
