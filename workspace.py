import os
import numpy as np
import matplotlib.pyplot as plt

from config import load_params_txt, get_robot_config


def generate_workspace_points(L: np.ndarray, q_limits: np.ndarray, n: int = 60):
    """
    Joint limitlerle workspace noktaları üretir.
    n: her mafsal için örnek sayısı (n^3 nokta).
    """
    q1 = np.linspace(q_limits[0, 0], q_limits[0, 1], n)
    q2 = np.linspace(q_limits[1, 0], q_limits[1, 1], n)
    q3 = np.linspace(q_limits[2, 0], q_limits[2, 1], n)

    xs = np.empty(n * n * n, dtype=float)
    ys = np.empty(n * n * n, dtype=float)

    k = 0
    for a in q1:
        for b in q2:
            t12 = a + b
            ca, sa = np.cos(a), np.sin(a)
            c12, s12 = np.cos(t12), np.sin(t12)
            for c in q3:
                t123 = t12 + c
                c123, s123 = np.cos(t123), np.sin(t123)

                x = L[0]*ca + L[1]*c12 + L[2]*c123
                y = L[0]*sa + L[1]*s12 + L[2]*s123

                xs[k] = x
                ys[k] = y
                k += 1

    return xs, ys


def plot_workspace(xs, ys, L, out_png="outputs/workspace.png",
                   target=None, show=False):
    """
    - Workspace scatter
    - Teorik sınırlar: Rmax & Rmin daireleri
    - Origin + (opsiyonel) target işareti
    - PNG export
    """
    os.makedirs(os.path.dirname(out_png) or ".", exist_ok=True)

    L1, L2, L3 = L
    Rmax = L1 + L2 + L3
    Rmin = abs(L1 - L2 - L3)  # planar 3R için teorik min yarıçap

    # Daire parametresi
    t = np.linspace(0, 2*np.pi, 800)
    cx_max, cy_max = Rmax*np.cos(t), Rmax*np.sin(t)
    cx_min, cy_min = Rmin*np.cos(t), Rmin*np.sin(t)

    plt.figure(figsize=(6.8, 6.8))

    # Workspace noktaları (daha okunur)
    plt.scatter(xs, ys, s=1, alpha=0.35, label="Reachable points")

    # Teorik sınırlar
    plt.plot(cx_max, cy_max, linewidth=2, label=f"Theoretical outer boundary (Rmax={Rmax:.2f} m)")
    plt.plot(cx_min, cy_min, linewidth=2, label=f"Theoretical inner boundary (Rmin={Rmin:.2f} m)")

    # Origin
    plt.scatter([0], [0], s=60, marker="x", label="Base (origin)")

    # Target (opsiyonel)
    if target is not None:
        tx, ty = float(target[0]), float(target[1])
        plt.scatter([tx], [ty], s=60, marker="o", label=f"Target ({tx:.2f}, {ty:.2f})")

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Workspace of 3R Planar Robot (joint limits applied)")
    plt.axis("equal")
    plt.grid(True)
    plt.legend(loc="upper right", fontsize=9)

    plt.savefig(out_png, dpi=300, bbox_inches="tight")

    if show:
        plt.show()
    plt.close()


def main():
    params = load_params_txt("robot_params.txt")
    robot_type, L, q_limits, q_sample, target = get_robot_config(params)

    n = 60  # 45-70 arası iyi. 60 = 216k nokta
    xs, ys = generate_workspace_points(L, q_limits, n=n)

    out_png = "outputs/workspace.png"


    # Planar olduğu için target z=0 değilse bile grafikte xy gösteririz
    plot_workspace(xs, ys, L, out_png=out_png, target=target, show=False)

    print("=== Workspace Generation (Report-ready) ===")
    print("Robot type:", robot_type)
    print("L:", L)
    print("Joint limits (rad):\n", q_limits)
    print(f"Generated points: {len(xs)} (n={n} => n^3)")
    print("Saved:", out_png)


if __name__ == "__main__":
    main()