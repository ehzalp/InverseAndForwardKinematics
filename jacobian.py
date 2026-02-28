import numpy as np

def jacobian_xy_planar_3r(q: np.ndarray, L: np.ndarray) -> np.ndarray:
    """
    Planar 3R için uç efektörün (x,y) konum Jacobian'ı: J_xy (2x3)
    p = [x(q), y(q)]
    """
    q1, q2, q3 = q
    L1, L2, L3 = L

    t1   = q1
    t12  = q1 + q2
    t123 = q1 + q2 + q3

    # x = L1 cos(t1) + L2 cos(t12) + L3 cos(t123)
    # y = L1 sin(t1) + L2 sin(t12) + L3 sin(t123)

    dx_dq1 = -L1*np.sin(t1)  - L2*np.sin(t12)  - L3*np.sin(t123)
    dx_dq2 =                 - L2*np.sin(t12)  - L3*np.sin(t123)
    dx_dq3 =                                  - L3*np.sin(t123)

    dy_dq1 =  L1*np.cos(t1)  + L2*np.cos(t12)  + L3*np.cos(t123)
    dy_dq2 =                  L2*np.cos(t12)   + L3*np.cos(t123)
    dy_dq3 =                                   L3*np.cos(t123)

    J = np.array([
        [dx_dq1, dx_dq2, dx_dq3],
        [dy_dq1, dy_dq2, dy_dq3],
    ], dtype=float)

    return J


def jacobian_xyphi_planar_3r(q: np.ndarray, L: np.ndarray) -> np.ndarray:
    """
    Planar 3R için [x, y, phi] Jacobian'ı: J (3x3)
    phi = q1 + q2 + q3
    Böylece det(J) tanımlı olur.
    """
    Jxy = jacobian_xy_planar_3r(q, L)
    J = np.vstack([
        Jxy,
        np.array([[1.0, 1.0, 1.0]], dtype=float)
    ])
    return J


def det_jacobian_xyphi(q: np.ndarray, L: np.ndarray) -> float:
    """
    3x3 Jacobian determinant (x,y,phi).
    """
    J = jacobian_xyphi_planar_3r(q, L)
    return float(np.linalg.det(J))


def is_singular(q: np.ndarray, L: np.ndarray, eps: float = 1e-9) -> bool:
    """
    det(J) ~ 0 ise singular kabul eder.
    """
    return abs(det_jacobian_xyphi(q, L)) < eps
