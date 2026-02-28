import numpy as np

def load_params_txt(path: str) -> dict:

    params = {}
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue

            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip()

            if "#" in val:
                val = val.split("#", 1)[0].strip()

            try:
                params[key] = float(val)
            except ValueError:
                params[key] = val

    return params


def get_robot_config(params: dict):
    """
    Txt'den gelen dict'i 'robot konfig' objelerine dönüştürür.
    Çıktı:
    - robot_type: str
    - L: np.ndarray (L1,L2,L3)
    - q_limits: np.ndarray shape (3,2) -> [[min,max],...]
    - q_sample: np.ndarray (q1,q2,q3)
    - target: np.ndarray (x,y,z)
    """
    robot_type = str(params.get("ROBOT_TYPE", "RRR_PLANAR"))

    L = np.array([params["L1"], params["L2"], params["L3"]], dtype=float)

    q_limits = np.array([
        [params["Q1_MIN"], params["Q1_MAX"]],
        [params["Q2_MIN"], params["Q2_MAX"]],
        [params["Q3_MIN"], params["Q3_MAX"]],
    ], dtype=float)

    q_sample = np.array([
        params["Q_SAMPLE_1"],
        params["Q_SAMPLE_2"],
        params["Q_SAMPLE_3"],
    ], dtype=float)

    target = np.array([
        params["TARGET_X"],
        params["TARGET_Y"],
        params["TARGET_Z"],
    ], dtype=float)

    return robot_type, L, q_limits, q_sample, target