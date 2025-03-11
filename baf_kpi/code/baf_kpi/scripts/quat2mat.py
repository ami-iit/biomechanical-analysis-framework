import numpy as np

def quat2mat(q):
    q = np.array(q, dtype=float)
    if not np.isclose(np.linalg.norm(q), 1.0):
        q = q / np.linalg.norm(q)

    R = np.zeros((3, 3))

    R[0, 0] = 1 - 2 * (q[3]**2 + q[2]**2)
    R[1, 1] = 1 - 2 * (q[3]**2 + q[1]**2)
    R[2, 2] = 1 - 2 * (q[2]**2 + q[1]**2)

    R[0, 1] = 2 * q[1] * q[2] - 2 * q[3] * q[0]
    R[1, 0] = 2 * q[1] * q[2] + 2 * q[3] * q[0]

    R[0, 2] = 2 * q[1] * q[3] + 2 * q[2] * q[0]
    R[2, 0] = 2 * q[1] * q[3] - 2 * q[2] * q[0]

    R[1, 2] = 2 * q[2] * q[3] - 2 * q[1] * q[0]
    R[2, 1] = 2 * q[2] * q[3] + 2 * q[1] * q[0]

    return R