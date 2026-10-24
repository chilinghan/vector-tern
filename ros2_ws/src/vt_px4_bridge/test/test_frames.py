import math
import random

from vt_px4_bridge.frames import covariance_diagonals, flu_to_frd_quaternion, flu_to_frd_vector

F = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]


def quat_to_matrix(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def random_quat(rng):
    q = [rng.gauss(0, 1) for _ in range(4)]
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def test_quaternion_matches_conjugation_by_F():
    rng = random.Random(0)
    for _ in range(100):
        q = random_quat(rng)
        expected = matmul(matmul(F, quat_to_matrix(q)), F)
        got = quat_to_matrix(flu_to_frd_quaternion(q))
        for i in range(3):
            for j in range(3):
                assert abs(expected[i][j] - got[i][j]) < 1e-9


def test_identity_stays_identity():
    assert flu_to_frd_quaternion((1, 0, 0, 0)) == (1, 0, 0, 0)


def test_yaw_left_becomes_yaw_negative_in_frd():
    # +90 deg yaw about z-up (turn left) is -90 deg about z-down.
    s = math.sqrt(0.5)
    w, x, y, z = flu_to_frd_quaternion((s, 0, 0, s))
    assert (w, x, y, z) == (s, 0, 0, -s)


def test_vector():
    # forward/left/up -> forward/right/down
    assert flu_to_frd_vector((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


def test_covariance_clamped():
    cov = [0.0] * 36
    cov[0], cov[7], cov[14], cov[35] = 0.01, 0.02, 0.03, 0.04
    pos, rot = covariance_diagonals(cov, 1e-4)
    assert pos == [0.01, 0.02, 0.03]
    assert rot == [1e-4, 1e-4, 0.04]


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
