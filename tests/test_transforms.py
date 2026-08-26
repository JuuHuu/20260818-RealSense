import numpy as np

from realsense_aruco.transforms import (
    invert,
    make_transform,
    quaternion_xyzw_to_rotation,
    rotation_to_quaternion_xyzw,
    validate_transform,
)


def test_inverse_round_trip():
    transform = make_transform(np.eye(3), [0.2, -0.1, 0.7])
    np.testing.assert_allclose(transform @ invert(transform), np.eye(4), atol=1e-12)


def test_identity_quaternion():
    np.testing.assert_allclose(rotation_to_quaternion_xyzw(np.eye(3)), [0, 0, 0, 1])


def test_quaternion_rotation_round_trip():
    rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
    quaternion = rotation_to_quaternion_xyzw(rotation)
    np.testing.assert_allclose(quaternion_xyzw_to_rotation(quaternion), rotation, atol=1e-12)


def test_rejects_scaled_rotation():
    transform = np.eye(4)
    transform[0, 0] = 2
    try:
        validate_transform(transform)
    except ValueError as exc:
        assert "orthonormal" in str(exc)
    else:
        raise AssertionError("invalid transform was accepted")
