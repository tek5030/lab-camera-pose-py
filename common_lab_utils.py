import numpy as np
from pygeodesy.ltp import LocalCartesian
from dataclasses import dataclass
from pylie import SO3, SE3


@dataclass
class Attitude:
    """Attitude represented as a raw set of Euler angles."""

    x_rot: float    # Angle around x-axis in radians.
    y_rot: float    # Angle around y-axis in radians.
    z_rot: float    # Angle around z-axis in radians.


@dataclass
class CartesianPosition:
    """Position in cartesian space as raw x, y, z values in meters."""
    x:  float       # Position along x-axis in meters.
    y:  float       # Position along y-axis in meters.
    z:  float       # Position along z-axis in meters.


@dataclass
class GeodeticPosition:
    """Geodetic position."""
    latitude:   float   # Latitude in degrees.
    longitude:  float   # Longitude in degrees.
    altitude:   float   # Altitude in meters.

@dataclass
class Intrinsics:
    """Intrinsic camera calibration parameters."""
    fu: float       # Focal length in the horizontal u-direction.
    fv: float       # Focal length in the vertical v-direction.
    s:  float       # Skew.
    cu: float       # Principal coordinate in the horizontal u-direction.
    cv: float       # Principal coordinate in the vertical v-direction.

    k1: float       # First radial distortion coefficient.
    k2: float       # Second radial distortion coefficient.
    k3: float       # Third radial distortion coefficient.


class LocalCoordinateSystem:
    """Computes and represents a local Cartesian coordinate system."""

    def __init__(self, origin: GeodeticPosition):
        self._local = LocalCartesian(origin.latitude, origin.longitude, origin.altitude)

    def to_local_pose(self, position_geodetic_body: GeodeticPosition, orientation_ned_body: SO3):
        """Transforms geodetic position and attitude to pose in local coordinate system."""

        # Compute transform from geodetic/NED representation to local Cartesian coordinate system.
        transform = self._local.forward(position_geodetic_body.latitude,
                                        position_geodetic_body.longitude,
                                        position_geodetic_body.altitude,
                                        True)

        # Compute orientation
        R_local_ned = np.array(transform.M).reshape(3, 3).transpose()
        orientation_local_body = SO3(R_local_ned) @ orientation_ned_body

        position_local_body = np.array([transform.toNed()[:3]]).T

        return SE3((orientation_local_body, position_local_body))


def homogeneous(x):
    """Transforms a Cartesian vector to a homogeneous vector"""
    return np.r_[x, [np.ones(x.shape[1])]]


def hnormalized(x):
    """Transforms a homogeneous vector to a Cartesian vector"""
    return x[:-1] / x[-1]
