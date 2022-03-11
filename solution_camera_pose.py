import cv2
import numpy as np
from pylie import SO3, SE3
from common_lab_utils import (
    Attitude, CartesianPosition, GeodeticPosition, Intrinsics, LocalCoordinateSystem,
    homogeneous, hnormalized
)
from dataset import Dataset
from viewer_3d import Viewer3D


def run_camera_pose_solution():
    # Set up dataset.
    dataset = Dataset()

    # Define local coordinate system based on the position of a light pole.
    light_pole_position = GeodeticPosition(59.963516, 10.667307, 321.0)
    local_system = LocalCoordinateSystem(light_pole_position)

    # Construct viewers.
    window_name = 'World point in camera'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    viewer_3d = Viewer3D()

    # Process each image in the dataset.
    for element in dataset:
        # Todo 1: Convert geographical position and attitude to local Cartesian pose.
        # Todo 1.1: Finish to_SO3() below.
        position_geodetic = element.body_position_in_geo
        orientation_ned_body = to_SO3(element.body_attitude_in_geo)

        # Compute the pose of the body in the local coordinate system.
        pose_local_body = local_system.to_local_pose(position_geodetic, orientation_ned_body)

        # Todo 1.2: Add body coordinate axes to 3d viewer.
        # Add body coordinate axes to the 3D viewer.
        viewer_3d.add_body_axes(pose_local_body)

        # Todo 2: Compute the pose of the camera
        # Todo 2.1: Finish to_vector() below,
        position_body_camera = to_vector(element.camera_position_in_body)
        orientation_body_camera = to_SO3(element.camera_attitude_in_body)

        # Todo 2.2: Construct pose_body_camera correctly using element.
        # Compute the pose of the camera relative to the body.
        pose_body_camera = SE3((orientation_body_camera, position_body_camera))

        # Todo 2.3: Construct pose_local_camera correctly using the poses above.
        pose_local_camera = pose_local_body @ pose_body_camera

        # Todo 2.4: Add camera coordinate axes to 3d viewer.
        # Add camera coordinate axes to the 3D viewer.
        viewer_3d.add_camera_axes(pose_local_camera)

        # Todo 3: Undistort the images.
        # Todo 3.1-2: Finish PerspectiveCamera.from_intrinsics() below.
        # Construct a camera model based on the intrinsic calibration and camera pose.
        camera_model = PerspectiveCamera.from_intrinsics(element.intrinsics, pose_local_camera)

        # Todo 3.3: Undistort image using the camera model. Why should we undistort the image?
        # Undistort the original image, instead of using it directly.
        undistorted_img = camera_model.undistort_image(element.image)

        # Todo 4: Project a geographic world point into the images
        # Todo 4.1: Finish PerspectiveCamera._compute_camera_projection_matrix() below.
        # Todo 4.2: Finish PerspectiveCamera.project_world_point() below.
        # Project world point (the origin) into the image.
        pix_pos = camera_model.project_world_point(np.zeros([3, 1]))

        # Draw a marker in the image at the projected position.
        cv2.drawMarker(undistorted_img, np.squeeze(pix_pos.astype(np.int32)), (0., 0., 255.), cv2.MARKER_CROSS, 40, 3)

        # Todo 4.3: Add the camera frustum to the 3D viewer.
        # Add the camera frustum with the image to the 3D viewer.
        viewer_3d.add_camera_frustum(camera_model, undistorted_img)

        # Show the image.
        cv2.imshow(window_name, undistorted_img)

        # Update the windows.
        viewer_3d.update()
        cv2.waitKey(10)

    # Remove image viewer.
    cv2.destroyWindow(window_name)

    # Run 3D viewer until stopped (press 'q').
    viewer_3d.show()


def to_SO3(attitude: Attitude):
    """Convert to SO3 representation."""

    # Todo 1.1: Compute an SO3 representation of the orientation by composing principal rotations.
    return SO3.rot_z(attitude.z_rot) @ SO3.rot_y(attitude.y_rot) @ SO3.rot_x(attitude.x_rot)


def to_vector(position: CartesianPosition):
    """Convert to column vector representation."""

    # Todo 2.1: Return Cartesian position as a column vector.
    return np.array([[position.x, position.y, position.z]]).T


class PerspectiveCamera:
    """Camera model for the perspective camera"""

    def __init__(self,
                 calibration_matrix: np.ndarray,
                 distortion_coeffs: np.ndarray,
                 pose_world_camera: SE3):
        """Constructs the camera model.

        :param calibration_matrix: The intrinsic calibration matrix.
        :param distortion_coeffs: Distortion coefficients on the form [k1, k2, p1, p2, k3].
        :param pose_world_camera: The pose of the camera in the world coordinate system.
        """
        self._calibration_matrix = calibration_matrix
        self._calibration_matrix_inv = np.linalg.inv(calibration_matrix)
        self._distortion_coeffs = distortion_coeffs
        self._pose_world_camera = pose_world_camera
        self._camera_projection_matrix = self._compute_camera_projection_matrix()

    @classmethod
    def from_intrinsics(cls, intrinsics: Intrinsics, pose_world_camera: SE3):
        """Construct a PerspectiveCamera from an Intrinsics object

        :param intrinsics: The camera model intrinsics
        :param pose_world_camera: The pose of the camera in the world coordinate system.
        """

        # Todo 3.1: Construct the calibration matrix correctly.
        calibration_matrix = np.array([
            [intrinsics.fu, intrinsics.s,   intrinsics.cu],
            [0.,            intrinsics.fv,  intrinsics.cv],
            [0.,            0.,             1.]
        ])

        # Todo 3.2: Construct the distortion coefficients on the form [k1, k2, 0, 0, k3].
        distortion_coeffs = np.array([intrinsics.k1, intrinsics.k2, 0., 0., intrinsics.k3])

        return cls(calibration_matrix, distortion_coeffs, pose_world_camera)

    def _compute_camera_projection_matrix(self):
        # Todo 4.1: Compute camera projection matrix.
        return self._calibration_matrix @ self._pose_world_camera.inverse().to_matrix()[:3, :]

    def project_world_point(self, point_world):
        """Projects a world point into pixel coordinates.

        :param point_world: A 3D point in world coordinates.
        """

        if point_world.ndim == 1:
            # Convert to column vector.
            point_world = point_world[:, np.newaxis]

        # Todo 4.2: Implement projection using camera_projection_matrix_.
        return hnormalized(self._camera_projection_matrix @ homogeneous(point_world))

    def undistort_image(self, distorted_image):
        """Undistorts an image corresponding to the camera model.

        :param distorted_image: The original, distorted image.
        :returns: The undistorted image.
        """

        return cv2.undistort(distorted_image, self._calibration_matrix, self._distortion_coeffs)

    def pixel_to_normalised(self, point_pixel):
        """Transform a pixel coordinate to normalised coordinates

        :param point_pixel: The 2D point in the image given in pixels.
        """

        if point_pixel.ndim == 1:
            # Convert to column vector.
            point_pixel = point_pixel[:, np.newaxis]

        return self._calibration_matrix_inv @ homogeneous(point_pixel)

    @property
    def pose_world_camera(self):
        """The pose of the camera in world coordinates."""
        return self._pose_world_camera

    @property
    def calibration_matrix(self):
        """The intrinsic calibration matrix K."""
        return self._calibration_matrix

    @property
    def calibration_matrix_inv(self):
        """The inverse calibration matrix K^{-1}."""
        return self._calibration_matrix_inv

    @property
    def distortion_coeffs(self):
        """The distortion coefficients on the form [k1, k2, p1, p2, k3]."""
        return self._distortion_coeffs

    @property
    def projection_matrix(self):
        """The projection matrix P."""
        return self._camera_projection_matrix


if __name__ == "__main__":
    run_camera_pose_solution()
