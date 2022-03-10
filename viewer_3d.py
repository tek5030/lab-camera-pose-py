import numpy as np
import pyvista as pv
from pylie import SE3


class Viewer3D:

    def __init__(self):
        self._plotter = pv.Plotter()

        # Add scene origin and plane
        scene_plane = pv.Plane(i_size=1000, j_size=1000)
        self._plotter.add_mesh(scene_plane, show_edges=True, style='wireframe')
        self._add_axis(SE3(), 100)

        def print_camera_callback():
            print(f"Position: {self._plotter.camera.position}")
            print(f"Focal point: {self._plotter.camera.focal_point}")
            print(f"Up: {self._plotter.camera.up}")
            print(f"Zoom: {self._plotter.camera.zoom}")
        self._plotter.add_key_event('p', print_camera_callback)

        # Set camera.
        self._plotter.camera.position = (100, 1500, -500)
        self._plotter.camera.up = (-0.042739, -0.226979, -0.972961)
        self._plotter.camera.focal_point = (100, 300, -200)

        self._plotter.show(title="3D visualization", interactive_update=True)

    def add_body_axes(self, pose_local_body: SE3):
        self._add_axis(pose_local_body)

    def add_camera_axes(self, pose_local_camera: SE3):
        self._add_axis(pose_local_camera)

    def add_camera_frustum(self, camera_model, image):
        self._add_frustum(camera_model, image)

    def _add_axis(self, pose: SE3, scale=10.0):
        T = pose.to_matrix()

        point = pv.Sphere(radius=0.1*scale)
        point.transform(T)
        self._plotter.add_mesh(point)

        x_arrow = pv.Arrow(direction=(1.0, 0.0, 0.0), scale=scale)
        x_arrow.transform(T)
        self._plotter.add_mesh(x_arrow, color='red')

        y_arrow = pv.Arrow(direction=(0.0, 1.0, 0.0), scale=scale)
        y_arrow.transform(T)
        self._plotter.add_mesh(y_arrow, color='green')

        z_arrow = pv.Arrow(direction=(0.0, 0.0, 1.0), scale=scale)
        z_arrow.transform(T)
        self._plotter.add_mesh(z_arrow, color='blue')

    def _add_frustum(self, camera_model, image, scale=20.0):
        T = camera_model.pose_world_camera.to_matrix() @ np.diag([scale, scale, scale, 1.0])

        img_height, img_width = image.shape[:2]

        point_bottom_left = np.squeeze(camera_model.pixel_to_normalised(np.array([img_width-1., img_height-1.])))
        point_bottom_right = np.squeeze(camera_model.pixel_to_normalised(np.array([0., img_height-1.])))
        point_top_left = np.squeeze(camera_model.pixel_to_normalised(np.array([0., 0.])))
        point_top_right = np.squeeze(camera_model.pixel_to_normalised(np.array([img_width-1., 0.])))

        point_focal = np.zeros([3])

        pyramid = pv.Pyramid([point_bottom_left, point_bottom_right, point_top_left, point_top_right, point_focal])
        pyramid.transform(T)

        rectangle = pv.Rectangle([point_bottom_left, point_bottom_right, point_top_left, point_top_right])
        rectangle.texture_map_to_plane(inplace=True)
        rectangle.transform(T)

        image_flipped_rgb = image[::-1, :, ::-1].copy()
        tex = pv.numpy_to_texture(image_flipped_rgb)

        self._plotter.add_mesh(pyramid, show_edges=True, style='wireframe')
        self._plotter.add_mesh(rectangle, texture=tex, opacity=0.9)

    def update(self, time=500):
        self._plotter.update(time)

    def show(self):
        self._plotter.show()
