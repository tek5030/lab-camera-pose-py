# Step 1: Get an overview
We will as usual start by presenting an overview of the method and the contents of this project.

## Holmenkollen dataset
In the ["data" directory](../data), we are given a dataset of 110 images taken from a handheld camera inside a helicopter.

For each image, we also have the following data:
- The intrinsic camera calibration.
- Navigation data: The pose of the helicopter (which we will call "body") in [geographical coordinates](https://en.wikipedia.org/wiki/Geographic_coordinate_system) (latitude, longitude and altitude).
- Camera pose measurements: The pose of the camera relative to the helicopter.

We have provided a data reader that we will use to extract these data.

## Lab overview
Our job today is to represent and visualise these camera poses in a common coordinate system. 
We will then project the geographical coordinate of a light pole into the images, so that we can find its pixel position in each image.

The main steps in today's lab are:
- Represent the geographical body poses in a common local Cartesian coordinate system.
- Compute the corresponding camera poses in the same coordinate system.
- Undistort the images using the supplied calibration parameters.
- Project a geographic world point into the images, using the perspective camera model.

## Introduction to the project source files
We have chosen to distribute the code on the following files:

- [**lab_camera_pose.py**](../lab_camera_pose.py)
  
  Contains the main loop of the program and all exercises. 
  Your task will be to finish the code in this module. 
  
- [**common_lab_utils.py**](../common_lab_utils.py)

  This module contains utility functions and classes that we will use both in the lab and in the solution.
  Please take a quick look through the code.
  The [PyGeodesy](https://mrjean1.github.io/PyGeodesy/) package is used to represent geographical coordinates in a local Cartesian coordinate system.

- [**dataset.py**](../dataset.py)

  This module contains functionality for reading the dataset.

- [**viewer_3d.py**](../viewer_3d.py)

  This module implements a 3D visualisation tool for the lab based on [PyVista](https://docs.pyvista.org/)
  
- [**solution_camera_pose.py**](../solution_camera_pose.py)

  This is our proposed solution to the lab.
  Please try to solve the lab with help from others instead of just jumping straight to the solution ;)


Please continue to the [next step](2-from-geographical-coordinates-to-pixels.md) to get started!

