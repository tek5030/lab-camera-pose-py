import os
import cv2
import numpy as np
from dataclasses import dataclass
from common_lab_utils import Attitude, CartesianPosition, GeodeticPosition, Intrinsics


@dataclass
class DataElement:
    """A set of data for each image in the dataset."""
    img_num: int
    image: np.ndarray
    intrinsics: Intrinsics
    camera_position_in_body: CartesianPosition
    camera_attitude_in_body: Attitude
    body_position_in_geo: GeodeticPosition
    body_attitude_in_geo: Attitude


class Dataset:
    """Represents the dataset for this lab."""

    _first_file_num = 502
    _last_file_num = 611

    _curr_file_num = _first_file_num

    def __init__(self, data_dir="data"):
        self._data_dir = data_dir

    def __iter__(self):
        self._curr_file_num = self._first_file_num
        return self

    def __next__(self):
        """Reads the next data element."""
        if self._curr_file_num > self._last_file_num:
            raise StopIteration

        image_filename = os.path.join(self._data_dir, f"110608_Oslo_0{self._curr_file_num}.jpg")

        metadata_filename = os.path.join(self._data_dir, f"110608_Oslo_0{self._curr_file_num}.txt")
        metadata = np.loadtxt(metadata_filename)

        next_data_element = DataElement(
            self._curr_file_num,
            cv2.imread(image_filename, cv2.IMREAD_UNCHANGED),
            Intrinsics(*metadata[:8]),
            CartesianPosition(*metadata[8:11]),
            Attitude(*metadata[11:14]),
            GeodeticPosition(*metadata[14:17]),
            Attitude(*metadata[17:])
        )

        self._curr_file_num += 1
        return next_data_element
