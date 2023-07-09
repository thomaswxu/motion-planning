"""Class for making a visualizable obstacle.
"""
from dataclasses import dataclass
import json
import numpy as np

@dataclass
class Obstacle:
    """Class for creating a visualizable obstacle.
    
    Currently assumes obstacles are axis-aligned bounding boxes (AABBs).
    """

    name: str
    min_x: float = 0
    max_x: float = 0
    min_y: float = 0
    max_y: float = 0
    min_z: float = 0
    max_z: float = 0

    def get_plottable_points(self):
        """Get the points that when plotted form the shape of the obstacle."""
        p1 = [self.min_x, self.min_y, self.min_z]
        p2 = [self.min_x, self.min_y, self.max_z]
        p3 = [self.max_x, self.min_y, self.max_z]
        p4 = [self.max_x, self.min_y, self.min_z]
        p5 = [self.min_x, self.max_y, self.min_z]
        p6 = [self.min_x, self.max_y, self.max_z]
        p7 = [self.max_x, self.max_y, self.max_z]
        p8 = [self.max_x, self.max_y, self.min_z]
        return np.array([p1, p2, p3, p4, p1, p5, p6, p2, p6, p7, p3, p7, p8, p4, p8, p5])

    @classmethod
    def read_from_json(cls, json_file: str, obstacle_name_in_file: str):
        with open(json_file) as f:
            obstacle_data = json.load(f)
            min_x = obstacle_data[obstacle_name_in_file]['x_mm']['min']
            max_x = obstacle_data[obstacle_name_in_file]['x_mm']['max']
            min_y = obstacle_data[obstacle_name_in_file]['y_mm']['min']
            max_y = obstacle_data[obstacle_name_in_file]['y_mm']['max']
            min_z = obstacle_data[obstacle_name_in_file]['z_mm']['min']
            max_z = obstacle_data[obstacle_name_in_file]['z_mm']['max']
            return cls(obstacle_name_in_file, min_x, max_x, min_y, max_y, min_z, max_z)