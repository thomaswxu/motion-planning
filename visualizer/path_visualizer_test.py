#1/usr/bin/env python3
import pytest
from path_visualizer import PathVisualizer

def test_file_reading():
    path_file = "../saved_path.txt"
    obstacles_file = "../config/obstacles.json"
    path_visualizer = PathVisualizer(path_file, obstacles_file)
    assert(path_visualizer.path_file_name == path_file)
    assert(path_visualizer.obstacles_file_name == obstacles_file)
