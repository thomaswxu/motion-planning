#!/usr/bin/env python3
"""File used to visualize motion planning results.
"""
import argparse
import json
from obstacle import Obstacle
import matplotlib.pyplot as plt
import numpy as np
import sys
from typing import List

class PathVisualizer:
    """Class containing various functions used to generate an interactive plot for a saved motion plan.

    Attributes:
        path_file_name: Name of the text file storing the path data.
        obstacles_file_name: Name of the JSON configuration file storing data of the obstacles present during planning.

        path_pose_index: Used to track which pose along the path to draw.
        arm_points: List holding all the arm joint XYZ points for each pose along the path.
        link_points: List holding XYZ points that trace the surface of each arm link, for visualization purposes.
        obstacles: List holding obstacle XYZ points read from a given configuration file.

        figure: Plotting figure to be used for visualization.
        ax: Axes object corresponding to plotting figure.
        forward_key: Which key to use to draw the next pose along the path.
        back_key: Which key to use to draw the previous pose along the path.
    """

    def __init__(self, path_file_name: str, obstacles_file_name: str):
        """Initializes the visualizer instance using data from passed in files.
        
        Args:
            path_file_name: Which path data text file to visualize.
            obstacles_file_name: Obstacle configuration JSON file corresponding to the path data file.
        """
        self.path_file_name = path_file_name
        self.obstacles_file_name = obstacles_file_name

        self.path_pose_index = 0
        self.arm_points = []
        self.link_points = []
        self.obstacles = []
        self.read_path_file()
        self.read_obstacles_file()

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(projection='3d')
        self.forward_key = 'j'
        self.back_key = 'k'

    def set_up_plot(self) -> None:
        """Set up the plot/figure to be used for visualization."""

        # Set up keyboard controls
        self.figure.canvas.mpl_connect('key_press_event', self.update_plot_on_key_press)

        # Plot initial state
        self.plot_pose()
        self.plot_obstacles()
        plt.show()

    def set_axes_equal(self) -> None:
        """Ensure each plot axis has the same scale (so shapes aren't stretched out).
        
        Credit: https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z
        """
        single_axis_limits_mm = (-1000, 1000)
        all_axis_limits_mm = np.array([single_axis_limits_mm, single_axis_limits_mm, single_axis_limits_mm])
        origin = np.mean(all_axis_limits_mm, axis=1)
        radius = 0.5 * np.max(np.abs(all_axis_limits_mm[:, 1] - all_axis_limits_mm[:, 0]))

        x, y, z = origin
        self.ax.set_xlim3d([x - radius, x + radius])
        self.ax.set_ylim3d([y - radius, y + radius])
        self.ax.set_zlim3d([z - radius, z + radius])

    def plot_pose_arm_points(self, arm_points: np.array) -> None:
        """Plot the arm points for a single pose along the path."""
        self.ax.plot(arm_points[:, 0], arm_points[:, 1], arm_points[:, 2], color='#11b81e', marker='o')

    def plot_pose(self) -> None:
        """Plot a single pose along the path, along with the corresponding environment."""
        self.ax.set_title('Motion Plan Visualization\n(' + self.forward_key + " = step forward, "
            + self.back_key + "= step back)")
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        self.plot_pose_arm_points(self.arm_points[self.path_pose_index])
        self.set_axes_equal()

    def plot_obstacles(self) -> None:
        for obstacle in self.obstacles:
            obstacle_pts = obstacle.get_plottable_points()
            self.ax.plot(obstacle_pts[:, 0], obstacle_pts[:, 1], obstacle_pts[:, 2], color='#fc4903', marker='.')

    def update_plot_on_key_press(self, event) -> None:
        """Update the plot to correspond to different poses along the path.
        
        Args:
            event: A key-press event.
        """
        if event.key == self.forward_key:
            self.path_pose_index += 1
        elif event.key == self.back_key:
            self.path_pose_index -= 1
        
        # Prevent drawn pose from going out of path range
        if self.path_pose_index >= len(self.arm_points):
            self.path_pose_index = len(self.arm_points) - 1
        elif self.path_pose_index < 0:
            self.path_pose_index = 0
        sys.stdout.flush()

        self.ax.cla()
        self.plot_pose()
        self.plot_obstacles()
        self.figure.canvas.draw()

    def read_path_file(self) -> None:
        """Read stored path data from the given text file."""
        self.arm_points = []
        self.link_points = []

        with open(self.path_file_name) as pf:
            reading_arm_points = True

            points = []
            for line in pf:
                l = line.strip()
                if l == "[":
                    points = []
                elif l == "]":
                    if reading_arm_points:
                        self.arm_points.append(np.array(points))
                    else:
                        self.link_points.append(np.array(points))
                elif "arm_points" in l:
                    reading_arm_points = True
                elif "link_points" in l:
                    reading_arm_points = False
                else:
                    l_list = [float(i) for i in l.split(', ')]
                    points.append(np.array(l_list))

    def read_obstacles_file(self) -> None:
        """Read stored obstacle data from the given configuration file."""
        with open(self.obstacles_file_name) as of:
            obstacles_data = json.load(of)
            for obstacle_name in obstacles_data:
                if obstacle_name == 'metadata':
                    continue
                self.obstacles.append(Obstacle.read_from_json(self.obstacles_file_name, obstacle_name))

def parse_cmdline_arguments(cmdline_args: List[str]) -> argparse.ArgumentParser:
    argparser = argparse.ArgumentParser(
        prog="Path Visualizer",
        description="Generate an interactive visualization of a saved motion plan."
    )
    argparser.add_argument(
        "-f",
        "--path_file",
        required=True,
        action="store",
        help="File containing the saved motion plan data."
    )
    argparser.add_argument(
        "-o",
        "--obstacle_file",
        required=True,
        action="store",
        help="File containing the obstacle data corresponding to the saved motion plan."
    )
    return argparser.parse_args(cmdline_args)

def main():
    command_line_args = parse_cmdline_arguments(sys.argv[1:])
    print("Given path file: " + command_line_args.path_file)
    print("Given obstacle file: " + command_line_args.obstacle_file)
    path_visualizer = PathVisualizer(command_line_args.path_file, command_line_args.obstacle_file)
    path_visualizer.set_up_plot()
    return 0

if __name__ == "__main__":
    main()