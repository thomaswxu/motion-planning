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
        arm_dimensions_file_name: Name of the JSON configuration file containing arm dimensions used during planning.

        path_pose_index: Used to track which pose along the path to draw.
        arm_points: List holding all the arm joint XYZ points for each pose along the path.
        link_points: List holding XYZ points that trace the surface of each arm link, for visualization purposes.
        obstacles: List holding obstacle XYZ points read from a given configuration file.
        link_radius_mm: Radius of the arm links used during planning, in millimeters.

        figure: Plotting figure to be used for visualization.
        ax: Axes object corresponding to plotting figure.
        forward_key: Which key to use to draw the next pose along the path.
        back_key: Which key to use to draw the previous pose along the path.

        arm_point_color: What color to draw the arm points and the lines connecting them.
        link_color: What color to draw the link shapes.
        link_alpha: What transparency value to use for drawing the link shapes.
        obstacle_color: What color to draw obstacles.
    """

    def __init__(self, path_file_name: str, obstacles_file_name: str, arm_dimensions_file_name: str):
        """Initializes the visualizer instance using data from passed in files.
        
        Args:
            path_file_name: Which path data text file to visualize.
            obstacles_file_name: Obstacle configuration JSON file corresponding to the path data file.
            arm_dimensions_file_name: Arm dimensions configuration JSON file corresponding to the path data file.
        """
        self.path_file_name = path_file_name
        self.obstacles_file_name = obstacles_file_name
        self.arm_dimensions_file_name = arm_dimensions_file_name

        self.path_pose_index = 0
        self.arm_points = []
        self.link_points = []
        self.obstacles = []
        self.link_radius_mm = 0
        self.read_path_file()
        self.read_obstacles_file()
        self.read_arm_dimensions_file()

        self.figure = plt.figure()
        self.ax = self.figure.add_subplot(projection='3d')
        self.forward_key = 'j'
        self.back_key = 'k'

        self.arm_point_color = '#11b81e'
        self.link_color = '#11b81e'
        self.link_alpha = 0.2
        self.obstacle_color = '#fc4903'

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
        self.ax.plot(arm_points[:, 0], arm_points[:, 1], arm_points[:, 2], color=self.arm_point_color, marker='o')

    def plot_pose_links(self, arm_points: np.array) -> None:
        """Plot the arm link shapes for a single pose along the path."""
        # Plot cylinders around each link
        for i in range(0, len(arm_points) - 1):
            start_pt = arm_points[i]
            end_pt = arm_points[i + 1]
            link_vector = end_pt - start_pt
            link_length = np.sqrt(link_vector[0]**2 + link_vector[1]**2 + link_vector[2]**2)
            link_unit_vector = self.normalize_vector(link_vector)

            additional_vector = np.array([0, 0, 1]) # For generating vectors perpendicular to link
            if (link_unit_vector == additional_vector).all():
                additional_vector = np.array([0, 1, 0])
            
            perpendicular_vector1 = self.normalize_vector(np.cross(link_unit_vector, additional_vector))
            perpendicular_vector2 = self.normalize_vector(np.cross(link_unit_vector, perpendicular_vector1))

            cylinder_discretization_steps = 5
            cylinder_length_range = np.linspace(0, link_length, cylinder_discretization_steps)
            cylinder_angle_range = np.linspace(0, 2 * np.pi, cylinder_discretization_steps)
            cylinder_mesh_lengths, cylinder_mesh_angles = np.meshgrid(cylinder_length_range, cylinder_angle_range)
            cylinder_X, cylinder_Y, cylinder_Z = [
                start_pt[i] + link_unit_vector[i] * cylinder_mesh_lengths
              + self.link_radius_mm * np.sin(cylinder_mesh_angles) * perpendicular_vector1[i]
              + self.link_radius_mm * np.cos(cylinder_mesh_angles) * perpendicular_vector2[i] for i in [0, 1, 2]]
            self.ax.plot_surface(cylinder_X, cylinder_Y, cylinder_Z, color=self.link_color, alpha=self.link_alpha)

        # Plot spheres around each joint
        for arm_point in arm_points:
            sphere_discretization_steps = 15j
            u, v = np.mgrid[0:2*np.pi:sphere_discretization_steps, 0:2*np.pi:sphere_discretization_steps]
            sphere_X = arm_point[0] + self.link_radius_mm * np.sin(u) * np.cos(v)
            sphere_Y = arm_point[1] + self.link_radius_mm * np.sin(u) * np.sin(v)
            sphere_Z = arm_point[2] + self.link_radius_mm * np.cos(u)
            self.ax.plot_wireframe(sphere_X, sphere_Y, sphere_Z, color=self.link_color, alpha=self.link_alpha / 3)

    def plot_pose(self) -> None:
        """Plot a single pose along the path, along with the corresponding environment."""
        self.ax.set_title('Motion Plan Visualization\n(' + self.forward_key + " = step forward, "
            + self.back_key + "= step back)")
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        self.plot_pose_arm_points(self.arm_points[self.path_pose_index])
        self.plot_pose_links(self.arm_points[self.path_pose_index])
        self.set_axes_equal()

    def plot_obstacles(self) -> None:
        for obstacle in self.obstacles:
            obstacle_pts = obstacle.get_plottable_points()
            self.ax.plot(
                obstacle_pts[:, 0], obstacle_pts[:, 1], obstacle_pts[:, 2], color=self.obstacle_color, marker='.')

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

    def read_arm_dimensions_file(self) -> None:
        """Read stored arm dimension data from the given configuration file."""
        with open(self.arm_dimensions_file_name) as adf:
            dimension_data = json.load(adf)
            self.link_radius_mm = dimension_data['links_mm']['L_radius']

    def normalize_vector(self, vec: np.array) -> np.array:
        """Normalize a given vector. Assumes 3-dimensional."""
        vec_norm = np.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
        return vec / vec_norm

def parse_cmdline_arguments(cmdline_args: List[str]) -> argparse.ArgumentParser:
    argparser = argparse.ArgumentParser(
        prog="Path Visualizer",
        description="Generate an interactive visualization of a saved motion plan."
    )
    argparser.add_argument(
        "-p",
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
    argparser.add_argument(
        "-d",
        "--arm_dimensions_file",
        required=True,
        action="store",
        help="File containing the arm dimensions corresponding to the saved motion plan."
    )
    return argparser.parse_args(cmdline_args)

def main():
    cmdline_args = parse_cmdline_arguments(sys.argv[1:])
    print("Given path file: " + cmdline_args.path_file)
    print("Given obstacle file: " + cmdline_args.obstacle_file)
    print("Given arm dimensions file: " + cmdline_args.arm_dimensions_file)
    path_visualizer = PathVisualizer(
        cmdline_args.path_file, cmdline_args.obstacle_file, cmdline_args.arm_dimensions_file)
    path_visualizer.set_up_plot()
    return 0

if __name__ == "__main__":
    main()