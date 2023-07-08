"""File used to visualize motion planning results.
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from typing import List

# Parse command line arguments.
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
command_line_args = argparser.parse_args()

def set_axes_equal(ax: plt.Axes) -> None:
    """Ensure each plot axis has the same scale (so shapes aren't stretched out).
    
    Credit: https://stackoverflow.com/questions/13685386/how-to-set-the-equal-aspect-ratio-for-all-axes-x-y-z
    """
    single_axis_limits_mm = (-1000, 1000)
    all_axis_limits_mm = np.array([single_axis_limits_mm, single_axis_limits_mm, single_axis_limits_mm])
    origin = np.mean(all_axis_limits_mm, axis=1)
    radius = 0.5 * np.max(np.abs(all_axis_limits_mm[:, 1] - all_axis_limits_mm[:, 0]))

    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def read_path_file(path_file_name: str) -> List[List[np.array]]:
    """Read stored path data from a text file.

    Args:
        path_file_name: Name of the text file storing the path data.
    
    Returns:
        Two lists of XYZ points. The lists will be the same length, which is equal to the number of poses in the path.
        The first list holds all the arm joint XYZ points for each pose along the path.
        The second list holds XYZ points along the surface of each arm link, for visualization purposes.
    """
    arm_points = []
    link_points = []

    with open(path_file_name) as pf:
        reading_arm_points = True

        points = []
        for line in pf:
            l = line.strip()
            if l == "[":
                points = []
            elif l == "]":
                if reading_arm_points:
                    arm_points.append(np.array(points))
                else:
                    link_points.append(np.array(points))
            elif "arm_points" in l:
                reading_arm_points = True
            elif "link_points" in l:
                reading_arm_points = False
            else:
                l_list = [float(i) for i in l.split(', ')]
                points.append(np.array(l_list))
    return arm_points, link_points

def plot_path(path_file_name: str, obstacles_file_name: str) -> None:
    """Plot a stored planned path.

    Args:
        path_file_name: Name of the text file storing the path data.
        obstacles_file_name: Name of the configuration file storing data of the obstacles present during planning.
    """
    arm_points, link_points = read_path_file(path_file_name)
    print(len(arm_points))
    print(len(link_points))

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.plot(1, 2, 3, color="#0062ff", marker='o')

    set_axes_equal(ax)
    plt.show()

def main():
    print("Given path file: " + command_line_args.path_file)
    print("Given obstacle file: " + command_line_args.obstacle_file)
    plot_path(command_line_args.path_file, command_line_args.obstacle_file)
    return 0

if __name__ == "__main__":
    main()