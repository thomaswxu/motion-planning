# motion-planning
This repository holds code for various motion and path planning explorations, with a focus on robot arms.

## Robot Arm Motion Planning

Most robot arms have 6 degrees of freedom (DOF). This makes path and motion planning more difficult to visualize than for 2D or 3D cases, but the principles and applicable algorithms are the same.

In this repository, A* and the Probabilistic Roadmap (PRM) method along with some of their variants are used.

The code in this repository assumes a generic 6 DOF robot arm (arm model determines dimensions, joint angle limits, etc. during planning).

## Setup

### Environment

It is recommended to use a Docker container by following the steps below. It may work outside of the container depending on your system, but no guarantees.

1. Install `docker` and `docker-compose`
    - e.g. in Arch-based distros: `yay -S docker docker-compose`
    - Be sure to `enable` and `start` the Docker daemon
        - e.g. `systemctl enable docker.service && systemctl start docker.service`
    - Note: depending on your install, either `docker compose` or `docker-compose` may work for the following commands.
2. Build the Docker image:
    - `sudo docker build -f Dockerfile -t motion-planning:latest .`
3. Start the container:
    - Create new container and start: `sudo docker compose -f docker-compose.yaml up -d`
    - Start existing container: `sudo docker compose -f docker-compose.yaml start`
4. Enter the container:
    - `sudo docker compose exec motion-planning bash`

When finished, either `stop` the container (which will allow it to be reused later with `start`) or `down` the container, which will stop and remove the container:
- `sudo docker compose -f docker-compose.yaml stop`
- `sudo docker compose -f docker-compose.yaml down`

### Arm Parameters

The code may be used out of the box with the provided [example arm configuration file](/config/arm_dimensions.json). However, you may wish to use dimensions/parameters specific to your own arm. If so, it's recommended to duplicate the example file and modify it to suit your purposes instead of editing it directly, which will probably make some unit tests fail.

Additionally, note that if you significantly change the arm dimensions you will probably want to modify the self-collision checking function in the [Map](./map/map.hpp) class.

### Obstacles

The guidance in [Arm Parameters](#arm-parameters) applies. Recommended to duplicate the [example obstacles configuration file](./config/obstacles.json) instead of modifying it, or else some unit tests may fail.

## Usage

1. If not already inside, enter Docker container (see steps [above](#set-up-environment)).
2. (Skip if done already) Configure the project with CMake:
    0. Navigate to `motion-planning` directory.
    1. `mkdir build`
    2. `cd build`
    3. `cmake ..`
3. Build the project:
    0. Navigate to the `build` directory.
    1. `cmake --build .`
4. TODO

### Visualize Results

1. TODO

## Unit Tests

Various unit tests are provided for the libraries in this repository. They currently use the Catch2 framework for C++ code.

To run the unit tests:
  1. Compile the code (e.g. by following the steps [above](#run-an-example).)
  2. Navigate to the `build` directory.
  3. Run any of the unit test executables, e.g. `Vec3Test`.

## Code Format/Style

The code mainly follows the Google code style guidelines. Max characters per line are limited to 120 instead of 80.

## License

Feel free to use or modify the code in this repository as you wish (MIT license).