# motion-planning
This repository holds code for various motion and path planning explorations, with a focus on robot arms.

## Robot Arm Motion Planning

Most robot arms have 6 degrees of freedom (DOF). This makes path and motion planning more difficult to visualize than for 2D or 3D cases, but the principles and applicable algorithms are the same.

In this repository, A* and the Probabilistic Roadmap (PRM) method along with some of their variants are used.

The code in this repository assumes a generic 6 DOF robot arm (arm model determines dimensions, joint angle limits, etc. during planning).

### Set Up Environment

1. Install `docker` and `docker-compose`
    - e.g. in Arch-based distros: `yay -S docker docker-compose`
    - Be sure to `enable` and `start` the Docker daemon
        - e.g. `systemctl enable docker.service && systemctl start docker.service`
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

### Run an Example

1. TODO

### Visualize Results

1. TODO

## Unit Tests

Various unit tests are provided for the libraries in this repository. They currently use the Catch2 framework for C++ code.

## Note on Code Format
The code mainly follows the Google code style guidelines. Max characters per line are limited to 120.

## License

Feel free to use or modify the code in this repository as you wish (MIT license).