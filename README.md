# Python Robotics Docker Project

This project is a Dockerized Python application that utilizes the Robotics Toolbox by Peter Corke for creating and visualizing robotic models in 3D.

## Project Structure

```
python-robotics-docker-project
├── src
│   └── main.py          # Main entry point of the application
├── Dockerfile            # Dockerfile for building the Docker image
├── docker-compose.yml    # Docker Compose configuration
├── Makefile              # Makefile for managing Docker containers
├── requirements.txt      # Python dependencies
└── README.md             # Project documentation
```

## Getting Started

To get started with this project, follow the instructions below.

### Prerequisites

Make sure you have Docker and Docker Compose installed on your machine.

### Pre-commit
On your python environment, run:
- `pip install pre-commit isort flake8 black`

Run this on the project root:
- `pre-commit install`

Now every time you `git commit`, `isort, black and flake8` will run.

To manually check files, run:
- `pre-commit run --all-files`

### Building the Docker Image

You can build the Docker image using the following command:

```bash
make build
```

### Running the Application

To run the application, use the following command:

```bash
make run
```
This will start the Docker container.

### Inside the container:
```
python -m roboticstoolbox.tools.np  # sanity-check RTP
python demo.py                      # talk to CoppeliaSim
```

### Stopping the Application

To stop the running application, use:

```bash
make stop
```

### Cleaning Up

To remove the Docker containers and images, run:

```bash
make clean
```

## Functionality

The application allows users to create and visualize robotic models in 3D using the Robotics Toolbox. You can modify the `src/main.py` file to implement different robotic models and visualizations.

## License

This project is licensed under the MIT License. See the LICENSE file for details.