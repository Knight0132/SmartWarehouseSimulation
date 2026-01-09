# SmartWarehouseSimulation: A Multi-Agent Robotic Warehouse System

## Overview

SmartWarehouseSimulation is a high-fidelity simulation environment developed in Unity 3D, designed to facilitate research and development in multi-agent pathfinding (MAPF), dynamic task allocation, and warehouse logistics optimization. This project provides a robust platform for testing and validating algorithms related to autonomous mobile robots (AMRs) in complex indoor environments.

Key research areas supported by this platform include:

- **Multi-Agent Path Finding (MAPF)**: Evaluating collision avoidance and path efficiency in dense robot swarms.
- **Dynamic Task Allocation**: real-time assignment of fetching and delivery tasks to heterogeneous agents.
- **Traffic Management**: Implementation of traffic rules and congestion control mechanisms in grid-based layouts.

## Features

- **Scalable Agent Simulation**: Supports the concurrent operation of multiple AMR agents with configurable kinematic properties.
- **Dynamic Environment Layer**: Utilizes a dynamic occupancy layer for real-time obstacle tracking and path re-planning.
- **Graph-Based Navigation**: Implements topological graph generation from grid maps for efficient route planning using A\* and DWA (Dynamic Window Approach).
- **Centralized Control Architecture**: Features a central controller for order management, robot dispatching, and system status monitoring.
- **Visualization Tools**: Includes debug visualization for path trajectories, occupancy grids, and intersection locks.

## System Architecture

The project is structured around a centralized control loop that coordinates:

1.  **Map Loader**: Parses static map data (JSON/IndoorGML) to construct the navigation graph.
2.  **Order Manager**: Generates and assigns tasks based on configurable strategies (e.g., Random, K-Means clustering).
3.  **Path Planner**: Computes optimal paths while accounting for static and dynamic obstacles.
4.  **Robot Agents**: Execute movement primitives and report state telemetry to the central controller.

## Installation

### Prerequisites

- **Unity**: Version 2021.3 LTS or later recommended.
- **NetTopologySuite**: Used for geometric operations (included in Dependencies).

### Getting Started

1.  Clone the repository:
    ```bash
    git clone https://github.com/Knight0132/SmartWarehouseSimulation.git
    ```
2.  Open the project in Unity Hub.
3.  Navigate to `Assets/Scenes` and open the main simulation scene.
4.  Press **Play** to start the simulation.

## Usage

- **Configuration**: Adjust simulation parameters (Robot Count, Order Frequency, etc.) via the `CentralController` GameObject in the Inspector.
- **Map Data**: Custom maps can be imported by placing compatible JSON files in the `StreamingAssets` directory and updating the `MapLoader` configuration.

## Contributing

We welcome contributions from the research community. Please adhere to the following guidelines:

1.  Fork the repository.
2.  Create a feature branch (`git checkout -b feature/NewAlgorithm`).
3.  Commit your changes with descriptive messages.
4.  Push to the branch and modify a Pull Request.

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

## Contact

For inquiries regarding the simulation platform or collaboration opportunities, please open an issue in the repository.
