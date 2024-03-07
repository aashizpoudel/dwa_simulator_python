# DWA Robot Simulation

This repository contains the files necessary for simulating a robot using the Dynamic Window Approach (DWA) algorithm.

## Files

1. `dwa_robot.py`: This file contains the Python code for simulating the robot. It implements the DWA algorithm and handles the robot's movement based on the provided waypoints.

2. `requirements.txt`: This file lists the Python libraries required to run the simulated robot. Make sure to install these dependencies before running the simulation.

3. `waypoints.txt`: This file contains the coordinates that the robot will traverse sequentially. You can edit this file to add or remove waypoints as per your requirements.

## Usage

1. Install the required dependencies by running the following command:

``` pip install -r requirements.txt ```

2. Edit the `waypoints.txt` file to specify the desired waypoints for the robot to navigate.

3. Run the simulation by executing the following command:

``` python dwa_robot.py ```

The robot will begin traversing the specified waypoints using the DWA algorithm. You can observe the robot's movement in the simulation environment.

## Contributing

Contributions to this project are welcome. If you find any issues or have suggestions for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).