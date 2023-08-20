# patrolling_sim
Fork of https://github.com/davidbsp/patrolling_sim

# Prerequisites
First install all prerequisites of the [mas_simulation](https://github.com/NU-IDEAS-Lab/mas_simulation) repository.

Then, perform the following:

 * Install Python dependencies.
   ```bash
   pip install networkx[default] numpy matplotlib zarr
   ```

# Installation
Once prerequisites are installed, perform the following:

 1) Use Git to download the patrolling sim repository.

    ```bash
    git clone --recurse https://github.com/NU-IDEAS-Lab/patrolling_sim.git
    ```
    
 2) Change to the `patrolling_sim` folder:

    ```bash
    cd ./patrolling_sim/
    ```

 3) Source the correct ROS 2 installation.
   
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    
 4) Build the patrolling sim using Colcon, the ROS 2 build system.

    ```bash
    colcon build --allow-overriding gazebo_ros
    ```

 5) Follow instructions below to run the simulation.
 
 # Operation Instructions
To operate the simulator, perform the following steps:

 1) Change to the `patrolling_sim` root directory:

    ```
    cd ./patrolling_sim/
    ```

 2) Source the installed packages, plus the Gazebo simulator, and set environment variables.
   
    ```
    source ./install/setup.bash
    source /usr/share/gazebo/setup.sh
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

 3) Run the simulation:

    ```
    ros2 launch patrolling_sim simulation.launch.py agent_count:=2
    ```

    * *Note: To view available launch arguments, add a* `-s` *flag to the end of the command.*
