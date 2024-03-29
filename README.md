# patrolling_sim
This began as a fork of [David Portugal's patrolling_sim](https://github.com/davidbsp/patrolling_sim), but has evolved into a rewrite of the original. This version makes use of the [Grex Machina multi-agent framework](https://github.com/NU-IDEAS-Lab/grex) and ROS 2, allowing for far more capabilities than the original patrolling_sim.

## Papers
This repository is used in multiple papers. See below for the revisions associated with specific submissions:

 * A. Goeckner, Y. Sui, N. Martinet, X. Li, and Q. Zhu, “Graph Neural Network-based Multi-agent Reinforcement Learning for Resilient Distributed Coordination of Multi-Robot Systems,” under review.

   * Commit 357e9a47064d67a6b6f66f41f73424933f86e345

 * A. Goeckner, X. Li, E. Wei, and Q. Zhu, “Attrition-Aware Adaptation for Multi-Agent Patrolling.” arXiv, Jan. 12, 2024. doi: 10.48550/arXiv.2304.01386.

   * Commit 36e474b734c9682798e0f0260ba9fda632c3b6ca


# Prerequisites
First install all prerequisites of the [Grex Machina](https://github.com/NU-IDEAS-Lab/grex) repository.

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
    colcon build
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
