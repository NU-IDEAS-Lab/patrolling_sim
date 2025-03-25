# patrolling_sim
This began as a fork of [David Portugal's patrolling_sim](https://github.com/davidbsp/patrolling_sim), but has evolved into a rewrite of the original. This version makes use of the [Grex Machina multi-agent framework](https://github.com/NU-IDEAS-Lab/grex) and ROS 2, allowing for far more capabilities than the original patrolling_sim.

## Papers
This repository is used in multiple papers. See below for the revisions associated with specific submissions:

 * A. Goeckner, Y. Sui, N. Martinet, X. Li, and Q. Zhu, [“Graph Neural Network-based Multi-agent Reinforcement Learning for Resilient Distributed Coordination of Multi-Robot Systems,”](https://doi.org/10.1109/IROS58592.2024.10802510) IROS 2024.

   * Commit: 357e9a47064d67a6b6f66f41f73424933f86e345
   * Citation:
      ```
      @INPROCEEDINGS{10802510,
      author={Goeckner, Anthony and Sui, Yueyuan and Martinet, Nicolas and Li, Xinliang and Zhu, Qi},
      booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
      title={Graph Neural Network-based Multi-agent Reinforcement Learning for Resilient Distributed Coordination of Multi-Robot Systems}, 
      year={2024},
      volume={},
      number={},
      pages={5732-5739},
      doi={10.1109/IROS58592.2024.10802510}}
      ```

 * A. Goeckner, X. Li, E. Wei, and Q. Zhu, [“Attrition-Aware Adaptation for Multi-Agent Patrolling.”](https://doi.org/10.1109/LRA.2024.3421793) IEEE Robotics and Automation Letters, 2024.

   * Commit 36e474b734c9682798e0f0260ba9fda632c3b6ca
   * Citation:
      ```
      @ARTICLE{10582411,
      author={Goeckner, Anthony and Li, Xinliang and Wei, Ermin and Zhu, Qi},
      journal={IEEE Robotics and Automation Letters}, 
      title={Attrition-Aware Adaptation for Multi-Agent Patrolling}, 
      year={2024},
      volume={9},
      number={8},
      pages={7230-7237},
      doi={10.1109/LRA.2024.3421793}}
      ```


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
