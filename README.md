# Minesweeping Robot
Robotics 2020-2021 Project
Minesweeping Robot - Group 8

_________________________________________________________

Henrique Rodrigues - Henrique.Rodrigues@UGent.be

Flor Sanders - Flor.Sanders@UGent.be

Lander Verloove - Lander.Verloove@UGent.be

Lauren Van De Ginste - Lauren.VanDeGinste@UGent.be

------------------------------------------------------------------------------------

This project was created in the context of an assignment for the Robotics course (E019370) taught by prof. Tony Belpaeme and prof. Alain Sarlette at Ghent University during the academic year of 2020-2021. 

The code found in this repository was developed in order to explore and assess the performance of different mine sweeping strategies making use of a Turtlebot3 burger prototyping robot. The paper that serves as a project report can be found in the `Report` folder.

## Installation

- [Install ROS noetic](http://wiki.ros.org/noetic/Installation).

- Clone this repository.

  ```bash
  $ git clone https://github.ugent.be/robotics-2021/Group-8.git
  ```

- Change directory into this project folder.

  ```bash
  $ cd Group-8
  ```

- (Optional) Create a virtual Python environment ([virtualenv](https://virtualenv.pypa.io/en/latest/)) for this project.

  ```bash
  $ mkdir ~/env
  $ virtualenv -p python3 ~/env/minesweeping_robot
  $ source virtualenv -p python3 ~/env/minesweeping_robot
  ```

- Install the required python packages.

  ```bash
  $ pip install jupyter notebook matplotlib numpy pyyaml rospkg pandas
  ```

- Make sure all the dependencies for the ROS packages are installed.

  ```bash
  $ rosdep install --from-paths src
  ```

- Add the following to your `.bashrc` file and source it.

  ```bash
  source /opt/ros/noetic/setup.bash
  source <path to project directory>/devel/setup.bash
  export TURTLEBOT3_MODEL=burger
  export GAZEBO_MODEL_PATH=<path to project directory>/src/minesweeper_package/gazebo_models:${GAZEBO_MODEL_PATH}
  export ROBOTICS_PROJECT_DIR="<path to project directory>"
  ```

  ```bash
  $ source ~/.bashrc
  ```

- Build the source.

  ```bash
  $ catkin_make
  ```

## Usage

### Running simulations

- Change directory into this project folder.

- (Optional) Activate the virtual python environment.

  ```bash
  source virtualenv -p python3 ~/env/minesweeping_robot
  ```

- Build the source.

  ```bash
  $ catkin_make
  ```

- Start the roscore service.

  ```bash
  $ roscore
  ```

- Launch one of the available simulation worlds.

  ```bash
  $ roslaunch turtlebot3_gazebo <launchfile.launch>
  ```

  Options:

  - `0_BasicMinefield.launch`
  - `1_BasicMinefield_Small.launch`
  - `2_EnhancedMinefield_Test.launch`
  - `3_EnhancedMinefield_Small.launch`
  - `4_EnhancedMinefield_Medium.launch`
  - `5_EnhancedMinefield_Large.launch`

- Launch one of the available sweeping strategies.

  - Basic strategies

    ```bash
    $ roslaunch minesweeper_package <launchfile.launch>
    ```

    Options:

    - `random_roomba.launch`
    - `square_spiral.launch`
    - `worm.launch`

  - Smart strategies

    ```bash
    $ roslaunch turtlebot3_slam turtlebot3_slam.launch open_rviz:=false
    $ roslaunch turtlebot3_navigation turtlebot3_navigation.launch
    $ roslaunch minesweeper_package <launchfile.launch>
    ```

    Options:

    - `smart_random.launch`
    - `smart_near.launch`
    - `smart_far.launch`

- Enjoy!

### Collecting statistics

- Change directory into this project folder.

- (Optional) Activate the virtual python environment.

  ```bash
  source virtualenv -p python3 ~/env/minesweeping_robot
  ```

- Build the source.

  ```bash
  $ catkin_make
  ```

- Start the roscore service.

  ```bash
  $ roscore
  ```

- Run the automaton script.

  ```bash
  $ rosrun minesweeper_package automaton.py <strategy> <environment> --repetitions <number of runs> --duration <simulation time duration>
  ```

  Strategy options:

  - `random_roomba`
  - `square_spiral`
  - `worm`
  - `smart_random`
  - `smart_near`
  - `smart_far`

  Environment options:

  - `0_BasicMinefield`
  - `1_BasicMinefield_Small`
  - `2_EnhancedMinefield_Test`
  - `3_EnhancedMinefield_Small`
  - `4_EnhancedMinefield_Medium`
  - `5_EnhancedMinefield_Large`

  Note: For older hardware it might be required to change the delay between node launches and simulations. This can be done using the `--delay <delay>` flag.

- Let the script run until the last simulation is shut down.

- Collect the log files created during the runs from `src/minesweeper_package/log`.

- Visualize the results (see below).

### Visualizing statistics

- Robot paths

  - Leave the log files in the `src/minesweeper_package/log` folder.

  - Copy the file name ending on `robot_pos.txt` for which run you want to visualize the robot path.

  - Run the robot_path script. If you add `-- mines` the detonated mines will be plotted too.

    ```bash
    $ python3 robot_path.py <world number> <file name>
    ```

    The script will produce a line-connected scatter plot of the robot positions over the duration of the simulation along with a visualization of the walls and obstacles in the selected world.

- Performance statistics

  - Place the log files for all of your simulations in the right subfolder of the `Simulations` folder.

  - Change directory into the `Simulations` folder.

  - Run the analyse script.

    ```bash
    $ python3 analyse.py
    ```

    Follow the CLI instructions to make selections about which data needs to be visualized. Two sets of boxplots will be generated: one giving the interdetonation time for a given configuration and the other shows the time until 40% of the mines have been detonated.

## Useful resources

Of course this project would not have been possible with the amazing tools and resources that are freely available on the web. Below we give some of the ones that proved most useful to us.

- Official ROS tutorials: http://wiki.ros.org/ROS/Tutorials
- Turtlebot tutorials: https://learn.turtlebot.com/
- Gazebo & roslaunch tutorial: http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros
- Gazebo colors and textures tutorial: http://gazebosim.org/tutorials?tut=color_model
- Gazebo populations tutorial: http://gazebosim.org/tutorials?tut=model_population&cat=build_world
- Launch turtlebot3 simulations with ros: https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
- ROS action client tutrial: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
- ROS navigation stack tutorials: https://www.youtube.com/watch?v=5nZc5iSr5is

## Copyright notice

This code is released under an open-source AGPL-3.0 license.
When making use of this codebase, please take the relevant conditions into account.
More information can be found in the `LICENSE` file.

Furthermore, we stand on the shoulders of giants, namely the Turtlebot3 community who built some of the packages that are used in this code and provided them for free under the Apache-2.0 license. The relevant repositories can be found in the links below:

- https://github.com/ROBOTIS-GIT/turtlebot3
- https://github.com/ROBOTIS-GIT/turtlebot3_simulations
- https://github.com/ROBOTIS-GIT/turtlebot3_msgs
