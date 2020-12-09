# Group-8
Robotics 2020-2021 Project: Minesweeping robot - Group 8
_________________________________________________________

Henrique Rodrigues - Henrique.Rodrigues@UGent.be

Flor Sanders - Flor.Sanders@UGent.be

Lander Verloove - Lander.Verloove@UGent.be

Lauren Van De Ginste - Lauren.VanDeGinste@UGent.be

------------------------------------------------------------------------------------

## Recommended Installation

- Install ROS noetic on ubuntu 20.04.

- Clone this repository in the `home` folder. `cd ~ && git clone https://github.ugent.be/robotics-2021/Group-8.git`.

- Add the following to your `~/.bashrc` file.

  ```bash
  source /opt/ros/noetic/setup.bash
  source ~/Group-8/devel/setup.bash
  export TURTLEBOT3_MODEL=burger
  export GAZEBO_MODEL_PATH=~/Group-8/src/minesweeper_package/gazebo_models:${GAZEBO_MODEL_PATH}
  export ROBOTICS_PROJECT_DIR="<path to project directory>"
  ```
  Note: the `export GAZEBO_MODEL_PATH=~$(ROBOTICS_PATH)/src/minesweeper_package/gazebo_models:${GAZEBO_MODEL_PATH}` command points towards the newly created Gazebo models and links them.

- Source bashrc. `source ~/.bashrc`.

- Change directory to this repository. `cd ~/Group-8/`.

- Build the source. `catkin_make`.

## General Usage
 1. Go to the working directory of the project
 1. execute `catkin_make && source devel/setup.bash && roscore`
 1. Bring up gazebo `roslaunch turtlebot3_gazebo <launchfile.launch>`
      * Launchfiles: 
        * 0_BasicMinefield.launch
        * 1_BasicMinefield_Small.launch
        * 2_EnhancedMinefield_Test.launch
        * 3_EnhancedMinefield_Small.launch
      * Problems with your graphics card? Execute: `export SVGA_VGPU10=0`
 1. Steer the robot: `roslaunch minesweeper_package <launchfile.launch>`
      * launchfiles: 
        * controller.py randomroomba
        * my_launch_file.launch
        
## Testing
- Launch a world in the gazebo simulator along with a turtlebot: E.g. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
- Launch rviz to visualize the readings of the turtlebot while operating: `roslaunch turtlebot3_fake turtlebot3_fake.launch`.
- Launch one of the available minesweeping strategies: E.g. `rosrun minesweeper_package controller.py randomroomba`.
  - Options: `squareloop/randomroomba`.
- Launch the scripts from lab session 3: `roslaunch minesweeper_package my_launch_file.launch`.

## Collecting statistics

Repeated simulations for a world and strategy of choice can be launched through: `rosrun minesweeper_package automaton.py [strategy] [environment]`.

More information for arguments and options can be obtained using `rosrun minesweeper_package automaton.py --help`, which prints:

```bash
automaton.py [-h] [--duration [DURATION]] [--repetitions [REPETITIONS]]
                    [--debug]
                    strategy environment

Automatically run simulations one after the other

positional arguments:
  strategy              Which minesweeping strategy to launch. Options:
                        [random_roomba/worm/square_spiral]
  environment           Which environment to launch in the gazebo simulator.
                        Options: [0_BasicMinefield/1_BasicMinefield_Small/2_En
                        hancedMinefield_Test/3_EnhancedMinefield_Small/4_Enhan
                        cedMinefield_Medium/5_EnhancedMinefield_Large]

optional arguments:
  -h, --help            show this help message and exit
  --duration [DURATION]
                        How long (in minutes) each simulation will run for (in
                        simulated time).
  --repetitions [REPETITIONS]
                        The total amount of times the simulation should be
                        repeated.
  --debug               Should debug messages be printed?

```

## Visualization of the statistics
The script robot_path can be used to visualize the trajectory of the robot and the distribution of the mines.
```
usage: robot_path.py [-h] [--duration DURATION] [--mines MINES]
                     world file_name

plots the trajectory of the robot

positional arguments:
  world                Number of the world used during the simulation,
                       options: [3/4/5]
  file_name            name of the file containing data for the robot
                       trajectory (located in the "log" folder)

optional arguments:
  -h, --help           show this help message and exit
  --duration DURATION  time in seconds of the simulation
  --mines MINES        Indicate the positions of the mines [True/False]
```

## Report

Overleaf report-link: https://www.overleaf.com/8928521917hdmphszkkkrj

## Useful resources

- Official ROS tutorials: http://wiki.ros.org/ROS/Tutorials
- Turtlebot tutorials: https://learn.turtlebot.com/
- Gazebo & roslaunch tutorial: http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros
- Gazebo colors and textures tutorial: http://gazebosim.org/tutorials?tut=color_model
- Gazebo populations tutorial: http://gazebosim.org/tutorials?tut=model_population&cat=build_world
- Launch turtlebot3 simulations with ros: https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
- ROS navigation stack tutorials: https://www.youtube.com/watch?v=5nZc5iSr5is
