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
  ```

- Source bashrc. `source ~/.bashrc`.

- Change directory to this repository. `cd ~/Group-8/`.

- Build the source. `catkin_make`.

## Usage

- Launch a world in the gazebo simulator along with a turtlebot: E.g. `roslaunch turtlebot3_gazebo turtlebot3_world.launch`.
- Launch rviz to visualize the readings of the turtlebot while operating: `roslaunch turtlebot3_fake turtlebot3_fake.launch`.
- Launch one of the available minesweeping strategies: E.g. `rosrun minesweeper_package controller.py randomroomba`.
  - Options: `squareloop/randomroomba`.
- Enjoy!
## Setup of The New World(s)
- Link the model folder
  ```
  echo  'export GAZEBO_MODEL_PATH=~$(ROBOTICS_PATH)/src/minesweeper_package/gazebo_models:${GAZEBO_MODEL_PATH}' >> ~/.bahsrc
  source ~/.bashrc
  ```
- Launch gazebo
  
  ```
  roscore
  roslaunch turtlebot3_gazebo minefield.launch #Lander
  OR
  roslaunch turtlebot3_gazebo turtlebot3_minesweeping.launch #Lauren
  ```
 
## Report

Overleaf report-link: https://www.overleaf.com/8928521917hdmphszkkkrj

## Useful resources

- Official ROS tutorials: http://wiki.ros.org/ROS/Tutorials
- Turtlebot tutorials: https://learn.turtlebot.com/
- Gazebo & roslaunch tutorial: http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros
- Launch turtlebot3 simulations with ros: https://automaticaddison.com/how-to-launch-the-turtlebot3-simulation-with-ros/
