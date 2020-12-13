#! /usr/bin/env python
import rospy
import numpy as np
import os
import time
import threading
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from minesweeper_python.robot_controller import RobotController


class mine_detection(RobotController):
    def __init__(self):
        super(mine_detection, self).__init__()
        self.node_name = "mine_detection"
        self._robot_radius = 0.105 #radius of the robot, used to remove the mines
        self._mine_radius = self._robot_radius/np.sqrt(5) #radius of a mine, set such that Area_mine=0.2*Area_robot
        self._spawned_mines = 50 #number of mines spawned on initialization
        self.cleared_mines = 0
        self.world_height = 10 #height of the minefield (y-axis)
        self.world_width = 10 #width of the minefield (x-axis)
        self.margin = 0.5 #margin between the walls and the minefield
        self.mines = [] #call count_mines() to add all mines from the minefield

        #create topic and publisher
        self.hit_publisher = rospy.Publisher('cleared_mines', Int16, queue_size=2)

        #retrieve the path to the working directory of the project
        self.root = os.environ.get("ROBOTICS_PROJECT_DIR")
        self.root = os.path.expanduser(self.root)

        #retrieve local time 
        year, month, day, hour, minute, second = time.strftime("%Y,%m,%d,%H,%M,%S").split(',')
        self.time_prefix = str(day) + "-" + str(month) + "-" + str(year) + "_" +str(hour) + ":" + str(minute) + ":" + str(second)

        #log for the number of cleared mines
        file_name = self.time_prefix + "_mines"+ ".txt"
        self.log = open(os.path.join(self.root, "src/minesweeper_package/log/", file_name), "w")
        self.log.write("time;count;mine_x;mine_y;mine_z;robot_x;robot_y;robot_z\n")

        #log for the robot position
        file_name = self.time_prefix  + "_robot_pos"+ ".txt"
        self.log_pos = open(os.path.join(self.root, "src/minesweeper_package/log/", file_name), "w")
        self.log_pos.write("time;robot_x;robot_y\n")
        
        #retrieve the distribution of the mines in the simulation world
        self.count_mines()

        #open the model of the green green mines
        model = open(os.path.join(self.root + "src/minesweeper_package/gazebo_models/Landmine_0-018x0-005_Green/model.sdf"), "r")
        self.green_mine_model = model.read()
        model.close()


    def run(self):
        
        """ main loop of the program """
        self.initial_time = rospy.get_time()
        while not rospy.is_shutdown():
            self.scan_mines()
        self.log.close()
        self.log_pos.close()



    def scan_mines(self):
        get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        remove_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        robot_coords = get_coordinates("turtlebot3_burger", "").pose.position
        robot_pos = np.array([robot_coords.x, robot_coords.y, robot_coords.z])
        simulation_time=rospy.get_time()-self.initial_time
        for mine in self.mines:
            mine_pose = get_coordinates(mine, "" ).pose
            mine_coords = mine_pose.position
            mine_pos = np.array([mine_coords.x, mine_coords.y, mine_coords.z])
            if np.linalg.norm(robot_pos-mine_pos) < self._robot_radius + self._mine_radius:
                remove_model(mine)
                self.mines.remove(mine)
                self.spwan_green_mine(mine_pose)
                self.cleared_mines += 1
                self.hit_publisher.publish(self.cleared_mines)
                self.log.write(str(simulation_time) + ";" + str(self.cleared_mines) + ";" + str(mine_coords.x) + ";" + str(mine_coords.y) 
                     + ";" + str(mine_coords.z) + ";" + str(robot_coords.x) + ";" + str(robot_coords.y) + ";" + str(robot_coords.z) + "\n")
        self.log_pos.write(str(simulation_time) + ";" + str(robot_coords.x) + ";" + str(robot_coords.y) + "\n")
        time.sleep(0.1)
    
    def spwan_green_mine(self, pose):
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model("cleared_mine_" + str(self.cleared_mines), self.green_mine_model, "", pose, "")

    def spawn_mines(self):
        f = open('/home/lander/Desktop/Robotics/Group-8/src/minesweeper_package/gazebo_models/coke_can/model.sdf','r')
        model = f.read()
        pose = Pose()
        for i in range(self._spawned_mines):
            pos = np.random.rand(2)
            pose.position.x = (pos[0]-0.5)*(self.world_width-self.margin)
            pose.position.y = (pos[1]-0.5)*(self.world_height-self.margin)
            pose.position.z = -0.01
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            spawn_model("mine_"+str(i), model, "", pose, "")

    def count_mines(self):
        rospy.wait_for_service('/gazebo/get_world_properties')
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        rospy.wait_for_service('/gazebo/get_model_state')
        get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        world = get_world_properties().model_names
        self.mines = []
        file_name = self.time_prefix + "_distribution.txt"
        self.log_distr = open(os.path.join(self.root, "src/minesweeper_package/log/", file_name), "w")
        self.log_distr.write("mine_x;mine_y\n")
        for model_name in world:
            if model_name[:8] == "Landmine":
                self.mines.append(model_name)
                mine_pos = get_coordinates(model_name, "" ).pose.position
                self.log_distr.write(str(mine_pos.x) + ";" + str(mine_pos.y) + "\n")
        self.log_distr.close()
        print("simulation started with: {} mines".format(len(self.mines)))
    

    
    def get_n_remaining_mines(self):
        return len(self.mines)

def main():
    x = mine_detection()
    x.start_ros()
    x.run()