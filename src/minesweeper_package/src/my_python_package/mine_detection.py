#! /usr/bin/env pyhon
import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel, GetWorldProperties
from geometry_msgs.msg import Pose
from std_msgs.msg import Int16
from my_python_package.abstract_turtle import AbstractTurtle


class mine_detection(AbstractTurtle):
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

        #"""create topic and publisher"""

        self.hit_publisher = rospy.Publisher('cleared_mines', Int16, queue_size=2)

    def run(self):
        """ main loop of the program """
        self.count_mines()
        while not rospy.is_shutdown():
                self.scan_mines()

    def scan_mines(self):
        get_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        remove_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        robot_coords = get_coordinates("turtlebot3_burger", "").pose.position
        robot_pos = np.array([robot_coords.x, robot_coords.y, robot_coords.z])
        for mine in self.mines:
            mine_coords = get_coordinates(mine, "" ).pose.position
            mine_pos = np.array([mine_coords.x, mine_coords.y, mine_coords.z])
            if np.linalg.norm(robot_pos-mine_pos) < self._robot_radius + self._mine_radius:
                remove_model(mine)

                self.mines.remove(mine)
                self.cleared_mines += 1
                self.hit_publisher.publish(self.cleared_mines)
                print("number of mines cleared: {}".format(self.cleared_mines))

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
        world = get_world_properties().model_names
        mines = []
        for model_name in world:
            if model_name[:8] == "Landmine":
                mines.append(model_name)
        self.mines=mines
        print("simulation started with: {} mines".format(len(mines)))
    
    def get_n_remaining_mines(self):
        return len(self.mines)

def main():
    x = mine_detection()
    x.start_ros()
    x.run()