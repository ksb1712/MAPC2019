from __future__ import division  # force floating point division when using plain /
import rospy
import sys

from behaviour_components.sensors import Sensor
from mapc_ros_bridge.msg import Position
from agent_common.agent_utils import relative_euclidean_distance
import numpy as np
from std_msgs.msg import String
import itertools
# import matplotlib.animation as animation
# import matplotlib.pyplot as plt

class PerceptionProvider(object):
    """
    Objects that holds current perception, provides additional reasoning/postprocessing, and gives easy access to RHBP
    sensors
    """

    def __init__(self):

        self.goals = []

        self.relative_goals = []

        self.dispensers = []

        self.obstacles = []

        self.blocks = []

        self.closest_dispenser = None

        self.closest_block = None 

        self.perception_range = 5

        self.number_of_blocks_sensor = Sensor(name="number_of_blocks", initial_value=0)  
        self.closest_block_distance_sensor = Sensor(name="closest_block_distance", initial_value=sys.maxint)

        self.closest_dispenser_distance_sensor = Sensor(name="closest_dispenser_distance", initial_value=sys.maxint)
        self.dispenser_visible_sensor = Sensor(name="dispenser_visible", initial_value=False)
       
        self.obstacle_sensor = Sensor(name="obstacle_visible",initial_value=0)

        self.closest_goal = None
        self.goal_visible_sensor = Sensor(name="goal_visible",initial_value=False)
        self.count_goal_cells = Sensor(name="count_goal_cells",initial_value=0)
        self.closest_goal_distance_sensor = Sensor(name="closest_goal_distance_sensor",initial_value=sys.maxint)
        
        self.local_map = np.ones((self.perception_range*2 + 1,self.perception_range*2 + 1)) * -1

        self.goal_origin = None
        self.origin_found = False

        self.agent = None
        self.agent_location = Position(self.perception_range,self.perception_range) #Agent initial agent position to center of grid (vision range 5)



    def update_perception(self, request_action_msg):
        """
        Has to be called on every simulation step and updated with the current request action message
        :param request_action_msg: request action message
        """

        self._request_action_msg = request_action_msg

        self.agent = request_action_msg.agent


        self._update_dispensers(request_action_msg)

        self._update_blocks(request_action_msg)

        self._update_goal_cell(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        self._update_obstacles(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above


        self.map_status()

    def get_goal_origin(self):
        
        if(len(self.relative_goals) >= 12):
            self.origin_found = True
            x_min = sys.maxint
            y_min = sys.maxint

            for g in self.relative_goals:
                if g.x < x_min:
                    x_min = g.x
                if g.y < y_min:
                    y_min = g.y

            print("{} Origin found at : x: {}, y: {} relative to agent_origin".format(self.agent.name,x_min + 1,y_min + 1))
            
            self.goal_origin = Position(x_min + 1, y_min + 1)

    def _update_goal_cell(self,request_action_msg):
        """
        Update self.goals
        Update count_goal_cells - sensor to indicate how many goal cells have
        been discovered
        """

        self.goals = request_action_msg.goals

        self.goal_visible_sensor.update(newValue=len(self.goals) > 0)
        self.count_goal_cells.update(newValue=len(self.relative_goals))

        self.count_goal_cells.sync()
        self.goal_visible_sensor.sync()



        if self.origin_found:
            pub = rospy.Publisher('origin_loc', String, queue_size=10)
            pub.publish(String("{},{},{}".format(self.agent.name,self.goal_origin.x,self.goal_origin.y)))
        

        if (self.origin_found == False):
            print("{} says False".format(self.agent.name))
            if(len(self.relative_goals) >= 12):
                print("{} says Located".format(self.agent.name))
                self.get_goal_origin()
            else:
                print("{} says not yet".format(self.agent.name))
                self._update_closest_goal_cell(goals=self.goals)
        



    
    def _update_closest_goal_cell(self, goals):
        
        self.closest_goal = None
        closest_distance = sys.maxint

        for g in goals:
            if self.closest_goal is None or closest_distance > relative_euclidean_distance(g.pos):
                self.closest_goal = g
                closest_distance = relative_euclidean_distance(g.pos)

        self.closest_goal_distance_sensor.update(newValue=closest_distance)
        self.closest_goal_distance_sensor.sync()



    def _update_dispensers(self, request_action_msg):
        """
        Update dispenser perception
        :param request_action_msg: full current request action message object
        """

        self.dispensers = request_action_msg.dispensers
        

        self.dispenser_visible_sensor.update(newValue=len(self.dispensers) > 0)
        self.dispenser_visible_sensor.sync()

        self._update_closest_dispenser(dispensers=self.dispensers)

    def _update_closest_dispenser(self, dispensers):
        """
        Update information about the closest visible dispenser
        :param dispensers: dispensers perception
        """

        self.closest_dispenser = None
        closest_distance = sys.maxint

        for d in dispensers:
            if self.closest_dispenser is None or closest_distance > relative_euclidean_distance(d.pos):
                self.closest_dispenser = d
                closest_distance = relative_euclidean_distance(d.pos)

        self.closest_dispenser_distance_sensor.update(newValue=closest_distance)
        self.closest_dispenser_distance_sensor.sync()

    def _update_blocks(self, request_action_msg):
        """
        Update block perception
        :param request_action_msg: full current request action message object
        """

        self.blocks = request_action_msg.blocks

        self.number_of_blocks_sensor.update(newValue=len(self.blocks))
        self.number_of_blocks_sensor.sync()

        
        self._update_closest_block(blocks=self.blocks)


        # if len(self.blocks) > 0:
        #     print("Blocks: {}".format(len(self.blocks))) 

    def _update_closest_block(self, blocks):
        """
        Update information about the closest visible block
        :param blocks: blocks perception
        """

        self.closest_block = None
        closest_distance = sys.maxint

        for b in blocks:
            if self.closest_block is None or closest_distance > relative_euclidean_distance(b.pos):
                self.closest_block = b
                closest_distance = relative_euclidean_distance(b.pos)

        self.closest_block_distance_sensor.update(newValue=closest_distance)
        self.closest_block_distance_sensor.sync()


    def _update_obstacles(self, request_action_msg):
        """
        Update obstacle perception
        :param request_action_msg: full current request action message object
        """

        self.obstacles = request_action_msg.obstacles

        self.obstacle_sensor.update(newValue=len(self.obstacles) > 0)
        self.obstacle_sensor.sync()

    def check_vision_range(self,last_direction):

        #TODO set insert values to -1
        #Set all cell in current 5 x 5 to 0
        # Update map then


        x, y = self.local_map.shape
        if last_direction == "n":
            if self.agent_location.y <= self.perception_range:
                temp = [-1 for i in range(y)]
                self.local_map = np.insert(self.local_map,0,temp,axis=0)
                self.agent_location.y += 1
        
        elif last_direction == "s":
            if x - self.agent_location.y <= self.perception_range:
                temp = [-1 for i in range(y)]
                self.local_map = np.insert(self.local_map,x,temp,axis=0)
        
        elif last_direction == "e":
            if y - self.agent_location.x <= self.perception_range:
                temp = [-1 for i in range(x)]
                self.local_map = np.insert(self.local_map,y,temp,axis=1)

        elif last_direction == "w":
            if self.agent_location.x <= self.perception_range:
                temp = [-1 for i in range(x)]
                self.local_map = np.insert(self.local_map,0,temp,axis=1)
                self.agent_location.x += 1 


    def _update_map(self):
        

        
        x_min = max(self.agent_location.x - self.perception_range,0)
        y_min = max(self.agent_location.y - self.perception_range,0)

        x_max = x_min + self.perception_range * 2
        y_max = y_min + self.perception_range * 2
       
       #Assume all cells in 5 x 5 perception range as free
        for i,j in itertools.product(range(y_min,y_max),range(x_min,x_max)):
            if(abs(self.agent_location.y - i) + abs(self.agent_location.x -j) <= self.perception_range):
                self.local_map[i][j] = 0
        

        for obstacle in self.obstacles:
            # print("obstacle  x: {} y: {}".format(obstacle.pos.x,obstacle.pos.y))
            self.local_map[obstacle.pos.y + self.agent_location.y][obstacle.pos.x +  self.agent_location.x] = 1
        
        for dispenser in self.dispensers:
            self.local_map[dispenser.pos.y +  self.agent_location.y][dispenser.pos.x +  self.agent_location.x] = 2
            # print("dispenser x: {} y: {}".format(dispenser.pos.x,dispenser.pos.y))

        # for block in self.blocks:
        #     self.local_map[block.pos.y +  self.agent_location.y][block.pos.x + self.agent_location.x] = 3

        for goal in self.goals:
            x_loc = goal.pos.x + self.agent_location.x
            y_loc = goal.pos.y + self.agent_location.y
            self.local_map[y_loc][x_loc] = 3
            temp = Position(x_loc,y_loc)
            if temp not in self.relative_goals:
                self.relative_goals.append(temp)
        self.local_map[self.agent_location.y][self.agent_location.x] = 4


    #TODO
    #Use -1 in map for nav
    #Use obstacles for nav
    #Call astar for nav
    #Once goals discovered include other behaviours



    

    
    
    
    def map_status(self):

        '''
        Update the local map (11 x 11) grid with agent at center. 
        Assumed vision range : 5

        x positive - right   (x of agent is y of map)
        y positive - up

        -1 - unknown
        0 - empty
        1 - obstacle
        2 - dispenser
        # 3 - block
        3 - goal
        # 5 - entity
        4 - agent

        todo:
        code based on dispenser type

        n - go up - agent.y decreases
        e - go left - agent.x increases

        '''

        """
        Identify the last successful move direction and change the map accordingly
        """
        last_direction = []
        if self.agent.last_action == "move":
            if self.agent.last_action_result == "success":
                last_direction = str(self.agent.last_action_params[0])
                # print("Move to " + last_direction)

        #Change location
        #Get percept range and modify map accordingly
        #Change location if needed
        #Update map

        if len(last_direction) > 0:
            self.local_map[self.agent_location.y][self.agent_location.x] = 0

            if last_direction == 'n':
                self.agent_location.y -= 1
                if(self.origin_found):
                    self.goal_origin.y += 1
            
            elif last_direction == 's':
                self.agent_location.y += 1

            elif last_direction == 'e':
                self.agent_location.x += 1

            elif last_direction == 'w':
                self.agent_location.x -= 1
                if(self.origin_found):
                    self.goal_origin.x += 1
            
            self.check_vision_range(last_direction)
        
        else:
            last_direction = "failed"

      

        
        self._update_map()
        
        
        # print("action: {} status: {} direction: {}, shape: {}".format(self.agent.last_action, self.agent.last_action_result, last_direction,self.local_map.shape))
        # print("\n")
        self.local_map.dump('/ros/map/{}_map.npy'.format(self.agent.name))
        # data = self.local_map.ravel()
        # pub = rospy.Publisher('map_a1', numpy_msg(int32[]),queue_size=10)
        # rospy.init_node('talker', anonymous=True)
        # r = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     pub.publish(x)
        #     r.sleep()

        print(self.agent.last_action, self.agent.last_action_result)
        # # print(self.local_map)
        # # M = self.local_map * 10
        # # plt.pcolor( M , cmap = 'jet' )
        # # plt.savefig('./a.png')
        # print("\n\n")
