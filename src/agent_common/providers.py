from __future__ import division  # force floating point division when using plain /
import rospy
import sys

from behaviour_components.sensors import Sensor
from mapc_ros_bridge.msg import Position
from agent_common.agent_utils import relative_euclidean_distance
import numpy as np
# import matplotlib.animation as animation
# import matplotlib.pyplot as plt

class PerceptionProvider(object):
    """
    Objects that holds current perception, provides additional reasoning/postprocessing, and gives easy access to RHBP
    sensors
    """

    def __init__(self):

        self.goals = []

        self.dispensers = []

        self.obstacles = []

        self.blocks = []

        self.closest_dispenser = None

        self.closest_block = None 

        self.closest_block_distance_sensor = Sensor(name="closest_block_distance", initial_value=sys.maxint)

        self.closest_dispenser_distance_sensor = Sensor(name="closest_dispenser_distance", initial_value=sys.maxint)

        # Here, we use the most basic Sensor class of RHBP and update manually within the provider to avoid the usage of
        # additional topics and their overhead.
        self.dispenser_visible_sensor = Sensor(name="dispenser_visible", initial_value=False)

        self.obstacle_sensor = Sensor(name="obstacle_visible",initial_value=0)
        self.number_of_blocks_sensor = Sensor(name="number_of_blocks", initial_value=0)  # TODO this is currently never updated

        self.local_map = np.zeros((11,11))

        self.agent = []
        self.agent_location = Position(5,5) #Agent initial agent position to center of grid (vision range 5)
        #TODO Entity sensor


    def update_perception(self, request_action_msg):
        """
        Has to be called on every simulation step and updated with the current request action message
        :param request_action_msg: request action message
        """

        self._request_action_msg = request_action_msg

        self._update_dispensers(request_action_msg)

        self._update_blocks(request_action_msg)

        self.goals = request_action_msg.goals  # TODO this could be more sophisticated and potentially extracted like above

        self._update_obstacles(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        self.agent = request_action_msg.agent

        self.map_status()


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

        self.number_of_blocks_sensor.update(newValue=len(self.blocks) > 0)
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
        
        x, y = self.local_map.shape
        if last_direction == "n":
            if self.agent_location.y <= 5:
                temp = [0 for i in range(y)]
                self.local_map = np.insert(self.local_map,0,temp,axis=0)
                self.agent_location.y += 1
        
        elif last_direction == "s":
            if x - self.agent_location.y <= 5:
                temp = [0 for i in range(y)]
                self.local_map = np.insert(self.local_map,x,temp,axis=0)
        
        elif last_direction == "e":
            if y - self.agent_location.x <= 5:
                temp = [0 for i in range(x)]
                self.local_map = np.insert(self.local_map,y,temp,axis=1)

        elif last_direction == "w":
            if self.agent_location.x <= 5:
                temp = [0 for i in range(x)]
                self.local_map = np.insert(self.local_map,0,temp,axis=1)
                self.agent_location.x += 1 


    def _update_map(self):
        
        for obstacle in self.obstacles:
            # print("obstacle  x: {} y: {}".format(obstacle.pos.x,obstacle.pos.y))
            self.local_map[obstacle.pos.y + self.agent_location.y][obstacle.pos.x +  self.agent_location.x] = 1
        
        for dispenser in self.dispensers:
            self.local_map[dispenser.pos.y +  self.agent_location.y][dispenser.pos.x +  self.agent_location.x] = 2
            # print("dispenser x: {} y: {}".format(dispenser.pos.x,dispenser.pos.y))

        # for block in self.blocks:
        #     self.local_map[block.pos.y +  self.agent_location.y][block.pos.x + self.agent_location.x] = 3

        for goal in self.goals:
            self.local_map[goal.pos.y + self.agent_location.y][goal.pos.x + self.agent_location.x] = 3

        self.local_map[self.agent_location.y][self.agent_location.x] = 5


    
    
    
    
    def map_status(self):

        '''
        Update the local map (11 x 11) grid with agent at center. 
        Assumed vision range : 5

        x positive - right   (x of agent is y of map)
        y positive - up

        0 - empty
        1 - obstacle
        2 - dispenser
        3 - block
        4 - goal
        5 - entity
        9 - agent

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
            
            elif last_direction == 's':
                self.agent_location.y += 1

            elif last_direction == 'e':
                self.agent_location.x += 1

            elif last_direction == 'w':
                self.agent_location.x -= 1
            
            self.check_vision_range(last_direction)
        
        else:
            last_direction = "failed"


        
        self._update_map()
        
        
        print("action: {} status: {} direction: {}, shape: {}".format(self.agent.last_action, self.agent.last_action_result, last_direction,self.local_map.shape))
        print("\n")
        self.local_map.dump('/ros/map.npy')
        # print(self.local_map)
        # M = self.local_map * 10
        # plt.pcolor( M , cmap = 'jet' )
        # plt.savefig('./a.png')
        print("\n\n")
