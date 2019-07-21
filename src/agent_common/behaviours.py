from __future__ import division  # force floating point division when using plain /
import rospy
import random
import time, sys

from behaviour_components.behaviours import BehaviourBase

from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from mapc_ros_bridge.msg import Position

from agent_common.agent_utils import get_bridge_topic_prefix, pos_to_direction

from agent_common.providers import PerceptionProvider

import numpy as np
import math
from agent_common.astar_path import *

def action_generic_simple(publisher, action_type, params=[]):
    """
    Generic helper function for publishing GenericAction msg
    :param publisher: publisher to use
    :param action_type: the type of the action msg
    :param params: optional parameter for the msg
    """
    action = GenericAction()
    action.action_type = action_type
    action.params = params
    publisher.publish(action)



        
class GenericActionBehaviour(BehaviourBase):
    """
    A simple behaviour for triggering generic MAPC actions that just need a action type and static parameters
    """

    def __init__(self, name, agent_name, action_type, params=[], **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param action_type: type of the MAPC action
        :param params: optional static parameters for the MAPC action
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(GenericActionBehaviour, self) \
            .__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._action_type = action_type
        self._params = params
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        rospy.logdebug(self._agent_name + "::" + self._name + " executing: " + self._action_type)
        action_generic_simple(publisher=self._pub_generic_action, action_type=self._action_type, params=self._params)


class AgentControl(BehaviourBase):
    """
    Control an agent through keyboard
    """
    def __init__(self, name, agent_name,perception_provider, **kwargs):

        super(AgentControl, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self._perception_provider = perception_provider


        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    
    def do_step(self):
        

        opt = int(raw_input("Enter action: "))
        action_type = GenericAction.ACTION_TYPE_MOVE
        if opt == 1:
            params = [KeyValue(key="direction", value="n")]
            action_type = GenericAction.ACTION_TYPE_MOVE
        elif opt == 2:
            params = [KeyValue(key="direction", value="s")]
            action_type = GenericAction.ACTION_TYPE_MOVE
        elif opt == 3:
            params = [KeyValue(key="direction", value="e")]
            action_type = GenericAction.ACTION_TYPE_MOVE
        elif opt == 4:
            params = [KeyValue(key="direction", value="w")]
            action_type = GenericAction.ACTION_TYPE_MOVE
        elif opt == 5:
            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)
            params = [KeyValue(key="direction", value=direction)]
            action_type=GenericAction.ACTION_TYPE_REQUEST
        elif opt == 6:
            direction = pos_to_direction(self._perception_provider.closest_block.pos)
            params = [KeyValue(key="direction", value=direction)]
            action_type=GenericAction.ACTION_TYPE_ATTACH
        elif opt == 7:
            direction = pos_to_direction(self._perception_provider.closest_block.pos)
            params = [KeyValue(key="direction", value=direction)]
            action_type=GenericAction.ACTION_TYPE_DETACH
        elif opt == 8:
            params = [KeyValue(key="direction", value="cw")]
            action_type=GenericAction.ACTION_TYPE_ROTATE

        elif opt == 9:
            params = [KeyValue(key="direction", value="ccw")]
            action_type=GenericAction.ACTION_TYPE_ROTATE

        print(params)
        action_generic_simple(publisher=self._pub_generic_action, action_type=action_type, params=params)
        time.sleep(0.2)
        sys.stdin.flush()        



class Dispense(BehaviourBase):
    """
    Dispenses a new block from the dispenser nearby
    """

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Dispense, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name
        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        
        # print("{} blocks dispensed: {}".format(self._agent_name,self._perception_provider.sensor_dispensed_blocks._value))
        direction = ''
        if self._perception_provider.sensor_dispensed_blocks._value < 1:
            print("{} Dispense called".format(self._agent_name))
            goal = self._perception_provider.goal_origin

            pos_x = self._perception_provider.target_dispenser_cell.x + goal.x
            pos_y = self._perception_provider.target_dispenser_cell.y + goal.y
            agent = self._perception_provider.agent_location
            if pos_x > agent.x:
                direction = 'e'
            elif pos_x < agent.x:
                direction = 'w'
            elif pos_y < agent.y:
                direction = 'n'
            elif pos_y > agent.y:
                direction = 's'
           
            params = [KeyValue(key="direction", value=direction)]
            print("{} Dispense direction {}".format(self._agent_name,direction))
            rospy.logdebug(self._agent_name + "::" + self._name + " request dispense " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_REQUEST,
                                    params=params)

        else:
            rospy.logerr("Behaviour:%s: no dispenser in range", self.name)


class MoveToDispenser(BehaviourBase):
    """
    Move to closest dispenser nearby
    """
    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(MoveToDispenser, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)
        # print("Explore behaviour _1")
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        # self.local_map = self.perception_provider.local_map
        self.path = []
        self.goal_location = self.perception_provider.goal_origin
        self.reached_loc = False
        self.target_cell = self.perception_provider.target_dispenser_cell

    
            
    
    def do_step(self):

        print("{} Move to D called".format(self._agent_name))
        # target = self.perception_provider.target_cell
        # goal = self.perception_provider.goal_origin

        if self.perception_provider.agent.last_action == "move":
                if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
                    print("{} Path planning failed".format(self._agent_name))
                    self.path = []
        direction = ''
        # target = Positon(self.target.x + self.goal_location.x,self.target.y + self.goal_location.y)
        if not self.reached_loc:
            if self.perception_provider.goal_origin != self.goal_location:
                self.goal_location = self.perception_provider.goal_origin
                self.path=[]
            agent_location = self.perception_provider.agent_location
            target = Position(self.target_cell.x + self.goal_location.x, self.target_cell.y + self.goal_location.y)
            # print("Target_org: {} \n Target_modified: {} \n agent: {} \n  Goal {}".format(self.target_cell, target,agent_location,self.goal_location))
            # print("Map shape: {}".format(self.perception_provider.local_map.shape))
        
            if abs(agent_location.x - target.x) + abs(agent_location.y - target.y) <= 1:
                self.reached_loc = True
                # print("{} has reached disp".format(self._agent_name))

            else:

                if len(self.path) == 0:
                    
                    self.path = get_astar_path(self.perception_provider.local_map,agent_location,target,full_path=False)
                    # print("Path: {}".format(self.path))
                    if len(self.path) > 0:
                        direction = self.path.pop(0)

                else:
                    direction = self.path.pop(0)

       

       


                
        params = [KeyValue(key="direction", value=direction)]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)










class Attach(BehaviourBase):
    """
    Attach a new block from the dispenser nearby
    """

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Attach, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        
        print("{} Attach called".format(self._agent_name))
        direction = ''
        if self.perception_provider.closest_block:
            
            goal = self.perception_provider.goal_origin

            pos_x = self.perception_provider.target_dispenser_cell.x + goal.x
            pos_y = self.perception_provider.target_dispenser_cell.y + goal.y
            agent = self.perception_provider.agent_location
            if pos_x > agent.x:
                direction = 'e'
            elif pos_x < agent.x:
                direction = 'w'
            elif pos_y < agent.y:
                direction = 'n'
            elif pos_y > agent.y:
                direction = 's'
            # direction = pos_to_direction(self._perception_provider.closest_block.pos)
            # random_move = ['n', 's', 'e', 'w']
            # direction = random.choice(random_move)
            params = [KeyValue(key="direction", value=direction)]

            rospy.logdebug(self._agent_name + "::" + self._name + " attach request " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ATTACH,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no BLOCK in range", self.name)








class Explore_astar(BehaviourBase):

    """
    Independent exploration behaviour 
    Objective: to find 12 goal cells
    exploration behaviour: 

    start : random move
    if obstacles: 

    """
    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Explore_astar, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)


        self._agent_name = agent_name
        self.perception_provider = perception_provider
        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
       
        self.goal_path = False
        self.path = []
        self.opp_direction_dict = {'n':'s','s':'n','e':'w','w':'e'}
        self.previous_direction = None
        self.history = []

    
    def mh_distance(self,a,b):
        x_dist = abs(a.x - b.x)
        y_dist = abs(a.y - b.y)

        return x_dist + y_dist
    # def check_map_increase(self,dir,agent_map):

    
    def unexplored_score(self,a,b,agent_map):
        H, W = agent_map.shape
        range_val = 4
        left = max(0,b.x - range_val)
        right = min(W-1,b.x + range_val)
        up = max(0,b.y - range_val)
        down = min(H-1,b.y + range_val)
        dist = self.mh_distance(a,b)
        # print("shape: {}, limits: {}".format(agent_map.shape,(left,right,up,down)))
        count = 0
        count_obs = 0
        for i in range(left,right):
            for j in range(up, down):
                if agent_map[j][i] == -1:
                    count += 1
                if agent_map[j][i] == 1:
                    count_obs += 1
        
        # print("obstacle: ",count_obs)
        # if count_obs > 3:
        #     count = 0
        return float(count) / (float(dist))


    def check_limits(self,limit,val):
        if val > 0 and val < limit:
            return True #Within limits
        
        return False
    def check_los_obstacle(self,agent_map,agent_location,x_sign = 0,y_sign = 0):

        H, W = agent_map.shape
        limit = 6
        for i in range(6):
            if (self.check_limits(W,agent_location.x + i*x_sign) and 
               self.check_limits(H,agent_location.y + i*y_sign)):
               if agent_map[agent_location.y + i*y_sign][agent_location.x + i*x_sign] == 1:
                   return True #Sees obstacle
        
        return False



    def check_obstacle_range(self,agent_map,agent_location,new_direction):

        if new_direction == 'e':
            if self.check_los_obstacle(agent_map,agent_location,x_sign = 1):
                return True #Obstacle seen
        
        if new_direction == 'w':
            if self.check_los_obstacle(agent_map,agent_location,x_sign = -1):
                return True
        
        if new_direction == 's':
            if self.check_los_obstacle(agent_map,agent_location,y_sign = 1):
                return True
        
        if new_direction == 'n':
            if self.check_los_obstacle(agent_map,agent_location,y_sign = -1):
                return True

        return False
    
    def get_new_path(self,agent_map,agent_location):

        u_Y, u_X = np.where(agent_map == 0)
        # print("count unexplored: ",len(u_X))

        list_unexplored = [Position(u_X[i],u_Y[i]) for i in range(len(u_X))]
        min_array = [self.unexplored_score(agent_location,cell,agent_map) for cell in list_unexplored]
        
        check = True
        path = []
        # while check:
        min_index = min_array.index(max(min_array))
        target_unexplored_cell = list_unexplored[min_index]
        # print("Score: {}".format(min_array[min_index]))
        while True:
            print("yeah {}".format(len(self.history)))
            if target_unexplored_cell not in self.history:
                path = get_astar_path(agent_map,agent_location,target_unexplored_cell,full_path=True)
                if len(path) > 0:
                    self.history.append(target_unexplored_cell)
                    return path
                else:
                    self.history.append(target_unexplored_cell)
                    min_array.pop(min_index)
                    list_unexplored.pop(min_index)
                
                    min_index = min_array.index(max(min_array))
                    target_unexplored_cell = list_unexplored[min_index]
            else:
                min_array.pop(min_index)
                list_unexplored.pop(min_index)
                
                min_index = min_array.index(max(min_array))
                target_unexplored_cell = list_unexplored[min_index]

                    
              


    def do_step(self):
        # random_move = ['n','e','w','s']
        direction = ''
        print("{} Explore_astar called".format(self._agent_name))
        # print("Target_selected_sensor: {}".format(self.perception_provider.target_dispenser_selected_sensor._value))
        if not self.perception_provider.target_disp_selected:

            # print("dispenser visible: {}".format(self.perception_provider.dispenser_visible_sensor._value))
            # print("#goal cells: {}".format(self.perception_provider.count_goal_cells._value))
            # print("#blocks: {}".format(self.perception_provider.sensor_dispensed_blocks._value))

            # self.perception_provider = 
            agent_map = self.perception_provider.local_map
            agent_location = self.perception_provider.agent_location
            # print("agent_loc: ({},{})".format(agent_location.x,agent_location.y))
            
            last_d = ''
            
            if self.perception_provider.agent.last_action == "move":
                if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:
                    path_available = False
                
                    last_d = self.perception_provider.agent.last_action_params
                    print("{} Path planning failed {}".format(self._agent_name,last_d))
                    self.path = []
                    self.goal_path = False
                
                elif self.perception_provider.agent.last_action_result == "success":
                    self.previous_direction = self.perception_provider.agent.last_action_params[0]
                    # print("hasKK:  ",self.opp_direction_dict[self.previous_direction])

            
            if self.path is None:
                self.path = []

            if len(self.path) > 0 and self.goal_path:
                direction = self.path.pop(0)
                
            else:

                g_Y, g_X = np.where(agent_map == 7)
                if len(g_X) > 0 and len(g_X) < 11:
                    goal_array = [Position(g_X[i],g_Y[i]) for i in range(len(g_X))]
                    dist_goal_array = [self.mh_distance(agent_location,cell) for cell in goal_array]
                    target_goal = goal_array[dist_goal_array.index(max(dist_goal_array))]
                    self.goal_path = True
                    self.path = get_astar_path(agent_map,agent_location,target_goal,full_path=False)
                    # print(self.path)
                    if len(self.path) > 0:
                        direction = self.path.pop(0)
                
                elif len(self.path) > 0:
                    direction = self.path.pop(0)
                    # if self.check_obstacle_range(agent_map,agent_location,direction):
                    # self.path = self.get_new_path(agent_map,agent_location)
                        # if self.path is None:
                        #     print("No unexplored cells left")
                        # else:
                        #     direction = self.path.pop(0)
                

                else:
                #Check if unexplored available
                    self.path = []
                    while len(self.path) < 1: 
                        self.path = self.get_new_path(agent_map,agent_location)
                        agent_map = self.perception_provider.local_map
                        agent_location = self.perception_provider.agent_location
                    # self.goal_path = False
                
                    direction = self.path.pop(0)
                    self.goal_path = False
            
           
        else:
            print("Still selected")
        
        params = [KeyValue(key="direction", value=direction)]
        # print(params)
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)
     
class MoveToGoal(BehaviourBase):
    """
    Move to goal area with block
    """
    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(MoveToGoal, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)
        # print("Explore behaviour _1")
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        # self.local_map = self.perception_provider.local_map
        self.path = []
        self.goal_location = None
        self.submit_origin = self.perception_provider.submit_origin
        self.block_location = None
        self.target_dispenser = None
        self.angle = 0

        self.reached_loc = False
        self.target_agent = None
        self.target_block = None

        self.index = 0
        # self.submit_array = self.perception_provider.target_submit_array
        # print("In goal: submit: {}",self.submit_location)

    
    # def get_target_agent(self,block):
    #     submit = Position(self.submit_location.x + self.goal_location.x, self.submit_location.y + self.goal_location.y)
    #     possible_locations = [Position(block.x + 1,block.y),Position(block.x - 1, block.y),
    #                           Position(block.x,block.y+1),Position(block.x,block.y - 1)]
    #     first_block = None
    #     for row in self.submit_array.dispensers:
    #         if row.pos.x + row.pos.y == 1:
    #             first_block = Position(self.goal_location.x + row.pos.x,self.goal_location.y + row.pos.y)
        
    #     if first_block is not None:
    #         for row in possible_locations:
    #             if row.x != first_block.x and row.y != first_block.y:
    #                 if self.perception_provider.local_map[row.y][row.x] in [0,7]:
    #                     print("Found target agent",row)
    #                     return row


        return None

    def math_ops(self,angle,sin=False,cos=False):
        if sin:
            if angle == 90:
                return 1
            elif angle == 270:
                return -1
            else:
                return 0
        if cos:
            if angle == 0:
                return 1
            elif angle == 180:
                return -1
            else:
                return 0


    
    def do_step(self):

        print("{} Move to G called".format(self._agent_name))

        direction = ''
        if self.perception_provider.agent.last_action == "move":
            if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
                print("{} Path planning failed".format(self._agent_name))
                self.path = []
                self.index = 0
                self.reached_loc = False

       

        if len(self.path) == 0 and not self.reached_loc:
            print("main")
            
            # target = self.perception_provider.target_cell
            task_submit = self.perception_provider.target_submit_cell
            # self.goal_location = self.perception_provider.goal_origin
            if self.submit_origin is None:
        #     print("None")
              self.submit_origin = self.perception_provider.submit_origin
              print("origin: {} \n target: {} \n goal: {}".format(self.submit_origin,task_submit,self.perception_provider.goal_origin))
            
            agent_location = self.perception_provider.agent_location
            # print("agent:{} ",agent_location)
            if self.block_location is None:
                self.target_dispenser = Position(self.perception_provider.target_dispenser_cell.x,self.perception_provider.target_dispenser_cell.y)
                self.goal_location = Position(self.perception_provider.goal_origin.x,self.perception_provider.goal_origin.y)
                self.block_location = Position(self.target_dispenser.x + self.goal_location.x, self.target_dispenser.y + self.goal_location.y)
                # if self.block_location.x > agent_location.x:
                #     self.angle = 0
                # elif self.block_location.x < agent_location.x:
                #     self.angle = 180
                # elif self.block_location.y > agent_location.y:
                #     self.angle = 90
                # else:
                #     self.angle = 180

            print("current: ({},{})".format(self.perception_provider.agent_location.x,self.perception_provider.agent_location.y))
            print("needed: ({},{})".format(self.goal_location.x + self.submit_origin.x,self.goal_location.y+self.submit_origin.y))
            target_agent = Position(self.goal_location.x + self.submit_origin.x,self.goal_location.y + self.submit_origin.y)
            target_block = Position(self.goal_location.x + self.submit_origin.x + task_submit.x,
                                    self.goal_location.y + self.submit_origin.y + task_submit.y)
            
            if self.perception_provider.agent.last_action == "move":
                if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
                    self.path = get_astar_path(self.perception_provider.local_map,agent_location,target_agent,full_path=True)
                else:
                    self.path = get_astar_path(self.perception_provider.local_map,agent_location,
                                       target_agent,self.block_location,target_block,flag=1,full_path=True)
            else:
                self.path = get_astar_path(self.perception_provider.local_map,agent_location,
                                       target_agent,self.block_location,target_block,flag=1,full_path=True)

            if len(self.path) > 0:
                print("Path: {}".format(self.path))
                direction = self.path[0]
            else:
                self.reached_loc = True
                self.perception_provider.is_submit_agent = True
        elif self.index >= len(self.path):
            print("out")
            print("current: ({},{})".format(self.perception_provider.agent_location.x,self.perception_provider.agent_location.y))
            print("needed: ({},{})".format(self.goal_location.x + self.submit_origin.x,self.goal_location.y+self.submit_origin.y))
            
            if (self.goal_location.x + self.submit_origin.x) == self.perception_provider.agent_location.x:
                if (self.goal_location.y + self.submit_origin.y) == self.perception_provider.agent_location.y:
                    print("yep")
                    self.reached_loc = True
                    self.perception_provider.is_submit_agent = True
            
            if not self.reached_loc:
                submit_cell = Position(self.goal_location.x + self.submit_origin.x,
                                        self.goal_location.y + self.submit_origin.y)
                self.path = []
                self.path = get_astar_path(self.perception_provider.local_map,
                                            self.perception_provider.agent_location,submit_cell,full_path=True)
                if len(self.path) > 0:
                    self.index = 0
                    print("######### Other path {}".format(self.path))
                    direction = self.path[0]
      

        elif not self.reached_loc:
            print("elif")
            print("current: ({},{})".format(self.perception_provider.agent_location.x,self.perception_provider.agent_location.y))
            print("needed: ({},{})".format(self.goal_location.x + self.submit_origin.x,self.goal_location.y+self.submit_origin.y))
            
            if len(self.path) > 0:
                # print("agent:{} ".format(self.perception_provider.agent_location))
                if self.perception_provider.agent.last_action in ["move","rotate"]:
                    if self.perception_provider.agent.last_action_result == "success":
                        if self.perception_provider.agent.last_action_params[0] == self.path[self.index]:
                            self.index += 1
                            print("index: {}".format(self.index))
                if self.index < len(self.path):
                    direction = self.path[self.index]
                
                # if len(self.path) == self.index:
                #     print("len: {}".format(len(self.path)))
                #     submit_cell = Position(self.goal_location.x + self.submit_origin.x,
                #                            self.goal_location.y + self.submit_origin.y)
                #     self.path = []
                #     self.path = get_astar_path(self.perception_provider.local_map,
                #                                self.perception_provider.agent_location,submit_cell,full_path=True)
                #     if len(self.path) > 0:
                #         self.index = 0
                #         print("######### Other path {}".format(self.path))
                #         direction = self.path[0]

               
                
                
        
        # else:
        #     print("Target reached")
            # if self.goal_location.x + self.submit_origin.x == self.perception_provider.agent_location.x:
            #     if self.goal_location.y + self.submit_origin.y == self.perception_provider.agent_location.y:

        # if self.perception_provider.agent.last_action == "move":
        #         if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
        #             print("{} Path planning failed".format(self._agent_name))
        #             self.path = []
        # print("origin: {} \n target: {}".format(self.submit_location,self.target_submit))

        # direction = ''
        # # target = Positon(self.target.x + self.goal_location.x,self.target.y + self.goal_location.y)
        # if not self.reached_loc:
        #     if self.perception_provider.goal_origin != self.goal_location:
        #         self.goal_location = self.perception_provider.goal_origin
        #         self.path=[]

        #     agent_location = self.perception_provider.agent_location #Agent location
        #     #block location same as dispenser
        #     block_location = Position(self.target_dispenser.x + self.goal_location.x, self.target_dispenser.y + self.goal_location.y)
        #     #if submit agent
        #     if self.target_submit.x + self.target_submit.y == 1 and len(self.path) == 0:
        #         print("First_agent ########")
        #         #agent goes to submit cell
        #         self.target_agent = Position(self.submit_location.x + self.goal_location.x, self.submit_location.y + self.goal_location.y)
        #         print("Target_agent_state: ({},{})".format(self.target_agent.x,target_agent.y))
        #         #block goes to respective location
        #         self.target_block = Position(self.target_agent.x + self.target_submit.x, self.target_agent.y + self.target_submit.y)
            
        #     elif len(self.path) == 0:
        #         self.target_block = Position(self.submit_location.x + self.goal_location.x + self.target_submit.x, self.submit_location.y + self.goal_location.y +  self.target_submit.y)
        #         self.target_agent = self.get_target_agent(self.target_block)                
                
            
        #     if self.target_agent is not None:
        #     # print("Target_org: {} \n Target_modified: {} \n agent: {} \n  Goal {}".format(self.target_submit, target,agent_location,self.goal_location))
        #     # print("Map shape: {}".format(self.perception_provider.local_map.shape))
        
        #         print("Target_agent: {} \n Target_block: {}".format(self.target_agent,self.target_block) )

        #         if abs(agent_location.x - self.target_agent.x) + abs(agent_location.y - self.target_agent.y) == 0:
        #             if self.perception_provider.local_map[agent_location.y + self.target_submit.y][agent_location.x + self.target_submit.x] == 10:
        #                 self.reached_loc = True
        #                 if len(self.perception_provider.selected_task.requirements) == 1:
        #                     self.perception_provider.is_submit_agent = True
                        
        #                 print("{} has reached disp".format(self._agent_name))

        #         else:

        #             if len(self.path) == 0:
                        
        #                 self.path = get_astar_path(self.perception_provider.local_map,agent_location,self.target_agent,block_location,self.target_block,flag=1,full_path=True)
        #                 print("Path: {}".format(self.path))
        #                 if len(self.path) > 0:
        #                     direction = self.path.pop(0)
        #                     print(direction)

        #             else:
        #                 direction = self.path.pop(0)

       
        #     else:
        #         print("Target_agent is None **********")
       


        
        if len(direction) >= 2:
                
                # if direction == 'cw':
                #     self.angle = (self.angle - 90)
                #     if self.angle < 0:
                #         self.angle += 360
                # else:
                #     self.angle = (self.angle + 90) % 360
                # if self.angle in [0,180]:
                #     self.block_location.x += self.math_ops(cos=True,self.angle)
                #     self.block_location.y += 
                # else:
                #     self.block_location.y += 

            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ROTATE, params=params)

        else:
            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)

class Submit(BehaviourBase):

    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Submit, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)
        # print("Explore behaviour _1")
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
    
    def do_step(self):

        print("{} in submit".format(self._agent_name))
        if self.perception_provider.is_submit_agent:
            params = [KeyValue(key="task", value=self.perception_provider.selected_task.name)]
            rospy.logdebug(self._agent_name + "::" + self._name + " submiting task  " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_SUBMIT, params=params)

            print(" *************** Hurrah *******************")
