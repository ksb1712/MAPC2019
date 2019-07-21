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
        """
        identify the direction to dispense from 
        and call dispense. Must be near dispenser
        """
        direction = ''
        if self._perception_provider.sensor_dispensed_blocks._value < 1:
            
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
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.path = [] 
        self.goal_location = self.perception_provider.goal_origin #Global origin
        self.reached_loc = False
        self.target_cell = self.perception_provider.target_dispenser_cell #Target dispenser in transformed coordinates

    
    def do_step(self):
        
        #Check for failure and reset
        if self.perception_provider.agent.last_action == "move":
                if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
                    self.path = []
        direction = ''
        #Keep moving untill dispenser is reached
        if not self.reached_loc:
            if self.perception_provider.goal_origin != self.goal_location:
                self.goal_location = self.perception_provider.goal_origin
                self.path=[]
            
            #Current location
            agent_location = self.perception_provider.agent_location
            #Transform dispenser location to local frame
            target = Position(self.target_cell.x + self.goal_location.x, self.target_cell.y + self.goal_location.y)
            if abs(agent_location.x - target.x) + abs(agent_location.y - target.y) <= 1:
                self.reached_loc = True

            else:

                if len(self.path) == 0:
                    
                    self.path = get_astar_path(self.perception_provider.local_map,agent_location,target,full_path=False)
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
        
        """
        find direction to attach block and 
        call attach.
        """
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
        
            params = [KeyValue(key="direction", value=direction)]

            rospy.logdebug(self._agent_name + "::" + self._name + " attach request " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ATTACH,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no BLOCK in range", self.name)



class Explore_astar(BehaviourBase):

    """
    Independent exploration behaviour 
    Objective: to find 12 goal cells and later task dependent dispensers
    exploration behaviour: based on count_of_unexplored cells and distance to cell

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
        self.history = []

    
    def mh_distance(self,a,b):
        """
        Get manhattan distance between two cells
        """
        x_dist = abs(a.x - b.x)
        y_dist = abs(a.y - b.y)

        return x_dist + y_dist

    
    def unexplored_score(self,a,b,agent_map):
        """
        Get cell score which depends on the 
        number of unexplored cells
        a: current location
        b: cell_location
        agent_map: current local map
        """
        H, W = agent_map.shape
        range_val = 4
        left = max(0,b.x - range_val)
        right = min(W-1,b.x + range_val)
        up = max(0,b.y - range_val)
        down = min(H-1,b.y + range_val)
        dist = self.mh_distance(a,b)

        count = 0
        count_obs = 0
        for i in range(left,right):
            for j in range(up, down):
                if agent_map[j][i] == -1:
                    count += 1
                if agent_map[j][i] == 1:
                    count_obs += 1
        
        return float(count) / (float(dist))

    def get_new_path(self,agent_map,agent_location):

        """
        Choose target cell for exploration and call A*
        agent_map: current local map
        agent_location: current location of agent
        """

        #Find Free cells
        u_Y, u_X = np.where(agent_map == 0)
        list_unexplored = [Position(u_X[i],u_Y[i]) for i in range(len(u_X))]
        min_array = [self.unexplored_score(agent_location,cell,agent_map) for cell in list_unexplored]
        
        check = True
        path = []
        min_index = min_array.index(max(min_array))
        target_unexplored_cell = list_unexplored[min_index]
        #Try to find suitable cell which can be reached and hasn't been chosen before
        while True:
            if target_unexplored_cell not in self.history:
                path = get_astar_path(agent_map,agent_location,target_unexplored_cell,full_path=True)
                if len(path) > 0:
                    self.history.append(target_unexplored_cell)
                    return path
                else:
                    #If chosen cell can't be reached, remove from list
                    self.history.append(target_unexplored_cell)
                    min_array.pop(min_index)
                    list_unexplored.pop(min_index)
                
                    min_index = min_array.index(max(min_array))
                    target_unexplored_cell = list_unexplored[min_index]
            else:
                #if cell already visited, remove from list
                min_array.pop(min_index)
                list_unexplored.pop(min_index)
                
                min_index = min_array.index(max(min_array))
                target_unexplored_cell = list_unexplored[min_index]

                    
            
    def do_step(self):
        direction = ''
        #Call exploration only if target dispenser not selected for task
        if not self.perception_provider.target_disp_selected:

            agent_map = self.perception_provider.local_map
            agent_location = self.perception_provider.agent_location
            last_d = ''
            
            #Check for failure
            if self.perception_provider.agent.last_action == "move":
                if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:
                    self.path = []
                    self.goal_path = False
                
                

            
            if self.path is None:
                self.path = []

            #If already on way towards goal
            if len(self.path) > 0 and self.goal_path:
                direction = self.path.pop(0)
                
            else:
                #Check for goal cells
                g_Y, g_X = np.where(agent_map == 7)
                if len(g_X) > 0 and len(g_X) < 11:
                    goal_array = [Position(g_X[i],g_Y[i]) for i in range(len(g_X))]
                    dist_goal_array = [self.mh_distance(agent_location,cell) for cell in goal_array]
                    target_goal = goal_array[dist_goal_array.index(max(dist_goal_array))]
                    self.goal_path = True
                    self.path = get_astar_path(agent_map,agent_location,target_goal,full_path=False)
                    if len(self.path) > 0:
                        direction = self.path.pop(0)

                #if goal cell not in range
                elif len(self.path) > 0:
                    direction = self.path.pop(0)
                
                #Get new path to unexplored cell
                else:
                    self.path = []
                    #As A* is stopped after a while check if path exists
                    while len(self.path) < 1: 
                        self.path = self.get_new_path(agent_map,agent_location)
                        agent_map = self.perception_provider.local_map
                        agent_location = self.perception_provider.agent_location
                
                    direction = self.path.pop(0)
                    self.goal_path = False
            
           
                
        params = [KeyValue(key="direction", value=direction)]
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
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        self.path = []
        self.goal_location = None   #Global origin
        self.submit_origin = self.perception_provider.submit_origin #Relativ location of submit cell
        self.block_location = None  #Location of block attached to agent
        self.target_dispenser = None #Location of dispenser which was allocated to agent
        self.angle = 0

        self.reached_loc = False
        self.target_agent = None
        self.target_block = None

        self.index = 0
       


        return None

    
    def do_step(self):

        direction = ''
        #Check for failure
        if self.perception_provider.agent.last_action == "move":
            if self.perception_provider.agent.last_action_result in ["failed_path","failed_parameter"]:                
                self.path = []
                self.index = 0
                self.reached_loc = False

       
        #If not path or not yet reached
        if len(self.path) == 0 and not self.reached_loc:
            
            
            task_submit = self.perception_provider.target_submit_cell
            if self.submit_origin is None:
              self.submit_origin = self.perception_provider.submit_origin
            
            agent_location = self.perception_provider.agent_location
            if self.block_location is None:
                self.target_dispenser = Position(self.perception_provider.target_dispenser_cell.x,self.perception_provider.target_dispenser_cell.y)
                self.goal_location = Position(self.perception_provider.goal_origin.x,self.perception_provider.goal_origin.y)
                self.block_location = Position(self.target_dispenser.x + self.goal_location.x, self.target_dispenser.y + self.goal_location.y)
              
            #Get locations in current frame
            target_agent = Position(self.goal_location.x + self.submit_origin.x,self.goal_location.y + self.submit_origin.y)
            target_block = Position(self.goal_location.x + self.submit_origin.x + task_submit.x,
                                    self.goal_location.y + self.submit_origin.y + task_submit.y)
            
           
           #Call path planner for block + agent system 
            self.path = get_astar_path(self.perception_provider.local_map,agent_location,
                                        target_agent,self.block_location,target_block,flag=1,full_path=True)

            if len(self.path) > 0:
                direction = self.path[0]
            else:
                #if no path => already at location as map is static
                self.reached_loc = True
                self.perception_provider.is_submit_agent = True
        
        #if value increases beyond limit
        elif self.index >= len(self.path):
            
            if (self.goal_location.x + self.submit_origin.x) == self.perception_provider.agent_location.x:
                if (self.goal_location.y + self.submit_origin.y) == self.perception_provider.agent_location.y:
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
                    direction = self.path[0]
      

        elif not self.reached_loc:
            
            if len(self.path) > 0:
                #increase index if previous move was success
                if self.perception_provider.agent.last_action in ["move","rotate"]:
                    if self.perception_provider.agent.last_action_result == "success":
                        if self.perception_provider.agent.last_action_params[0] == self.path[self.index]:
                            self.index += 1
                if self.index < len(self.path):
                    direction = self.path[self.index]
                
             

        
        if len(direction) >= 2:
                
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
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
    
    def do_step(self):

        if self.perception_provider.is_submit_agent:
            params = [KeyValue(key="task", value=self.perception_provider.selected_task.name)]
            rospy.logdebug(self._agent_name + "::" + self._name + " submiting task  " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_SUBMIT, params=params)

