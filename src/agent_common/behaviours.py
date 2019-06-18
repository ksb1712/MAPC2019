from __future__ import division  # force floating point division when using plain /
import rospy
import random
import time, sys

from behaviour_components.behaviours import BehaviourBase

from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction

from agent_common.agent_utils import get_bridge_topic_prefix, pos_to_direction

from agent_common.providers import PerceptionProvider

import numpy as np
import math
import agent_common.astar

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

        # elif opt == 10:
        # elif opt == 11:

        print(params)
        action_generic_simple(publisher=self._pub_generic_action, action_type=action_type, params=params)
        time.sleep(0.2)
        sys.stdin.flush()        

class Explore(BehaviourBase):
    """
    Independent exploration behaviour 
    Objective: to find 12 goal cells
    exploration behaviour: in direction of least overlap of perception provider
    """
    def __init__(self, name, agent_name, perception_provider, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param perception_provider: the current perception
        :type perception_provider: PerceptionProvider
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(Explore, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)
        # print("Explore behaviour _1")
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
        self.prev_param = []


    def get_direction(self, x,y,h,w,p_range,prev_param=[]):
        min_val= p_range
        random_move = ['n','e','w','s']

        direction = random.choice(random_move)

        if(abs((y - 1)) < min_val):
            min_val = abs((y - 1))
            direction = 'n'
        if(abs(h - (y + 1)) < min_val):
            min_val = abs(h - (y + 1))
            direction = 's'
        if(abs((x - 1)) < min_val):
            min_val = abs((x - 1))
            direction = 'w'
        if(abs(w - (x + 1)) < min_val):
            min_val = abs(w - (x + 1))
            direction = 'e'
        
        if len(prev_param) > 0:
            check = True
            while check:
                if(direction in prev_param):
                    direction = random.choice(random_move)
                else:
                    check = False
        
        print("Going " + direction)
        
        return direction
            

            

    def do_step(self):
        

        if (self.perception_provider.agent.last_action == "move" and 
            self.perception_provider.agent.last_action_result != "success"):
            temp = self.perception_provider.agent.last_action_params[0]
            if temp not in self.prev_param:
                self.prev_param.append(temp)
            if len(self.prev_param) > 3:
                self.prev_param.pop(0)
        
        """
        find direction with least overlap
        """
        local_map = self.perception_provider.local_map
        perception_range = self.perception_provider.perception_range
        x , y = self.perception_provider.agent_location.x, self.perception_provider.agent_location.y
        h, w = local_map.shape
        
        print("In explore")
        if len(self.perception_provider.relative_goals) < 12:
            if self.perception_provider.closest_goal:
                print("Found goal")
                direction = pos_to_direction(self.perception_provider.closest_goal.pos)

            else:
                direction = self.get_direction(x,y,h,w,perception_range,self.prev_param)
        else:
                direction = self.get_direction(x,y,h,w,perception_range,self.prev_param)

        params = [KeyValue(key="direction", value=direction)]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)




        

class RandomMove(BehaviourBase):
    """
    Move in randomly chosen directions
    """
    
    def __init__(self, name, agent_name, **kwargs):
        """
        :param name: name of the behaviour
        :param agent_name: name of the agent for determining the correct topic prefix
        :param kwargs: more optional parameter that are passed to the base class
        """
        super(RandomMove, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

        self._agent_name = agent_name
        

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
        
    
        self.count = 0

    def do_step(self):
        # if self.count < 40:
        #     random_move = ['e']
        #     self.count += 1
        # else:
        #     random_move = ['n','e']
        random_move = ['n','e','w','s']
        move = raw_input("Enter a direction: ")
        random_move = [move]
        params = [KeyValue(key="direction", value=random.choice(random_move))]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)


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

        if self._perception_provider.closest_dispenser:

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)
            # random_move = ['n', 's', 'e', 'w']
            # direction = random.choice(random_move)
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

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):

        if self._perception_provider.closest_dispenser:

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)

            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " request dispense " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no dispenser in range", self.name)


# class Plan_Path(BehaviourBase):
#     """
#     Behaviour to decide which agent should do
#     which part of task. Uses AStar path planner
#     """

#     def __init__(self, name, agent_name, perception_provider):
#          """
#         :param name: name of the behaviour
#         :param agent_name: name of the agent for determining the correct topic prefix
#         :param perception_provider: the current perception
#         :type perception_provider: PerceptionProvider
#         :param kwargs: more optional parameter that are passed to the base class
#         """
#         super(Attach, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)

#         self._agent_name = agent_name

#         self._perception_provider = perception_provider

#         self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
#                                                    , queue_size=10)

#          #Get map
#         agent_map = self._perception_provider.local_map
#         grid_height, grid_width = agent_map.shape

#         #Get location of obstacles
#         obs_Y, obs_X = np.where(agent_map==1)
        
#         #Make tuples
#         obs_cood = []
#         for i in range(len(obs_X)):
#             obs_cood.append((obs_Y[i],obs_X[i]))
        
#         obs_cood = tuple(obs_cood)

#         #Get current location
#         cur_location = (self._perception_provider.agent_location.x,self._perception_provider.agent_location.y)

#         #Get first goal cell location
#         goal_list = self._perception_provider.goals
#         goal_location = (goal_list[0].x,goal_list[0].y)

#         #Initialize AStar

#         grid = astar.AStar(grid_height,grid_width)
#         grid.init_grid(obs_cood,cur_location,goal_location)
#         self.path = grid.get_path()

#         self.path_index = 0

#     def do_step(self):
#         if(self.path_index < len(self.path)):
#             direction = self.path[self.path_index]
#             self.path_index += 1

#             params = [KeyValue(key="direction", value=direction)]
#             rospy.logdebug(self._agent_name + "::" + self._name + " request dispense " + str(params))
#             action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE,
#                                   params=params)

#         else:
#             rospy.logerr("Path planning completed for {}".format(self.agent_name))

            
       






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

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):

        if self._perception_provider.closest_block:

            direction = pos_to_direction(self._perception_provider.closest_block.pos)
            # random_move = ['n', 's', 'e', 'w']
            # direction = random.choice(random_move)
            params = [KeyValue(key="direction", value=direction)]

            rospy.logdebug(self._agent_name + "::" + self._name + " attach request " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_ATTACH,
                                  params=params)

        else:
            rospy.logerr("Behaviour:%s: no BLOCK in range", self.name)