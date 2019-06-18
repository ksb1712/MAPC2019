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

        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

        def get_direction(self, x,y,h,w,p_range,prev_param=''):
            min_val= p_range

            direction = 'n'

            if(abs((y - 1)) < min_val):
                min_val = abs((y - 1))
                direction = 'n'
            elif(abs(h - (y + 1)) < min_val):
                min_val = abs(h - (y + 1))
                direction = 's'
            elif(abs((x - 1)) < min_val):
                min_val = abs((x - 1))
                direction = 'w'
            elif(abs(w - (x + 1)) < min_val):
                min_val = abs(w - (x + 1))
                direction = 'e'
            
            if len(prev_param) > 0:
                check = True
                while check:
                    if(direction == prev_param):
                        random_move = ['n','e','w','s']
                        direction = random.choice(random_move)
                    else:
                        check = False
            
            return direction
            

            

        def do_step(self):
            
            prev_param = ''
            if (self.perception_provider.agent.last_action == "move" and 
                self.perception_provider.agent.last_action_result != "success"):
                prev_param = self.perception_provider.agent.last_action_params[0]
            
            """
            find direction with least overlap
            """
            local_map = self.perception_provider.local_map
            perception_range = self.perception_provider.perception_range
            x , y = self.perception_provider.agent_location.x, self.perception_provider.agent_location.y
            h, w = local_map.shape
            
            direction = self.get_direction(x,y,h,w,perception_range,prev_param)
            params = [KeyValue(key="direction", value=direction)]
            rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
            action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)



            