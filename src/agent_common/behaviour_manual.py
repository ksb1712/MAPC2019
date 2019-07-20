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

from agent_common.behaviour_base import action_generic_simple


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