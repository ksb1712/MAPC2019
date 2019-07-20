from __future__ import division  # force floating point division when using plain /
import rospy
import random
import time, sys

from behaviour_components.behaviours import BehaviourBase

from diagnostic_msgs.msg import KeyValue
from mapc_ros_bridge.msg import GenericAction
from mapc_ros_bridge.msg import Position


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
