#!/usr/bin/env python2

import rospy
from mapc_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye

from behaviour_components.managers import Manager
from behaviour_components.activators import BooleanActivator, ThresholdActivator, EqualActivator, GreedyActivator
from behaviour_components.conditions import Negation, Condition
from behaviour_components.goals import GoalBase
from behaviour_components.condition_elements import Effect

from agent_common.behaviours import RandomMove, Dispense, MoveToDispenser, Attach, AgentControl
from agent_common.behaviours import Explore
from agent_common.providers import PerceptionProvider
from agent_common.agent_utils import get_bridge_topic_prefix


def control_player():
    print("1. move north")
    print("2. move south")
    print("3. move east")
    print("4. move west")
    print("5. Dispense block")
    print("6. Attach block")
    print("7. Detach block")
    print("8. Rotate cw")
    print("9. Rotate ccw")
    print("10. connect")
    print("11. submit")


class RhbpAgent(object):
    """
    Main class of an agent, taking care of the main interaction with the mapc_ros_bridge
    """

    def __init__(self):
        rospy.logdebug("RhbpAgent::init")

        rospy.init_node('agent_node', anonymous=True, log_level=rospy.INFO)

        self._agent_name = rospy.get_param('~agent_name', 'agentA1')  # default for debugging 'agentA1'

        self._agent_topic_prefix = get_bridge_topic_prefix(agent_name=self._agent_name)

        # ensure also max_parallel_behaviours during debugging
        self._manager = Manager(prefix=self._agent_name, max_parallel_behaviours=1)

        self.behaviours = []
        self.goals = []

        self.perception_provider = PerceptionProvider()

        self._sim_started = False


        self.manual_player = False





        # subscribe to MAPC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

    def _sim_start_callback(self, msg):
        """
        here we could also evaluate the msg in order to initialize depending on the role etc.
        :param msg:  the message
        :type msg: SimStart
        """

        if not self._sim_started:  # init only once here

            rospy.loginfo(self._agent_name + " started")

            # creating the actual RHBP model
            self._initialize_behaviour_model()

        self._sim_started = True

    def _callback_generic_action(self, msg):
        """
        ROS callback for generic actions
        :param msg: ros message
        :type msg: GenericAction
        """
        self._received_action_response = True

    def _sim_end_callback(self, msg):
        """
        :param msg:  the message
        :type msg: SimEnd
        """
        rospy.loginfo("SimEnd:" + str(msg))
        for g in self.goals:
            g.unregister()
        for b in self.behaviours:
            b.unregister()
        self._sim_started = False

    def _bye_callback(self, msg):
        """
        :param msg:  the message
        :type msg: Bye
        """
        rospy.loginfo("Simulation finished")
        rospy.signal_shutdown('Shutting down {}  - Simulation server closed'.format(self._agent_name))

    def _action_request_callback(self, msg):
        """
        here we just trigger the decision-making and planning
        while tracking the available time and behaviour responses
        :param msg: the message
        :type msg: RequestAction
        """

        # calculate deadline for the current simulation step
        start_time = rospy.get_rostime()
        safety_offset = rospy.Duration.from_sec(0.2)  # Safety offset in seconds
        deadline_msg = rospy.Time.from_sec(msg.deadline / 1000.0)
        current_msg = rospy.Time.from_sec(msg.time / 1000.0)
        deadline = start_time + (deadline_msg - current_msg) - safety_offset

        self.perception_provider.update_perception(request_action_msg=msg)

        self._received_action_response = False

        # self._received_action_response is set to True if a generic action response was received(send by any behaviour)
        while not self._received_action_response and rospy.get_rostime() < deadline:
            # wait until this agent is completely initialised
            if self._sim_started:  # we at least wait our max time to get our agent initialised

                # action send is finally triggered by a selected behaviour
                self._manager.step(guarantee_decision=True)
            else:
                rospy.sleep(0.1)

        if self._received_action_response:  # One behaviour replied with a decision
            duration = rospy.get_rostime() - start_time
            rospy.logdebug("%s: Decision-making duration %f", self._agent_name, duration.to_sec())

        elif not self._sim_started:  # Agent was not initialised in time
            rospy.logwarn("%s idle_action(): sim not yet started", self._agent_name)
        else:  # Our decision-making has taken too long
            rospy.logwarn("%s: Decision-making timeout", self._agent_name)

    def _initialize_behaviour_model(self):
        """
        This function initialises the RHBP behaviour/goal model.
        """
        if self.manual_player:
            control_player()
            control = AgentControl(name="manual_control", perception_provider=self.perception_provider,
                                                agent_name=self._agent_name)
            self.behaviours.append(control)
        else:
            # Exploration targeted at locating goal
            explorer = Explore(name="explore", perception_provider=self.perception_provider, agent_name=self._agent_name)
            self.behaviours.append(explorer)
            explorer.add_effect(
                Effect(self.perception_provider.count_goal_cells.name, indicator=+1, sensor_type=float))

            
if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
