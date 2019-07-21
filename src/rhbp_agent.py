#!/usr/bin/env python2

import rospy
from mapc_ros_bridge.msg import RequestAction, GenericAction, SimStart, SimEnd, Bye,Task,Requirement,Position

from behaviour_components.managers import Manager
from behaviour_components.activators import BooleanActivator, ThresholdActivator, EqualActivator, GreedyActivator
from behaviour_components.conditions import Negation, Condition, Conjunction
from behaviour_components.goals import GoalBase
from behaviour_components.condition_elements import Effect

from agent_common.behaviours import  Dispense, MoveToDispenser, Attach, AgentControl
from agent_common.behaviours import   Explore_astar, MoveToGoal, Submit
from agent_common.providers import PerceptionProvider
from agent_common.agent_utils import get_bridge_topic_prefix,euclidean_distance,TaskBreakdown


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




    """ 
    MSG: mapc_ros_bridge/Requirement
    Position pos
    string details
    string type
    """



def checkAttachedBlock(self):

    block=self.perception_provider.blocks
    myPos=Position()
    if block:
        type=block.type
        """
        TO DO calculate if block is attached to agent based on position
        """
        return type


def initTask(self):
    """
    This method will break down  each requirement of a task for agent to select.
    It will first find the submitter cell position and publish other requirements
    on ROS topic activeTasks which each agent subscribes
    :param self:
    :return:
    """
    rospy.loginfo("#######inside intitask")

    block=self.perception_provider.blocks
    if block:
        myBlock=block.type
    else:
        myBlock = "b0"


    if self.task:
        #self.task = self.perception_provider.tasks
        #currenttask = self.task[len(self.task)-1]

        for t,currenttask in enumerate(self.task):
            rospy.loginfo("####current task %s", currenttask)

            #Temp
            if self._agent_name=="agentA1":
                self.myBlock ="b0"
            else:
                self.myBlock = "b1"
            #Temp END


            if len(currenttask.requirements) == 0:
                rospy.loginfo("task empty")
                return
            
            reqList = currenttask.requirements
            initPos = Position()
            initPos.x = 0
            initPos.y = 0
            distance = 1000
            submitterIndex = 0
            #self.hasactiveTask = True
            if self.onTask is False:
                if self.hasactiveTask is False:  # if no active task found from susbscriber
                    for i, elem in enumerate(reqList):
                        thispos = elem.pos
                        thisdistance = euclidean_distance(thispos, initPos)
                        rospy.logdebug("###comparing distance %f",thisdistance)
                        if thisdistance < distance:
                            distance = thisdistance
                            submitterIndex = i

                    submitterReq = reqList[submitterIndex]
                    submitterType = submitterReq.type
                    
                    if myBlock == submitterType:
                        rospy.logdebug("###submitter equal agent")
                        self.hasTaskBlock = True
                        self.isSubmitter = True
                        self.targetPosBlock = reqList[submitterIndex].pos
                        reqList.pop(submitterIndex)
                        rospy.logdebug("#####publishing new list %s",reqList)
                        currenttask.requirements=reqList
                        self.activeTaskPub.publish(currenttask)  # publish new req
                else:  # if active task exists
                    thistask = self.activeTasks
                    
                    taskreq=thistask.requirements
                    for j,reqElem in enumerate(taskreq):
                        reqBlockType = reqElem.type

                        if myBlock == reqBlockType or myBlock!=reqBlockType:
                            rospy.logdebug("#####got equal block ")
                            self.hasTaskBlock = True
                            self.targetPosBlock = reqElem.pos
                            self.myCurrentReq = reqElem
                            # TO DO call path planner and check if valid path exists ?
                            taskreq.pop(j)
                            thistask.requirements=taskreq
                            rospy.logdebug("#####publishing new list %s", thistask)
                            self.activeTaskPub.publish(thistask)  # publish new req
                            if not taskreq:
                                self.hasactiveTask=False
                            break

def workOnTask(self):
    """
    Move to destination and execute behaviours for connect,submit
    """

    rospy.loginfo("#######inside workonTask")

    if self.hasTaskBlock :
        transformBlockLocIntoPos =[0,0]
        goalCell = [0, 0]  # TO DO Init Goal cell
        destination =self.targetPosBlock+goalCell #add goal cell coordinates to get absolute destination location


        path = Explore_better.get_astar_path( destination) # TO DO check path of AStar
        while path:  # if valid path found

            self.onTask = True
            reached = Explore_better.do_step()  # Move using path
            if reached:
                rospy.loginfo("Reached destination cell")
                if self.isSubmitter:  # submitter agent for current task
                    """
                    TO DO subscribe doneTasks and keep pooling if all requirements done,get agent name from details
                    TO DO wait for all agents connect and finally submit
                    TO DO how to detect agent ready for connect and submit
                    #connect behaviour
                    #keep pooling doneTask topic to check if relevent task are done by comparing local reqList copy
                    #submit behaviour
                    #if successful or task time out reset everything
                    #hasblock and ontask set to false						
                    """

                else:  # if its not submitter for current task
                    self.myCurrentReq.details = self._agent_name
                    self.doneTaskPub.publish(
                        self.myCurrentReq)  # publish what requiremnt is done for submitter to subscribe and know
                    """
                    TO DO wait for all agents connect  
                    #connect behaviour
                    #if successful or task time out reset everything
                    #hasblock and ontask set to false                      
                    """






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

        #if manual player is called, do not call other behaviours
        self.manual_player = False

        """
        my changes
        """
        self.onTask = False,
        self.hasTaskBlock = False
        self.isSubmitter=False
        self.task = []
        self.myBlock =""
        self.targetPosBlock = []
        self.hasactiveTask = False
        self.activeTasks = []
        self.taskname=""
        self.activetaskTopic = "activeTask"
        self.taskCompletedTopic = "doneTask/" + self.taskname
        self.myCurrentReq = []
        self.onTask = False


        # subscribe to MAPC bridge core simulation topics
        rospy.Subscriber(self._agent_topic_prefix + "request_action", RequestAction, self._action_request_callback)

        rospy.Subscriber(self._agent_topic_prefix + "start", SimStart, self._sim_start_callback)

        rospy.Subscriber(self._agent_topic_prefix + "end", SimEnd, self._sim_end_callback)

        rospy.Subscriber(self._agent_topic_prefix + "bye", Bye, self._bye_callback)

        rospy.Subscriber(self._agent_topic_prefix + "generic_action", GenericAction, self._callback_generic_action)

        self._received_action_response = False

        """
        my 
        """      
       
        rospy.Subscriber(self.activetaskTopic, Task, self.callback_active_task)
        self.activeTaskPub = rospy.Publisher(self.activetaskTopic, Task, queue_size=1,
                                                    latch=True)
        self.doneTaskPub = rospy.Publisher(self.taskCompletedTopic, Requirement, queue_size=1,
                                                    latch=True)

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

        """
        my
        """
        #self.task=self.perception_provider.tasks
        self.task=self.perception_provider._udpate_tasks(request_action_msg=msg)
        rospy.loginfo("###### get percept task %s",self.task)

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

        #initTask(self)
        #workOnTask(self)
        taskAllocator=TaskBreakdown()
        targetBlckPos=taskAllocator.initTask(self.task,self.perception_provider.blocks)

        if self.manual_player:
            control_player()
            control = AgentControl(name="manual_control", perception_provider=self.perception_provider,
                                                agent_name=self._agent_name)
            self.behaviours.append(control)
        else:
            # Exploration targeted at locating goal
            explorer = Explore_astar(name="explore", perception_provider=self.perception_provider, agent_name=self._agent_name,priority=1)
            self.behaviours.append(explorer)
            explorer.add_effect(
                Effect(self.perception_provider.count_goal_cells.name, indicator=+1, sensor_type=float))


            #Reach dispenser once origin has been obtained
            reach_dispenser = MoveToDispenser(name="reach_dispenser", perception_provider=self.perception_provider,
                                                agent_name=self._agent_name,priority=2)
            self.behaviours.append(reach_dispenser)
            
            reach_dispenser.add_effect(
                            Effect(self.perception_provider.closest_dispenser_distance_sensor.name, indicator=-1, sensor_type=float))
            
          
            pre_cond2 = Condition(self.perception_provider.target_dispenser_selected_sensor, 
                                  BooleanActivator(desiredValue=True))
            reach_dispenser.add_precondition(pre_cond2)
          
        
            # #Dispense from dispenser if near
            dispense = Dispense(name="dispense", perception_provider=self.perception_provider, 
                                agent_name=self._agent_name,priority=3)
            self.behaviours.append(dispense)
            dispense.add_effect(
                Effect(self.perception_provider.sensor_dispensed_blocks.name, indicator=+1, sensor_type=float))

            pre_cond3 = Condition(self.perception_provider.closest_dispenser_distance_sensor,
                                                ThresholdActivator(isMinimum=False, thresholdValue=1))
            dispense.add_precondition(Conjunction(pre_cond2,pre_cond3))

            # # Attach a block if close enough
            attach = Attach(name="attach", perception_provider=self.perception_provider, agent_name=self._agent_name,priority=4)
            self.behaviours.append(attach)
            pre_cond4 = Condition(self.perception_provider.sensor_dispensed_blocks,
                                         ThresholdActivator(isMinimum=True,thresholdValue=1))
            
            pre_cond5 =  Condition(self.perception_provider.closest_block_distance_sensor,
                                                ThresholdActivator(isMinimum=False, thresholdValue=1))
            attach.add_effect(
                Effect(self.perception_provider.sensor_attached_blocks.name, indicator=+1, sensor_type=float))

            attach.add_precondition(Conjunction(pre_cond4,pre_cond5))

<<<<<<< HEAD


            # attach_goal = GoalBase("attaching",permanent=True,
            #                         conditions=[Condition(self.perception_provider.sensor_attached_blocks, GreedyActivator())],
            #                         planner_prefix=self._agent_name,
            #                         priority=1)
=======
            #Move to goal once attached
>>>>>>> clean

            reach_goal = MoveToGoal(name="reach_goal", perception_provider=self.perception_provider,
                                                agent_name=self._agent_name,priority=10)
            self.behaviours.append(reach_goal)
            
            reach_goal.add_effect(
                            Effect(self.perception_provider.submit_count.name, indicator=+1, sensor_type=float))
            
           
            pre_cond6 = Condition(self.perception_provider.sensor_attached_blocks, 
                                  ThresholdActivator(isMinimum=True,thresholdValue=1))
            reach_goal.add_precondition(pre_cond6)
        
            
            #Submit Task if agent has reached location
            submit_task = Submit(name="submit_task",perception_provider=self.perception_provider,
                                                agent_name=self._agent_name,priority=20)

            self.behaviours.append(submit_task)

            submit_task.add_effect(Effect(self.perception_provider.score_sensor.name,indicator=+1, sensor_type=float))

            pre_cond7 = Condition(self.perception_provider.submit_sensor,
                                  ThresholdActivator(isMinimum=True,thresholdValue=1))
            
            submit_task.add_precondition(pre_cond7)


            #Keep dispensing if possible 
            dispense_goal = GoalBase("dispense_goal",permanent=True,
                                    conditions=[Condition(self.perception_provider.sensor_dispensed_blocks, GreedyActivator())],
                                    planner_prefix=self._agent_name,
                                    priority=1)

            self.goals.append(dispense_goal)

            #Attach to dispensed blocks
            attach_goal = GoalBase("attach_goal",permanent=True,
                                    conditions=[Condition(self.perception_provider.sensor_attached_blocks, GreedyActivator())],
                                    planner_prefix=self._agent_name,
                                    priority=2)

            self.goals.append(attach_goal)

            #Primary goal to get more points
            submit_goal = GoalBase("submit_goal",permanent=True,
                                    conditions=[Condition(self.perception_provider.score_sensor, GreedyActivator())],
                                    planner_prefix=self._agent_name,
                                    priority=4)



    
    def callback_active_task(self,msg) :
        """
        Callback for active task list
        :param self:
        :param msg:
        :return:
        """
        rospy.loginfo("####inside call back active task ")

        if msg:
            req=msg.requirements
            if req :
                self.activeTasks = msg
                rospy.loginfo("#### active task received %s", msg)
                self.hasactiveTask = True
            else:
                self.hasactiveTask = False
        else:
            self.hasactiveTask = False




                
                
						

"""
/WIP
"""






if __name__ == '__main__':
    try:
        rhbp_agent = RhbpAgent()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("program interrupted before completion")
