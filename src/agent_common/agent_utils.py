from __future__ import division  # force floating point division when using plain /

import math

from mapc_ros_bridge.msg import Position,Task,Requirement

import numpy as np
import rospy




def get_bridge_topic_prefix(agent_name):
    """
    Determine the topic prefix for all topics of the bridge node corresponding to the agent (name)
    :param agent_name: current agents name
    :return: prefix just before the topic name of the bridge
    """
    return '/bridge_node_' + agent_name + '/'


def euclidean_distance(pos1, pos2):
    """
    Calculate the euclidean distance between two positions
    :param pos1: position 1
    :type pos1: Position
    :param pos2: position 2
    :type pos2: Position
    :return: euclidean distance
    """
    return math.sqrt((pos1.x - pos2.x) ** 2 + (pos1.y - pos2.y) ** 2)


def relative_euclidean_distance(pos1):
    """
    Calculate the relative euclidean distance to a point
    :param pos1: position 1
    :type pos1: Position
    :return: relative euclidean distance
    """
    return math.sqrt(pos1.x ** 2 + pos1.y ** 2)


def pos_to_direction(pos):
    """
    Determine a direction from a given relative position
    :param pos: relative position with x and y
    :return: direction string like 'n', 's', 'e', 'w'
    """

    if pos.x == 0 and pos.y > 0:
        return 's'
    elif pos.x == 0 and pos.y < 0:
        return 'n'
    elif pos.y == 0 and pos.x > 0:
        return 'e'
    elif pos.y == 0 and pos.x < 0:
        return 'w'
    else:  # if not directly on the same column/row as the position, we try to reduce the smaller axis value (x/y) first
        if pos.x > pos.y:
            if pos.y > 0:
                return 's'
            else:
                return 'n'
        else:
            if pos.x > 0:
                return 'e'
            else:
                return 'w'
    return None




def callback_transformMotion(self, twist):
    """
    Transform Twist msg to 2D and return direction
    :param twist:
    :return:
    """
    norm_distance = twist.linear.x
    angle = twist.angular.z
    gridY = norm_distance * np.cos(angle)
    gridX = norm_distance * np.sin(angle)

    if np.abs(gridX) > np.abs(gridY):
        if np.sign(gridX) == 1:
            direction = 'e'
            rospy.logdebug("######## EAST#######")
        else:
            direction = 'w'
            rospy.logdebug("######## WEST######")
    else:
        if np.sign(gridY) == 1:
            direction = 'n'
            rospy.logdebug("######## NORTH ######")
        else:
            direction = 's'
            rospy.logdebug("####### SOUTH######")

    #Uncomment below if global variable direction
    #self.direction=direction

    return direction



"""
Utility class for breaking tasks into requirements and making decisions to select task for current agent or not.
Returns the target Position of block
Note:This tested as an alternate approach and not used currently
"""
class TaskBreakdown():


    def __init__(self):
        self.onTask = False,
        self.hasTaskBlock = False
        self.isSubmitter = False
        self.task = []
        self.myBlock = ""
        self.targetPosBlock = []
        self.hasactiveTask = False
        self.activeTasks = []
        self.taskname = ""
        self.activetaskTopic = "activeTask"
        self.taskCompletedTopic = "doneTask/" + self.taskname
        self.myCurrentReq = []
        self.onTask = False

        rospy.Subscriber(self.activetaskTopic, Task, self.callback_active_task)
        self.activeTaskPub = rospy.Publisher(self.activetaskTopic, Task, queue_size=1,
                                             latch=True)
        self.doneTaskPub = rospy.Publisher(self.taskCompletedTopic, Requirement, queue_size=1,
                                           latch=True)


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


    def initTask(self,task,block):
        """
        This method will break down  each requirement of a task for agent to select.
        It will first find the submitter cell position and publish other requirements
        on ROS topic activeTasks which each agent subscribes
        :param self:
        :return target Position of block of requirement:
        """
        rospy.loginfo("#######inside intitask")

        #block = perception_provider.blocks
        if block:
            myBlock = block.type
        else:
            myBlock = "b0"


        #task=perception_provider.tasks
        if task:
            # self.task = self.perception_provider.tasks
            # currenttask = self.task[len(self.task)-1]

            for t, currenttask in enumerate(task):
                rospy.loginfo("####current task %s", currenttask)

                # # Temp
                # if self._agent_name == "agentA1":
                #     self.myBlock = "b0"
                # else:
                #     self.myBlock = "b1"
                # # Temp END

                if len(currenttask.requirements) == 0:
                    rospy.loginfo("task empty")
                    return

                reqList = currenttask.requirements
                initPos = Position()
                initPos.x = 0
                initPos.y = 0
                distance = 1000
                submitterIndex = 0
                # self.hasactiveTask = True
                if self.onTask is False:
                    if self.hasactiveTask is False:  # if no active task found from susbscriber
                        for i, elem in enumerate(reqList):
                            thispos = elem.pos
                            thisdistance = euclidean_distance(thispos, initPos)
                            rospy.logdebug("###comparing distance %f", thisdistance)
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
                            rospy.logdebug("#####publishing new list %s", reqList)
                            currenttask.requirements = reqList
                            self.activeTaskPub.publish(currenttask)  # publish new req
                            return self.targetPosBlock
                    else:  # if active task exists
                        thistask = self.activeTasks

                        taskreq = thistask.requirements
                        for j, reqElem in enumerate(taskreq):
                            reqBlockType = reqElem.type

                            if myBlock == reqBlockType or myBlock != reqBlockType:
                                rospy.logdebug("#####got equal block ")
                                self.hasTaskBlock = True
                                self.targetPosBlock = reqElem.pos
                                self.myCurrentReq = reqElem
                                # TO DO call path planner and check if valid path exists ?
                                taskreq.pop(j)
                                thistask.requirements = taskreq
                                rospy.logdebug("#####publishing new list %s", thistask)
                                self.activeTaskPub.publish(thistask)  # publish new req
                                if not taskreq:
                                    self.hasactiveTask = False
                                return self.targetPosBlock

