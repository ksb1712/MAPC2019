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
import agent_common.astar as astar

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

class Explore_better(BehaviourBase):

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
        super(Explore_better, self).__init__(name=name, requires_execution_steps=True, planner_prefix=agent_name, **kwargs)
        # print("Explore behaviour _1")
        self._agent_name = agent_name

        self.perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)
        self.prev_param = []
        self.direction_dict = {0: 'w',1: 'e', 2: 'n', 3: 's'}
        self.opp_direction_dict = {0: 'e', 1:'w',2:'s',3: 'n'}

        self.const_direction = self.direction_dict[random.randint(0,3)]

        self.goal_path = False
        # self.have_path = False
        # self.reached_loc = False

        self.path = []

    def mh_distance(self,a,b):
        x_dist = abs(a.x - b.x)
        y_dist = abs(a.y - b.y)

        return x_dist + y_dist

    def get_astar_path(self,agent_map,start,dest):
        
        grid_height, grid_width = agent_map.shape

        #Get location of obstacles
        
        x1 = (agent_map ==1)
    
        obs_Y, obs_X = np.where(x1)
        ent_Y, ent_X = np.where(agent_map == 8)

        obs_Y = obs_Y.tolist() + ent_Y.tolist()
        obs_X = obs_X.tolist() + ent_X.tolist()

        #Make tuples
        obs_cood = []
        for i in range(len(obs_X)):
            obs_cood.append((obs_X[i],obs_Y[i]))

        
        obs_cood = tuple(obs_cood)

        grid = astar.AStar(grid_height,grid_width)
        grid.init_grid(obs_cood,(start.x,start.y),(dest.x,dest.y))
        self.path = grid.get_path()
    

    def check_obstacle_closeness(self,agent_map,dest):
        
        range_ = 5 + 1
        for i in range(range_):
            if dest.x + i < agent_map.shape[1]:
                if agent_map[dest.y][dest.x + i] == 1:
                    return True
            if dest.x - i > 0:
                if agent_map[dest.y][dest.x - i] == 1:
                    return True
            if dest.y + i < agent_map.shape[0]:
                if agent_map[dest.y + i][dest.x] == 1:
                    return True
            
            if dest.y - i > 0:
                if agent_map[dest.y - i][dest.x] == 1:
                    return True

        return False
      

    def get_path_unknown(self,agent_map,agent_location,last_d):

        min_distance = 1000
        target_location = Position(agent_location.x,agent_location.y)
        
        #Find closest unexplored location

        u_Y, u_X = np.where(agent_map == -1)
        u_X = u_X.tolist()
        u_Y = u_Y.tolist()
        min_dist_array = [self.mh_distance(Position(i,j),agent_location) for i,j in zip(u_X,u_Y)]

        while len(min_dist_array) > 0:
            min_dist = min(min_dist_array)
            min_index = min_dist_array.index(min_dist)
            temp = Position(u_X[min_index],u_Y[min_index])
            if self.check_obstacle_closeness(agent_map,temp):
                min_dist_array.pop(min_index)
                u_X.pop(min_index)
                u_Y.pop(min_index)
            else:
                self.get_astar_path(agent_map,agent_location,temp)
                if self.path[0] != last_d:
                    print("{} --- {}".format(self._agent_name, self.path[0]))
                    break
                min_dist_array.pop(min_index)
                u_X.pop(min_index)
                u_Y.pop(min_index)
                



        # for i in range(len(un_exp[0])):
        #     temp = Position(un_exp[1][i],un_exp[0][i])
        #     dist = self.mh_distance(agent_location,temp)
        #     if( dist < min_distance and self.check_obstacle_closeness(agent_map,temp) == False):
        #         min_distance = dist
        #         target_location = temp

        # Get path from astar
        
    def get_inverse_distance_sum(self,_map,agent_location):

        obs_Y, obs_X = np.where(_map == 1)
        if len(obs_Y) > 0:
            _sum = 0.0
            for i in range(len(obs_Y)):
                dist = self.mh_distance(agent_location,Position(obs_X[i],obs_Y[i]))
                _sum += 1./dist
            
            return _sum
        else:
            return 0



    def do_step(self):

        print("{} Explore_b called".format(self._agent_name))

        # print("dispenser visible: {}".format(self.perception_provider.dispenser_visible_sensor._value))
        # print("#goal cells: {}".format(self.perception_provider.count_goal_cells._value))
        # print("#blocks: {}".format(self.perception_provider.closest_block_distance_sensor._value))

        agent_map = self.perception_provider.local_map
        agent_location = self.perception_provider.agent_location
        # direction = self.const_direction
        # goal_path = False
        random_move = ['n','e','w','s']
        last_d = ''
        direction = random.choice(random_move)
        if (self.perception_provider.agent.last_action == "move" and 
            self.perception_provider.agent.last_action_result == "failed_path"):
            path_available = False
            last_d = self.perception_provider.agent.last_action_params
            print("{} Path planning failed {}".format(self._agent_name,last_d))
            self.path = []
        

        if len(self.path) > 0 and self.goal_path:
            direction = self.path.pop(0)
            
        else:

            g_Y, g_X = np.where(agent_map == 7)
            if len(g_X) > 0 and len(g_X) < 11:
                target_goal = Position(g_X[-1],g_Y[-1])
                self.goal_path = True
                self.get_astar_path(agent_map,agent_location,target_goal)
            
            elif len(self.path) > 0:
                direction = self.path.pop(0)
                if self.check_obstacle_closeness(agent_map,agent_location) == False:
                    self.path = []


            else:
            #Check if unexplored available
                un_exp = np.where(agent_map == -1)
                self.goal_path = False
                # obs = np.where(agent_map == 1)
                if len(un_exp[0]) > 0:
                    self.get_path_unknown(agent_map,agent_location,last_d)
                    if len(self.path) > 0:
                        direction = self.path.pop(0)
            
            # #Obstacle based repulsion
            # elif len(obs[0]) > 0:
            #     left = agent_map[:,:agent_location.x - 1]
            #     right = agent_map[:,agent_location.x + 1]
            #     up = agent_map[:agent_location.y - 1,:]
            #     down = agent_map[agent_location.y + 1:,:]

            #     dist_list = []
            #     dist_list.append(get_inverse_distance_sum(left,agent_location))
            #     dist_list.append(get_inverse_distance_sum(right,agent_location))
            #     dist_list.append(get_inverse_distance_sum(up,agent_location))
            #     dist_list.append(get_inverse_distance_sum(down,agent_location))

            #     max_dist = max(dist_list)
            #     max_index = dist_list.index(max(dist_list)) #Get max of sum of inverse mh dist

            #     if max_dist >= 1:
            #         direction = self.opp_direction_dict[max_index]

            #     else:
            #         direction = self.direction_dict[max_index]  
        
        
        params = [KeyValue(key="direction", value=direction)]
        rospy.logdebug(self._agent_name + "::" + self._name + " executing move to " + str(params))
        action_generic_simple(publisher=self._pub_generic_action, action_type=GenericAction.ACTION_TYPE_MOVE, params=params)
     



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
        print("Explore called")
        self.direction_dict = {0: 'n',1: 's', 2: 'w', 3: 'e'}


    def get_direction(self, x,y,h,w,p_range,prev_param=[]):
        min_val= p_range
        random_move = ['n','e','w','s']

        direction = random.choice(random_move)

        if(abs((y - 1)) < min_val):
            min_val = abs((y - 1))
            direction = 'n'
        if(abs(h - 1 - (y + 1)) < min_val):
            min_val = abs(h - (y + 1))
            direction = 's'
        if(abs((x - 1)) < min_val):
            min_val = abs((x - 1))
            direction = 'w'
        if(abs(w -1 - (x + 1)) < min_val):
            min_val = abs(w - (x + 1))
            direction = 'e'
        
        if len(prev_param) > 0:
            check = True
            while check:
                if(direction in prev_param):
                    direction = random.choice(random_move)
                else:
                    check = False
        
        print("{} Going {}".format(self.perception_provider.agent.name, direction))
        
        return direction
            

            

    def do_step(self):
        
        print("{} Explore called".format(self._agent_name))

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
        
        # print("In explore")
        if len(self.perception_provider.relative_goals) < 12:
            if self.perception_provider.closest_goal:
                print("{} Found goal".format(self.perception_provider.agent.name))
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
        
        
        print("{} Dispense called".format(self._agent_name))
        print("{} blocks dispensed: {}".format(self._agent_name,self._perception_provider.sensor_dispensed_blocks._value))
        if self._perception_provider.closest_dispenser:
            

            direction = pos_to_direction(self._perception_provider.closest_dispenser.pos)
            # random_move = ['n', 's', 'e', 'w']
            # direction = random.choice(random_move)
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
        self.reached_loc = False

    def get_astar_path(self,agent_map,start,dest):
        
        grid_height, grid_width = agent_map.shape

        #Get location of obstacles
        obs_Y, obs_X = np.where(agent_map==1)
        ent_Y, ent_X = np.where(agent_map == 8)

        obs_Y = obs_Y.tolist() + ent_Y.tolist()
        obs_X = obs_X.tolist() + ent_X.tolist()
        #Make tuples
        obs_cood = []
        for i in range(len(obs_X)):
            obs_cood.append((obs_X[i],obs_Y[i]))
        
        obs_cood = tuple(obs_cood)

        grid = astar.AStar(grid_height,grid_width)
        grid.init_grid(obs_cood,(start.x,start.y),(dest.x,dest.y))
        self.path = grid.get_path()
        print(self.path)
    
    def do_step(self):

        print("{} Move to D called".format(self._agent_name))
        
        direction = ''

        
        if len(self.path) == 0 and self.reached_loc == False:
           
            target = self.perception_provider.target_cell
            agent_location = self.perception_provider.agent_location
            self.get_astar_path(self.perception_provider.local_map,agent_location,target)
            if len(self.path) > 0:
                direction = self.path.pop(0)

        elif self.reached_loc == False:
            direction = self.path.pop(0)

        else:
            self.reached_loc = True
            print("{} has reached disp".format(self._agent_name))

       


                
        if len(direction) > 0:
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

        self._perception_provider = perception_provider

        self._pub_generic_action = rospy.Publisher(get_bridge_topic_prefix(agent_name) + 'generic_action', GenericAction
                                                   , queue_size=10)

    def do_step(self):
        
        print("{} Attach called".format(self._agent_name))
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