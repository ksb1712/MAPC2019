from __future__ import division  # force floating point division when using plain /
import rospy
import sys

from behaviour_components.sensors import Sensor
from rhbp_workspace.msg import Position, Dispenser, DLoc, Task, Reset
from agent_common.agent_utils import *
import numpy as np
from std_msgs.msg import String
import itertools
from agent_common.astar_path import *
import copy


class PerceptionProvider(object):
    """
    Objects that holds current perception, provides additional reasoning/postprocessing, and gives easy access to RHBP
    sensors
    """

    def __init__(self):


        #from mapc_rospy_bridge

        self.goals = []

        self.dispensers = []

        self.obstacles = []

        self.blocks = []

        self.entities = []

        self.tasks = []

        #Relative positions

        self.relative_goals = []

        self.count_attached_blocks = 0
        self.sensor_attached_blocks = Sensor(name="sensor_attached_blocks",initial_value=0)

        self.count_dispensed_blocks = 0
        self.sensor_dispensed_blocks = Sensor(name="sensor_dispensed_blocks", initial_value=0)  


        self.closest_dispenser = None

        self.closest_block = None 

        self.perception_range = 5

        self.closest_block_distance_sensor = Sensor(name="closest_block_distance", initial_value=sys.maxint)

        #for dispensing
        self.closest_dispenser_distance_sensor = Sensor(name="closest_dispenser_distance", initial_value=sys.maxint)
        self.dispenser_visible_sensor = Sensor(name="dispenser_visible", initial_value=False)
        self.dispenser_found = False
        self.dispenser_type = []

        self.obstacle_sensor = Sensor(name="obstacle_visible",initial_value=0)

        #for finding if 12 goal cells detected
        self.closest_goal = None
        self.goal_visible_sensor = Sensor(name="goal_visible",initial_value=False)
        self.count_goal_cells = Sensor(name="count_goal_cells",initial_value=0)
        self.closest_goal_distance_sensor = Sensor(name="closest_goal_distance_sensor",initial_value=sys.maxint)
        
        self.local_map = np.ones((self.perception_range*2 + 1,self.perception_range*2 + 1)) * -1

        self.goal_origin = None
        self.submit_origin = None
        self.origin_found = False

        self.agent = None
        self.agent_location = Position(self.perception_range,self.perception_range) #Agent initial agent position to center of grid (vision range 5)

        self.pub_status = False

        self.reset_timeout = 0

        #List of all detected dispensers by all agents
        self.data = DLoc()

        #for selecting dispenser based on task
        self.target_disp_selected = False
        self.target_dispenser_array = DLoc()
        self.target_dispenser_cell = Position(0,0)
        self.target_dispenser_selected_sensor = Sensor(name="dispenser_selected",initial_value=False)
        
        #for selecting submit cell based on task
        self.is_task_selected = False
        self.selected_task = Task()
        self.target_submit_array = DLoc()
        self.target_submit_cell = Position(0,0)
        self.target_submit_selected_sensor = Sensor(name="submit_selected",initial_value=False)

        self.submit_count = Sensor(name="submit_count",initial_value=0)
        self.is_submit_agent = False
        self.submit_sensor = Sensor(name="agent_submit",initial_value=0)

        self.team_score = 0
        self.score_sensor = Sensor(name="team_score",initial_value=0)


        self.reset = 0

        rospy.Subscriber("dispenser_loc",DLoc,self.callback_disp_loc)
        rospy.Subscriber("target_dispenser",DLoc,self.callback_target_disp)
        rospy.Subscriber("task_selected",Task,self.callback_task_selected)
        rospy.Subscriber("target_submit",DLoc,self.callback_target_submit)
        rospy.Subscriber("submit_origin",Position,self.callback_submit_origin)
        rospy.Subscriber("reset_val",Reset,self.callback_reset_all)
    
    def callback_reset_all(self,msg):
        """
        Reset all sensors if task submited
        *not working
        """
        if msg.val == 1:

            if self.reset_timeout < 5:
                self.reset = 1
                self.is_task_selected = False
                self.selected_task = Task()
                self.selected_task.name = ''
                self.is_submit_agent = False
                self.submit_sensor.update(newValue=0)
                self.submit_sensor.sync()
                self.count_attached_blocks = 0
                self.sensor_attached_blocks.update(newValue=self.count_attached_blocks)
                self.sensor_attached_blocks.sync()

                self.count_dispensed_blocks = 0
                self.sensor_dispensed_blocks.update(newValue=self.count_dispensed_blocks)
                self.sensor_dispensed_blocks.sync()

                self.target_submit_selected_sensor.update(newValue=False)
                self.target_submit_selected_sensor.sync()    
                self.target_dispenser_selected_sensor.update(newValue=False)
                self.target_dispenser_selected_sensor.sync()
                self.target_submit_array.dispensers *= 0
                self.target_dispenser_array.dispensers *= 0
                self.submit_origin = None
                self.reset_timeout += 1


                pub = rospy.Publisher("reset_val",Reset,queue_size=1)
                pub.publish(1)


                pub = rospy.Publisher("target_dispenser",DLoc,queue_size=1)
                pub.publish(self.target_dispenser_array) 
                pub = rospy.Publisher("submit_origin",Position,queue_size=1)
                pub.publish(Position())
                pub = rospy.Publisher("target_submit",DLoc,queue_size=1)
                pub.publish(self.target_submit_array)

                pub = rospy.Publisher("task_selected",Task,queue_size=1)
                pub.publish(self.selected_task) 

            else:
                self.reset = 0
                self.reset_timeout = 0
                pub = rospy.Publisher("reset_val",Reset,queue_size=1)
                pub.publish(self.reset)

        else:
            pub = rospy.Publisher("reset_val",Reset,queue_size=1)
            pub.publish(self.reset)


    def callback_target_submit(self,msg):
        """
        Update list of all submit cells from all agents
        to avoid conflicts
        """
        targets = msg.dispensers
        for row in targets:
            if self.check_if_disp_exits(row,self.target_submit_array):
                self.target_submit_array.dispensers.append(row)
    
    def callback_submit_origin(self,msg):
        """
        Only one submit origin per task
        """
        if self.submit_origin is None and self.origin_found:
            submit_origin = Position(msg.x,msg.y)


    def callback_target_disp(self,msg):
        """
        select dispenser for agent to connect to
        """
        targets = msg.dispensers
        for row in targets:
            if self.check_if_disp_exits(row,self.target_dispenser_array):
                self.target_dispenser_array.dispensers.append(row)

    def check_if_disp_exits(self,row,data):
        
        for disp in data.dispensers:
            if row.pos.x == disp.pos.x:
                if row.pos.y == disp.pos.y:
                    if row.type == disp.type:
                        return False
        
        return True

    def callback_disp_loc(self,msg):
        
       
        
        for row in msg.dispensers:
           if self.check_if_disp_exits(row,self.data):
                self.data.dispensers.append(row)

       

    def count_seen_disp_type(self):
        count = []
        for row in self.data.dispensers:
            if row.type not in count:
                count.append(row.type)
        
        return len(count)

    def check_target_availability(self,x):
        
        target_type = []
        for row in self.target_dispenser_array.dispensers:
            if row.pos == x.pos:
                return False
            if row.type not in target_type:
                target_type.append(row.type)

        if x.type in target_type:
            if len(target_type) < self.count_seen_disp_type():
                return False


        return True
        

   
    def _update_global_dispenser(self):
        
          
        for d_type in self.dispenser_type:
            d_Y, d_X = np.where(self.local_map == self.dispenser_type.index(d_type) + 2)
            for x,y in zip(d_X,d_Y):
                path_len = len(get_astar_path(self.local_map,self.goal_origin,Position(x,y),allow_overflow=True))
                rel_x = x - self.goal_origin.x
                rel_y = y - self.goal_origin.y
                temp_d = Dispenser(pos=Position(rel_x,rel_y),type=d_type,distance=path_len)
                if self.check_if_disp_exits(temp_d,self.data):
                    if path_len > 0:
                        self.data.dispensers.append(temp_d)
        
        
        
        if len(self.data.dispensers) > 0:
            self.pub_status = True
           
            pub = rospy.Publisher("dispenser_loc",DLoc,queue_size=1)
            pub.publish(self.data) 



     
        if self.is_task_selected:  #See if I have a task
            if not self.target_disp_selected: #See if I have alreay been allocated a task
                for row in self.selected_task.requirements: #Read requirements
                    if row.type in self.dispenser_type and not self.target_disp_selected: #See if I have seen the required dispenser type
                        if self.check_if_disp_exits(row,self.target_submit_array):  #See if the task has been allocated to some other agent
                            min_dist = 1000
                            temp_dispenser = None
                            for disp in self.data.dispensers:
                                if disp.type == row.type:
                                    if self.check_if_disp_exits(disp,self.target_dispenser_array): #See if dispenser is free
                                        if disp.distance < min_dist:
                                            path = get_astar_path(self.local_map,self.agent_location,Position(self.goal_origin.x + disp.pos.x,self.goal_origin.y + disp.pos.y))
                                            if len(path) > 0: #See if path exists or is not cut short by limiter
                                                min_dist = len(path)
                                                temp_dispenser = disp.pos
                            if temp_dispenser is not None:
                                if self.submit_origin is None:
                                    temp_org = self.find_submit_cell()
                                    self.submit_origin = Position(temp_org.x,temp_org.y)
                                if self.submit_origin is not None:
                                   
                                    self.target_dispenser_cell.x = temp_dispenser.x
                                    self.target_dispenser_cell.y = temp_dispenser.y
                                    self.target_dispenser_selected_sensor.update(newValue=True)
                                    self.target_dispenser_selected_sensor.sync()
                                    self.target_dispenser_array.dispensers.append(Dispenser(pos=temp_dispenser,type=row.type,distance=min_dist))
                                    self.target_disp_selected = True

                                    self.target_submit_cell.x = row.pos.x
                                    self.target_submit_cell.y = row.pos.y
                                    self.target_submit_array.dispensers.append(Dispenser(pos=row.pos,type=row.type,distance=0,agent=self.agent.name))


            if len(self.target_dispenser_array.dispensers) > 0:
                pub = rospy.Publisher("target_dispenser",DLoc,queue_size=1)
                pub.publish(self.target_dispenser_array) 
            if self.submit_origin is not None:
                pub = rospy.Publisher("submit_origin",Position,queue_size=1)
                pub.publish(self.submit_origin)
            if len(self.target_submit_array.dispensers) > 0:
                pub = rospy.Publisher("target_submit",DLoc,queue_size=1)
                pub.publish(self.target_submit_array)






    def update_perception(self, request_action_msg):
        """
        Has to be called on every simulation step and updated with the current request action message
        :param request_action_msg: request action message
        """

        self._request_action_msg = request_action_msg



        if self.reset == 1:
                pub = rospy.Publisher("reset_val",Reset,queue_size=1)
                pub.publish(1)


        self.agent = request_action_msg.agent

        self.entities = request_action_msg.entities

        self._update_dispensers(request_action_msg)

        self._update_blocks(request_action_msg)

        self._update_goal_cell(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        self._update_obstacles(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        if self.origin_found:
            self.update_tasks(request_action_msg)
            self._update_global_dispenser()
            self.update_score(request_action_msg)
            self.update_submit_ready()


        # self.update_tasks(request_action_msg)

        self.update_submit_ready()

        self.map_status()
    def update_score(self,msg):
        score = msg.team.score
        self.team_score = score
        self.score_sensor.update(newValue=score)

        self.score_sensor.sync()

    def update_submit_ready(self):
        if self.target_submit_selected_sensor._value:
            if self.local_map[self.agent_location.y + self.target_submit_cell.y][self.agent_location.x + self.target_submit_cell.x] == 10:
                self.submit_count.update(newValue=1)
                self.submit_count.sync()
            
            if self.is_submit_agent:
                self.submit_sensor.update(newValue=1)
                self.submit_sensor.sync()

    def callback_task_selected(self,msg):

        if len(msg.name) > 0:
            self.is_task_selected = True
            self.selected_task = msg

    
    def update_tasks(self,msg):
        
        self.tasks = msg.tasks
        duration = 0
        selected_task = Task()
        if not self.is_task_selected:
            for task in self.tasks:
                if task.deadline > duration:
                    for row in task.requirements:
                        for disp in self.data.dispensers:
                            if row.type == disp.type:
                                duration = task.deadline
                                selected_task = task

        else:
            selected_task = self.selected_task

        pub.publish(selected_task) 

    def check_if_cell_free(self,x,y):  
        H,W = self.local_map.shape
        x_left = max(0,x-3)
        x_right = min(W-1,x+3)
        y_top = max(0,y-3)
        y_bottom = min(H-1,y + 2)

        allowed_values = ['0','7','9']
        for i in range(y_top,y_bottom+1):
            for j in range(x_left,x_right+1):
                if self.local_map[i][j] not in allowed_values:     # If nearby cells are not free, goal or agent itself
                    return False
        
        return True

    def find_submit_cell(self):
        allowed_values = [0,7,9]
        if self.origin_found:
            g_Y, g_X = np.where(self.local_map == 7) #Get locations of goal cells
            for x,y in zip(g_X,g_Y):
                count = 0

                for row in self.selected_task.requirements:
                    if self.local_map[y + row.pos.y][x + row.pos.x] in allowed_values:                        
                        count += 1
                if count == len(self.selected_task.requirements):
                    return Position(x - self.goal_origin.x,y - self.goal_origin.y)

        return None

    def get_goal_origin(self):
        
        if(len(self.relative_goals) >= 12):
            self.origin_found = True
            x_min = sys.maxint
            y_min = sys.maxint

            for g in self.relative_goals:
                if g.x < x_min:
                    x_min = g.x
                if g.y < y_min:
                    y_min = g.y

            
            self.goal_origin = Position(x_min + 1, y_min + 1)

    def _update_goal_cell(self,request_action_msg):
        """
        Update self.goals
        Update count_goal_cells - sensor to indicate how many goal cells have
        been discovered
        """

        self.goals = request_action_msg.goals

        self.goal_visible_sensor.update(newValue=len(self.goals) > 0)
        self.count_goal_cells.update(newValue=len(self.relative_goals))

        self.count_goal_cells.sync()
        self.goal_visible_sensor.sync()



     

        if (self.origin_found == False):
            if(len(self.relative_goals) >= 12):
                self.get_goal_origin()
            else:
                self._update_closest_goal_cell(goals=self.goals)
        



    
    def _update_closest_goal_cell(self, goals):
        
        self.closest_goal = None
        closest_distance = sys.maxint

        for g in goals:
            if self.closest_goal is None or closest_distance > relative_euclidean_distance(g.pos):
                self.closest_goal = g
                closest_distance = relative_euclidean_distance(g.pos)

        self.closest_goal_distance_sensor.update(newValue=closest_distance)
        self.closest_goal_distance_sensor.sync()



    def _update_dispensers(self, request_action_msg):
        """
        Update dispenser perception
        :param request_action_msg: full current request action message object
        """

        self.dispensers = request_action_msg.dispensers
        
        if len(self.dispensers) > 0:
            self.dispenser_found = True
        self.dispenser_visible_sensor.update(newValue=self.dispenser_found)
        self.dispenser_visible_sensor.sync()

        self._update_closest_dispenser(dispensers=self.dispensers)

    def _update_closest_dispenser(self, dispensers):
        """
        Update information about the closest visible dispenser
        :param dispensers: dispensers perception
        """
        print("In distance")
        self.closest_dispenser = None
        closest_distance = sys.maxint
        if self.target_disp_selected:
            closest_distance = (abs(self.target_dispenser_cell.x + self.goal_origin.x - self.agent_location.x) + 
                               abs(self.target_dispenser_cell.y + self.goal_origin.y - self.agent_location.y))
       
        self.closest_dispenser_distance_sensor.update(newValue=closest_distance)
        self.closest_dispenser_distance_sensor.sync()

    def _update_blocks(self, request_action_msg):
        """
        Update block perception
        :param request_action_msg: full current request action message object
        """

        self.blocks = request_action_msg.blocks

        """
        blocks split as dispensed and attached
        for behaviour manipulation
        """
        if len(self.blocks) > 0:
            if self.agent.last_action == "request":
                if self.agent.last_action_result in ["success"]:
                    self.count_dispensed_blocks = 1
                    self.sensor_dispensed_blocks.update(newValue=self.count_dispensed_blocks)
                    self.sensor_dispensed_blocks.sync()

            elif self.agent.last_action == "attach":
                if self.agent.last_action_result == "success":
                    self.count_attached_blocks = 1
                    self.sensor_attached_blocks.update(newValue=self.count_attached_blocks)
                    self.sensor_attached_blocks.sync()

                    self.count_dispensed_blocks = 0
                    self.sensor_dispensed_blocks.update(newValue=self.count_dispensed_blocks)
                    self.sensor_dispensed_blocks.sync()

                    self.target_submit_selected_sensor.update(newValue=True)
                    self.target_submit_selected_sensor.sync()
            
            self._update_closest_block(blocks=self.blocks)



    def _update_closest_block(self, blocks):
        """
        Update information about the closest visible block
        :param blocks: blocks perception
        """

        self.closest_block = None
        closest_distance = sys.maxint

        for b in blocks:
            if self.closest_block is None or closest_distance > relative_euclidean_distance(b.pos):
                self.closest_block = b
                closest_distance = relative_euclidean_distance(b.pos)

        self.closest_block_distance_sensor.update(newValue=closest_distance)
        self.closest_block_distance_sensor.sync()


    def _update_obstacles(self, request_action_msg):
        """
        Update obstacle perception
        :param request_action_msg: full current request action message object
        """

        self.obstacles = request_action_msg.obstacles

        self.obstacle_sensor.update(newValue=len(self.obstacles) > 0)
        self.obstacle_sensor.sync()

   


    def check_vision_range(self,last_direction):


        x, y = self.local_map.shape
        if last_direction == "n":
            if self.agent_location.y <= self.perception_range:
                temp = [-1 for i in range(y)]
                self.local_map = np.insert(self.local_map,0,temp,axis=0)
                self.agent_location.y += 1
                if(self.origin_found):
                    self.goal_origin.y += 1
        
        elif last_direction == "s":
            if x - self.agent_location.y <= self.perception_range:
                temp = [-1 for i in range(y)]
                self.local_map = np.insert(self.local_map,x,temp,axis=0)
        
        elif last_direction == "e":
            if y - self.agent_location.x <= self.perception_range:
                temp = [-1 for i in range(x)]
                self.local_map = np.insert(self.local_map,y,temp,axis=1)

        elif last_direction == "w":
            if self.agent_location.x <= self.perception_range:
                temp = [-1 for i in range(x)]
                self.local_map = np.insert(self.local_map,0,temp,axis=1)
                self.agent_location.x += 1 
                if(self.origin_found):
                    self.goal_origin.x += 1

    def block_borders(self):
        H,W = self.local_map.shape


    def _update_map(self):
        

        
        x_min = max(self.agent_location.x - self.perception_range,0)
        y_min = max(self.agent_location.y - self.perception_range,0)

        x_max = x_min + self.perception_range * 2
        y_max = y_min + self.perception_range * 2
       
       #Assume all cells in 5 x 5 perception range as free
        for i,j in itertools.product(range(y_min,y_max),range(x_min,x_max)):
            if(abs(self.agent_location.y - i) + abs(self.agent_location.x -j) <= self.perception_range):
                self.local_map[i][j] = 0


        for goal in self.goals:
            x_loc = goal.pos.x + self.agent_location.x
            y_loc = goal.pos.y + self.agent_location.y
            self.local_map[y_loc][x_loc] = 7
            temp = Position(x_loc,y_loc)
            if temp not in self.relative_goals:
                self.relative_goals.append(temp)

        for obstacle in self.obstacles:
            self.local_map[obstacle.pos.y + self.agent_location.y][obstacle.pos.x +  self.agent_location.x] = 1

        for entity in self.entities:
            self.local_map[entity.pos.y + self.agent_location.y][entity.pos.x +  self.agent_location.x] = 8

        
        for dispenser in self.dispensers:
            b_type = dispenser.type
            if b_type not in self.dispenser_type:
                self.dispenser_type.append(b_type)
            b_val = self.dispenser_type.index(b_type) + 2
            self.local_map[dispenser.pos.y +  self.agent_location.y][dispenser.pos.x +  self.agent_location.x] = b_val
           
        for block in self.blocks:
            self.local_map[block.pos.y +  self.agent_location.y][block.pos.x + self.agent_location.x] = 10

        
        self.local_map[self.agent_location.y][self.agent_location.x] = 9




    

    
    
    
    def map_status(self):

        '''
        Update the local map (11 x 11) grid with agent at center. 
        Assumed vision range : 5

        x positive - right   (x of agent is y of map)
        y positive - up

        -1 - unknown
        0 - empty
        1 - obstacle
        2 - dispenser 1
        3 - disp 2 
        4 - disp 3 
        5 - disp 4
        6 - disp 5
        7 - goal
        8 - entity
        9 - agent
        10 - block

        todo:
        code based on dispenser type

        n - go up - agent.y decreases
        e - go left - agent.x increases

        '''

        """
        Identify the last successful move direction and change the map accordingly
        """
        last_direction = []
        if self.agent.last_action == "move":
            if self.agent.last_action_result == "success":
                last_direction = str(self.agent.last_action_params[0])
   

        if len(last_direction) > 0:
            self.local_map[self.agent_location.y][self.agent_location.x] = 0

            if last_direction == 'n':
                self.agent_location.y -= 1
               
            
            elif last_direction == 's':
                self.agent_location.y += 1

            elif last_direction == 'e':
                self.agent_location.x += 1

            elif last_direction == 'w':
                self.agent_location.x -= 1
                
            
            self.check_vision_range(last_direction)
        
        else:
            last_direction = "failed"

      

        
        self._update_map()
        
        
        self.local_map.dump('../map/{}_map.npy'.format(self.agent.name))
       
        if self.agent.last_action == "submit":
                if self.agent.last_action_result in ["success", "failed_target"] :
                    self.reset = 1
                    pub = rospy.Publisher("reset_val",Reset,queue_size=1)
                    pub.publish(1)  

                