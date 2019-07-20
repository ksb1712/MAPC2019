from __future__ import division  # force floating point division when using plain /
import rospy
import sys

from behaviour_components.sensors import Sensor
from rhbp_workspace.msg import Position, Dispenser, DLoc, Task
from agent_common.agent_utils import *
import numpy as np
from std_msgs.msg import String
import itertools
from agent_common.astar_path import *
import copy
# import matplotlib.animation as animation
# import matplotlib.pyplot as plt



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

        self.closest_dispenser_distance_sensor = Sensor(name="closest_dispenser_distance", initial_value=sys.maxint)
        self.dispenser_visible_sensor = Sensor(name="dispenser_visible", initial_value=False)
        self.dispenser_found = False
        self.dispenser_type = []

        self.obstacle_sensor = Sensor(name="obstacle_visible",initial_value=0)

        self.closest_goal = None
        self.goal_visible_sensor = Sensor(name="goal_visible",initial_value=False)
        self.count_goal_cells = Sensor(name="count_goal_cells",initial_value=0)
        self.closest_goal_distance_sensor = Sensor(name="closest_goal_distance_sensor",initial_value=sys.maxint)
        
        self.local_map = np.ones((self.perception_range*2 + 1,self.perception_range*2 + 1)) * -1

        self.goal_origin = None
        self.origin_found = False

        self.agent = None
        self.agent_location = Position(self.perception_range,self.perception_range) #Agent initial agent position to center of grid (vision range 5)

        self.pub_status = False

        self.data = DLoc()

        self.target_selected = False
        self.target_list = DLoc()
        self.target_cell = Position(0,0)
        self.target_selected_sensor = Sensor(name="target_selected",initial_value=False)
        
        self.is_task_selected = False
        self.selected_task = Task()

        rospy.Subscriber("dispenser_loc",DLoc,self.callback_disp_loc)
        rospy.Subscriber("target_dispenser",DLoc,self.callback_target_disp)
        rospy.Subscriber("task_selected",Task,self.callback_task_selected)

    

    def callback_target_disp(self,msg):
        # if not self.target_selected:
        # print(" *** in target call back *** ")
        targets = msg.dispensers
        for row in targets:
            if self.check_if_disp_exits(row,self.target_list):
                self.target_list.dispensers.append(row)

    def check_if_disp_exits(self,row,data):
        
        # print("In check: {}".format(len(data.dispensers)))
        for disp in data.dispensers:
            if row.pos.x == disp.pos.x:
                if row.pos.y == disp.pos.y:
                    if row.type == disp.type:
                        return False
        
        return True

    def callback_disp_loc(self,msg):
        
        # print("hello")
        # print(data.dispensers[0])
        # print(" ******* call back data: {}".format(len(data.dispensers)))

        # print("**in call **")

        # self.data.dispensers = []

        bf_l = len(self.data.dispensers)
        # print("Before Lenght: {}".format(bf_l))
        
        for row in msg.dispensers:
           if self.check_if_disp_exits(row,self.data):
                # print("Row: {}".format(row))
                self.data.dispensers.append(row)

        af_l = len(self.data.dispensers)
        # print("After Lenght: {}".format(af_l))

        # temp_set = set()
        # if af_l > bf_l:
        #     for row in self.data.dispensers:
        #         if row not in temp_set:
        #             temp_set.add(row)
            
        #     temp_set = list(temp_set)
        #     self.data.dispensers = []
        #     self.data.dispensers = temp_set


        # print("After After Lenght: {}".format(len(self.data.dispensers)))


        
        # print(self.data.dispensers)

    def count_seen_disp_type(self):
        count = []
        for row in self.data.dispensers:
            if row.type not in count:
                count.append(row.type)
        
        return len(count)

    def check_target_availability(self,x):
        
        target_type = []
        for row in self.target_list.dispensers:
            if row.pos == x.pos:
                return False
            if row.type not in target_type:
                target_type.append(row.type)

        if x.type in target_type:
            if len(target_type) < self.count_seen_disp_type():
                return False


        return True
        

   
    def _update_global_dispenser(self):
        
            # print("**call sub **")

        # if self.pub_status == False:
        for d_type in self.dispenser_type:
            d_Y, d_X = np.where(self.local_map == self.dispenser_type.index(d_type) + 2)
            for x,y in zip(d_X,d_Y):
                path_len = len(get_astar_path(self.local_map,self.goal_origin,Position(x,y)))
                rel_x = x - self.goal_origin.x
                rel_y = y - self.goal_origin.y
                temp_d = Dispenser(pos=Position(rel_x,rel_y),type=d_type)
                if self.check_if_disp_exits(temp_d,self.data):
                    # print("Temp_disp: {}".format(temp_d))
                    self.data.dispensers.append(temp_d)
        
        # new_list = set()
        # self.data.dispensers = set()
        # for row in self.data.dispensers:
        #     if row not in new_list:
        #         new_list.add(row)
        
        # self.data.dispensers = list(new_list)
        
        if len(self.data.dispensers) > 0:
            self.pub_status = True
            # l = len(self.data.dispensers)
            # if len(self.data.dispensers) > int(l/2):
            #     self.data.dispensers
            # print(data.dispensers[0])
            # print("publishing first time")
            pub = rospy.Publisher("dispenser_loc",DLoc,queue_size=1)
            pub.publish(self.data) 


        print("Dispensers found: {}".format(len(self.data.dispensers)))

        if not self.target_selected:
            print("target not selected")

            if len(self.target_list.dispensers) > 0:
                min_dist = 10000
                temp_cell = None
                for row in self.data.dispensers:
                    print("row: ",row.pos)
                    if self.check_target_availability(row):
                        dest = Position(row.pos.x + self.goal_origin.x, row.pos.y + self.goal_origin.y)
                        temp_dist = get_astar_path(self.local_map, self.agent_location, dest,full_path=False)
                        if len(temp_dist) < min_dist:
                            min_dist = len(temp_dist)
                            temp_cell = row.pos
                            print("Temp:cell: {}".format(temp_cell))

                        # self.target_list.dispensers.append(Dispenser(pos=Position(row.pos.x,row.pos.y),type=row.type))

                if temp_cell is not None:
                    print("** target selected other** ")

                    print(temp_cell)
                    self.target_cell.x = temp_cell.x
                    self.target_cell.y = temp_cell.y
                    self.target_selected_sensor.update(newValue=True)
                    self.target_selected_sensor.sync()
                    self.target_list.dispensers.append(Dispenser(pos=temp_cell,type=row.type))
                    self.target_selected = True
            
            else:
                min_dist = 10000
                temp_cell = None
                # dest_path = []
                for row in self.data.dispensers:
                    print("row: ",row.pos)
                    dest = Position(row.pos.x + self.goal_origin.x, row.pos.y + self.goal_origin.y)
                    temp_dist = get_astar_path(self.local_map, self.agent_location, dest,full_path=False)
                    # print("dist: {}".format(temp_dist))
                    if len(temp_dist) < min_dist:
                        min_dist = len(temp_dist)
                        temp_cell = row.pos
                        print("Temp:cell: {}".format(temp_cell))
                        # dest_path = temp_dist
                # print("** target selected ** ")
                if temp_cell is not None:
                    print("** target selected base** ")
                    print(temp_cell)
                    self.target_cell.x = temp_cell.x
                    self.target_cell.y = temp_cell.y
                    self.target_selected_sensor.update(newValue=True)
                    self.target_selected_sensor.sync()
                    self.target_list.dispensers.append(Dispenser(pos=temp_cell,type=row.type))
                    self.target_selected = True

                
                        
            print("Target Selection: {}".format(self.target_cell))
        if len(self.target_list.dispensers) > 0:
            # print("** publish target ** ")
            pub = rospy.Publisher("target_dispenser",DLoc,queue_size=1)
            pub.publish(self.target_list) 






    def update_perception(self, request_action_msg):
        """
        Has to be called on every simulation step and updated with the current request action message
        :param request_action_msg: request action message
        """

        self._request_action_msg = request_action_msg


        # if self.origin_found:
        #     pub = rospy.Publisher('origin_loc', String, queue_size=5)
        #     pub.publish(String("{},{},{}".format(self.agent.name,self.goal_origin.x,self.goal_origin.y)))
       
        if self.origin_found:
            self._update_global_dispenser()


        self.update_tasks(request_action_msg)

        self.agent = request_action_msg.agent

        self.entities = request_action_msg.entities

        self._update_dispensers(request_action_msg)

        self._update_blocks(request_action_msg)

        self._update_goal_cell(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        self._update_obstacles(request_action_msg)  # TODO this could be more sophisticated and potentially extracted like above

        print("Goal cells {}".format(len(self.relative_goals)))

        self.map_status()

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
                    duration = task.deadline
                    selected_task = task

        else:
            selected_task = self.selected_task

        pub = rospy.Publisher("task_selected",Task,queue_size=1)
        pub.publish(selected_task) 


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

            print("{} Origin found at : x: {}, y: {} relative to agent_origin".format(self.agent.name,x_min + 1,y_min + 1))
            
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
            print("{} says False".format(self.agent.name))
            if(len(self.relative_goals) >= 12):
                print("{} says Located".format(self.agent.name))
                self.get_goal_origin()
            else:
                print("{} says not yet".format(self.agent.name))
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
        print("Selected: ",self.target_selected)
        if self.target_selected:
            # target = Position(, )
            closest_distance = (abs(self.target_cell.x + self.goal_origin.x - self.agent_location.x) + 
                               abs(self.target_cell.y + self.goal_origin.y - self.agent_location.y))
           
            print("Target: {}, Distance: {}".format((self.target_cell.x + self.goal_origin.x,self.target_cell.y + self.goal_origin.y),closest_distance))
            # if closest_distance <= 1.0:
            #     self.closest_dispenser = target
        # for d in dispensers:
        #     if self.closest_dispenser is None or closest_distance > relative_euclidean_distance(d.pos):
        #         self.closest_dispenser = d
        #         closest_distance = relative_euclidean_distance(d.pos)

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
                    self.count_dispensed_blocks += 1
                    print("blocks updated")
                    self.sensor_dispensed_blocks.update(newValue=self.count_dispensed_blocks)
                    self.sensor_dispensed_blocks.sync()

            elif self.agent.last_action == "attach":
                if self.agent.last_action_result == "success":
                    self.count_attached_blocks += 1
                    self.sensor_attached_blocks.update(newValue=self.count_attached_blocks)
                    self.sensor_attached_blocks.sync()

                    self.count_dispensed_blocks -= 1
                    self.sensor_dispensed_blocks.update(newValue=self.count_dispensed_blocks)
                    self.sensor_dispensed_blocks.sync()

            
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

        #TODO set insert values to -1
        #Set all cell in current 5 x 5 to 0
        # Update map then


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
            # print("obstacle  x: {} y: {}".format(obstacle.pos.x,obstacle.pos.y))
            self.local_map[obstacle.pos.y + self.agent_location.y][obstacle.pos.x +  self.agent_location.x] = 1

        for entity in self.entities:
            self.local_map[entity.pos.y + self.agent_location.y][entity.pos.x +  self.agent_location.x] = 8

        
        for dispenser in self.dispensers:
            b_type = dispenser.type
            if b_type not in self.dispenser_type:
                self.dispenser_type.append(b_type)
            b_val = self.dispenser_type.index(b_type) + 2
            self.local_map[dispenser.pos.y +  self.agent_location.y][dispenser.pos.x +  self.agent_location.x] = b_val
            # print("dispenser x: {} y: {}".format(dispenser.pos.x,dispenser.pos.y))

        # for block in self.blocks:
        #     self.local_map[block.pos.y +  self.agent_location.y][block.pos.x + self.agent_location.x] = 3

        
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
                # print("Move to " + last_direction)

        #Change location
        #Get percept range and modify map accordingly
        #Change location if needed
        #Update map

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
        
        
        # print("action: {} status: {} direction: {}, shape: {}".format(self.agent.last_action, self.agent.last_action_result, last_direction,self.local_map.shape))
        # print("\n")
        self.local_map.dump('/ros/map/{}_map.npy'.format(self.agent.name))
        # data = self.local_map.ravel()
        # pub = rospy.Publisher('map_a1', numpy_msg(int32[]),queue_size=10)
        # rospy.init_node('talker', anonymous=True)
        # r = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():
        #     pub.publish(x)
        #     r.sleep()

        print("agent_provider_loc ({},{})".format(self.agent_location.x,self.agent_location.y))
        print(self.agent.last_action, self.agent.last_action_result)
        # # print(self.local_map)
        # # M = self.local_map * 10
        # # plt.pcolor( M , cmap = 'jet' )
        # # plt.savefig('./a.png')
        # print("\n\n")
