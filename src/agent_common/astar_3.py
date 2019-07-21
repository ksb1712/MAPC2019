import heapq
import numpy as np
import math

"""
X - left to right
Y - Top to bottom


"""


class Cell(object):
    def __init__(self, x, y, reachable,angle=0):
        """
        Initialize new cell

        @param x cell x coordinate
        @param y cell y coordinate
        @param reachable is cell reachable? not a wall?
        """
        self.reachable = reachable
        self.x = x
        self.y = y
        self.angle = angle
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

class AStar(object):
    def __init__(self,grid_height,grid_width):
        self.opened = []
        heapq.heapify(self.opened)
        self.closed = []
        self.cells = []
        self.grid_height = grid_height
        self.grid_width = grid_width
    
    def get_relative_orientation(self,agent, block):
        
        if agent.y > block.y:
            return 90

        elif agent.y < block.y:
            return 270
        
        elif agent.x > block.x:
            return 180
        
        else:
            return 0

    def print_cell(self,title,cell):
        print("{}: {},{},{}".format(title,cell.x,cell.y,cell.angle))

    def init_grid(self,obstacles,agent_start_cell,agent_end_cell,block_start_cell = (0,0),block_end_cell = (0,0),flag=0,allow_overflow=False):
        walls = obstacles # Tuple ((x,y),(x,y))
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable,0))

        self.allow_overflow = allow_overflow
        agent_start = self.get_cell(agent_start_cell[0],agent_start_cell[1]) #(x,y)
        agent_end = self.get_cell(agent_end_cell[0],agent_end_cell[1]) #(x,y)
        
        # self.print_cell("start",agent_start)

        self.is_same = True
        
        if flag != 0:
            block_start = self.get_cell(block_start_cell[0],block_start_cell[1]) #(x,y)
            self.start_angle = self.get_relative_orientation(agent_start,block_start)
            # print(start_angle)
            self.is_same = False
        else:
            # print("yes")
            self.start_angle = 0
        if flag != 0:    
            block_end = self.get_cell(block_end_cell[0],block_end_cell[1]) #(x,y)
            end_angle = self.get_relative_orientation(agent_end,block_end)
            self.is_same = False
        else:
            end_angle = 0
        # start_angle = self.get_relative_orientation(agent_start,block_start)
        # end_angle = self.get_relative_orientation(agent_end,block_end)

        # self.print_cell("block",block_start)

        # print("***** status *** {}".format(self.is_same))

        self.start = self.get_cell(agent_start_cell[0],agent_start_cell[1]) #(x,y)
        # print(start_angle)
        self.start.angle=self.start_angle
        # print(self.start.angle)
        self.update_angle(agent_start_cell[0],agent_start_cell[1],self.start_angle)
        # print(self.start.angle)
        self.end = self.get_cell(agent_end_cell[0],agent_end_cell[1]) #(x,y)
        self.end.angle=end_angle
        self.update_angle(agent_end_cell[0],agent_end_cell[1],end_angle)
        self.count = 0
        if flag == 1:
            print("start: {}".format((self.start.x,self.start.y,self.start.angle)))
            print("end: {}".format((self.end.x,self.end.y,self.end.angle)))

        # print("start orientation: {} \n end orienteation: {}".format(self.start.angle,self.end.angle))

    def get_cost(self,cell,adj_cell):

        cost = 10
        # print((cell.x,cell.y,cell.angle),(adj_cell.x,adj_cell.y,adj_cell.angle))
        if adj_cell.angle != cell.angle:
            cost += 10
        
        return cost



    def get_heuristic(self, cell):
        """
        Compute the heuristic value H for a cell: distance between
        this cell and the ending cell multiply by 10.

        @param cell
        @returns heuristic value H
        """
        # print("angle cell {}, end: {}".format(cell.angle,self.end.angle))
        d1 =  10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))
        r1 =  abs(cell.angle - self.end.angle)

        # print("d1: {} r1 {}".format(d1,r1))

        return d1 + r1

    def update_angle(self,x,y,angle):

        """
        update angle for rotaion
        """
        self.cells[x * self.grid_height + y].angle = angle

    def get_cell(self, x, y):
        """
        Returns a cell from the cells list

        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        return self.cells[x * self.grid_height + y]

    def update_cell(self, adj, cell):
        """
        Update adjacent cell

        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        if adj.angle != cell.angle:
            if self.check_cell_similarity(cell,self.start) == False:
                self.update_angle(cell.x,cell.y,cell.angle)
        adj.g = self.get_cost(cell,adj)
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g
    
    def get_adjacent_cells(self, cell):
        """
        Returns adjacent cells to a cell. Clockwise starting
        from the one on the right.

        @param cell get adjacent cells for this cell
        @returns adjacent cells list 
        """
        cells = []
        # print("curr cell: {}".format((cell.x,cell.y,cell.angle)))
        self.start.angle = self.start_angle
        self.count += 1
        """
        cell limits
        """

        if self.is_same:
            x_left = x_right = cell.x
            y_top = y_low = cell.y
            y_length = 0
            x_length = 0
            y_sign = 0
            x_sign = 0
        

        else:
            if cell.angle == 0:
                x_sign = 1
                y_sign = 0
                x_length = 1
                y_length = 0
            elif cell.angle == 90:
                x_sign = 0
                y_sign = -1
                x_length = 0
                y_length = -1
            elif cell.angle == 180:
                x_sign = -1
                y_sign = 0
                x_length = -1
                y_length = 0
            
            else:
                x_sign = 0
                y_sign = 1
                x_length = 0
                y_length = 1
            
            x_left = min(cell.x,cell.x + x_sign)
            x_right = max(cell.x,cell.x + x_sign)

            y_top = min(cell.y, cell.y + y_sign)
            y_low = max(cell.y, cell.y + y_sign)



        # s = self.get_cell(2,1)
        # print(s.reachable)
            

        # rot_x = cell.x + self.len_vertical * int(math.cos(math.radians(self.angle + 90)))

        # if x_left < self.grid_width - 1:

        # if not self.is_same:
        #     length = 1
        
        # else:
        #     length = 2

        if x_right < self.grid_width-1:
            if x_right != cell.x:
                temp_cell = self.get_cell(cell.x+ 1 + x_length,cell.y + y_length)
            else:
                temp_cell = self.get_cell(cell.x+ 1,cell.y + y_length)

            if temp_cell.reachable:
                cells.append(Cell(cell.x+1, cell.y,temp_cell.reachable,cell.angle))
       
        if y_top > 0:
            if y_top != cell.y:
                temp_cell = self.get_cell(cell.x + x_length,cell.y-1-y_length)
            else:
                temp_cell = self.get_cell(cell.x + x_length,cell.y-1)
            if temp_cell.reachable: 
                cells.append(Cell(cell.x, cell.y-1,temp_cell.reachable,cell.angle))
       
        if x_left > 0:
            if x_left != cell.x:
                temp_cell = self.get_cell(cell.x -1 - x_length,cell.y + y_length)
            else:
                temp_cell = self.get_cell(cell.x -1,cell.y + y_length)
            if temp_cell.reachable: 
                cells.append(Cell(cell.x-1, cell.y,temp_cell.reachable,cell.angle))
       
        if y_low < self.grid_height-1:
            if y_low != cell.y:
                temp_cell = self.get_cell(cell.x + x_length,cell.y +1 + y_length)
            else:
                temp_cell = self.get_cell(cell.x + x_length,cell.y +1)
            # print("Update: {},{}".format(temp_cell.y,temp_cell.x))

            # print("reachable: ",temp_cell.reachable, temp_cell.x, temp_cell.y)

            if temp_cell.reachable: 
                cells.append(Cell(cell.x, cell.y+1,temp_cell.reachable,cell.angle))

        
        # temp_cell = cell.deepcopy()

        if self.is_same == False:
            if y_sign == 0:
                if cell.y < self.grid_height - 1:
                    if self.get_cell(cell.x,cell.y + 1).reachable:
                        # temp_cell.angle = 270
                        # print("changed: {}".format(self.get_cell(cell.x,cell.y).angle))
                        # temp = Cell(temp_cell.x,temp_cell.y,temp_cell.reachable,temp_cell.angle)
                        cells.append(Cell(cell.x,cell.y,cell.reachable,270))             
                if cell.y > 0:
                    if self.get_cell(cell.x,cell.y - 1).reachable:
                        # temp_cell.angle = 90
                        cells.append(Cell(cell.x,cell.y,cell.reachable,90))             
            
            if x_sign == 0:
                
                if cell.x < self.grid_width - 1:
                    if self.get_cell(cell.x + 1,cell.y).reachable:
                        # temp_cell.angle = 0
                        cells.append(Cell(cell.x,cell.y,cell.reachable,0))             
                    
                if cell.x > 0:    
                    if self.get_cell(cell.x-1,cell.y).reachable:
                        # temp_cell.angle = 180
                        cells.append(Cell(cell.x,cell.y,cell.reachable,180))             



        # print("Num of neighbours: {}".format(len(cells)))
        return cells

    
    def get_direction(self,cur_cell,nxt_cell):
        """
        Get the direction to take (n,s,e,w) to reach
        the next cell
        """

        if nxt_cell.x > cur_cell.x:
            return 'e'
        if nxt_cell.x < cur_cell.x:
            return 'w'
        if nxt_cell.y > cur_cell.y:
            return 's'
        if nxt_cell.y < cur_cell.y:
            return 'n'
        
        if self.is_same:
            return 'n'

        if nxt_cell.angle == 0:
            if cur_cell.angle == 270:
                return 'ccw'
        if cur_cell.angle == 0:
            if nxt_cell.angle == 270:
                return 'cw'
        
        if nxt_cell.angle > cur_cell.angle:
            return 'ccw'
        
        if nxt_cell.angle < cur_cell.angle:
            return 'cw'
    
    def check_cell_similarity(self,a,b):
        if a.x == b.x:
            if a.y == b.y:
                if a.angle == b.angle:
                    return True

        return False

    def return_path(self,cell,full_path=False):
        # cell = self.end
        path = []
        # print(cell.x,cell.y,cell.parent.x,cell.parent.y)
        if full_path == False and cell.parent is not None:
            cell = cell.parent
        # print("Start: {}".format((self.start.x,self.start.y,self.start.angle)))
        while self.check_cell_similarity(cell,self.start) == False:
            if cell is not None:
                # print 'path: cell: %d,%d,%d' % (cell.x, cell.y,cell.angle)
            
                path.append(self.get_direction(cell.parent,cell))
                cell = cell.parent
            # path.append(self.get_direction(parent,cell))
        path.reverse()
        if path is None:
            path = []
        return path


    def get_path(self,full_path = False):
    # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        path = []
        # count = 0
        while len(self.opened):
            
            # count = len(self.closed)
            # pop cell from heap queue 
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            # self.closed.add(cell)
            self.closed.append(cell)
            # print self.closed
            # print("closed: {}".format(len(self.closed)))
            # if ending cell, display found path
            # print(cell.x,cell.y,cell.angle,cell.h)
            if (cell.x == self.end.x and cell.y == self.end.y and cell.h == 0) or self.count > 100:
                # print("End is {}".format((cell.x,cell.y,cell.angle)))
                path = self.return_path(cell,full_path)

                if self.count > 100:
                    print("overflow",self.allow_overflow)
                    if self.allow_overflow:
                        path=[]
                # else:
                
                if path is None:
                    path = []

                return path
                break
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            # print("adj cells {}".format(len(adj_cells)))
            for adj_cell in adj_cells:
                
                temp = self.get_cell(adj_cell.x,adj_cell.y)

                # print("adj: {},{},{}, {}, cost: {}, heuristic: {}".format(adj_cell.x,adj_cell.y,adj_cell.angle,adj_cell.reachable,self.get_cost(cell,adj_cell),self.get_heuristic(adj_cell)))
                adj_set = (adj_cell.x,adj_cell.y,adj_cell.angle)
                # if adj_set in self.closed:
                #     print("adj {} closed".format((adj_cell.x,adj_cell.y)))
                
                status = True
                for item in self.closed:
                    if self.check_cell_similarity(adj_cell,item):
                        status=False
                        break

                if temp.reachable and status:
                    # print("reachable")
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found for this adj
                        # cell.
                        if adj_cell.g > cell.g + self.get_cost(cell,adj_cell):
                            # print("in update")
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)

                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))
            
            

