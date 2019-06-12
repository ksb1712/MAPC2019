import math
import heapq



"""


Bugged. Check again
"""

class Cell(object):
    def __init__(self,x,y, reachable,rot_direction=0):

        self.reachable = reachable
        self.x = x
        self.y = y
        self.rot_direction = rot_direction
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0
    

class Astar(object):
    def __init__(self,grid_height,grid_width):
        self.opened = []
        heapq.heapify(self.opened)

        self.closed = []
        self.cells = []
        self.grid_height = grid_height
        self.grid_width = grid_width
    

    def init_grid(self,obstacles,start_cell,end_cell,dest_cell):
        walls = obstacles
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable))
        self.start = self.get_cell(start_cell[0],start_cell[1]) #(x,y)
        self.end = self.get_cell(end_cell[0],end_cell[1])
        self.dest = self.get_cell(dest_cell[0],dest_cell[1]) #(x,y)
        self.len_horizontal = self.end.x - self.start.x
        self.len_vertical = self.end.y - self.start.y
        self.angle = 0
    
    def get_cost(self, cell):
        """
        Compute the heuristic value H for a cell: distance between
        this cell and the ending cell multiply by 10.

        @param cell
        @returns heuristic value H
        """
      
        return 10 * (abs(cell.x - self.dest.x) + abs(cell.y - self.dest.y))

    def get_cell(self, x, y):
        """
        Returns a cell from the cells list

        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        return self.cells[x * self.grid_height + y]

    def update_cell(self, adj, cell,rotation=False):
        """
        Update adjacent cell

        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        weight = 10
        if rotation:
            weight = 15
        adj.g = cell.g + weight
        adj.h = self.get_cost(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    
    def get_adjacent_cells(self, cell, obj_len=1):
        """
        Returns adjacent cells to a cell. Clockwise starting
        from the one on the right.

        @param cell get adjacent cells for this cell
        @returns adjacent cells list 
        """
        cells = []
        if cell.x < min(self.grid_width - 1, self.grid_width-1 - self.len_horizontal):
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y - self.len_vertical> 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x - self.len_horizontal> 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.grid_height-1 - self.len_vertical:
            cells.append(self.get_cell(cell.x, cell.y+1))
        
        rot_x = cell.x + self.len_vertical * int(math.cos(math.radians(self.angle + 90)))
        rot_y = cell.y + self.len_horizontal * int(math.sin(math.radians(self.angle + 90)))

        if rot_x >= 0 and rot_x < self.grid_width:
            if rot_y >= 0 and rot_y < self.grid_height:
                temp = self.get_cell(rot_x,rot_y)
                temp.rot_direction = 1 #CCW
                cells.append(temp)
        

        rot_x = cell.x + int(math.cos(math.radians(self.angle - 90)))
        rot_y = cell.y - int(math.sin(math.radians(self.angle - 90)))

        if rot_x >= 0 and rot_x < self.grid_width:
            if rot_y >= 0 and rot_y < self.grid_height:
                temp = self.get_cell(rot_x,rot_y)
                temp.rot_direction = -1 #CW
                cells.append(temp)
        

        return cells
    
    def get_direction(self,cur_cell,nxt_cell):
        """
        Get the direction to take (n,s,e,w) to reach
        the next cell
        """
        if nxt_cell.rot_direction == 1:
            return 'ccw'
        elif nxt_cell.rot_direction == -1:
            return 'cw'
        elif nxt_cell.x > cur_cell.x:
            return 'e'
        elif nxt_cell.x < cur_cell.x:
            return 'w'
        elif nxt_cell.y > cur_cell.y:
            return 's'
        else:
            return 'n'

    def return_path(self):
        cell = self.dest
        path = []
        while cell is not self.start:
            path.append(self.get_direction(cell.parent,cell))
            cell = cell.parent
            print 'path: cell: %d,%d' % (cell.x, cell.y)
            # path.append(self.get_direction(cell.parent,cell))
        path.reverse()
        return path

    def get_path(self):
    # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        path = []
        while len(self.opened):
            # pop cell from heap queue 
            f, cell = heapq.heappop(self.opened)
            # print("cell processed: ({},{})".format(cell.x,cell.y))
            # add cell to closed list so we don't process it twice
            self.closed.append(cell)
            # if ending cell, display found path
            if cell is self.dest:
                path = self.return_path()
                print(path)
                break
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found for this adj
                        # cell.
                        weight = 10
                        # rotation=False
                        # if adj_cell.rot_direction != 0:
                        #     weight = 15
                        #     rotation=True
                        if adj_cell.g > cell.g + weight:
                            self.update_cell(adj_cell, cell)
                    else:
                        # rotation=False
                        # if adj_cell.rot_direction != 0:
                        #     rotation=True
                        #     if adj_cell.rot_direction == 1:
                        #         self.angle += 90
                        #     else:
                        #         self.angle -= 90
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))
