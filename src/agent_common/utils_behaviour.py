import agent_common.astar as astar
import numpy as np


def get_astar_path(agent_map,start,dest):
        
        grid_height, grid_width = agent_map.shape

        #Get location of obstacles
            
        obs_Y, obs_X = np.where(agent_map == 1)
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
        path = grid.get_path()
        return path

def mh_distance(a,b):
        x_dist = abs(a.x - b.x)
        y_dist = abs(a.y - b.y)

        return x_dist + y_dist


def check_obstacle_closeness(agent_map,dest):
        
        range_ = 5
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


 def get_path_unknown(agent_map,agent_location,last_d):

        min_distance = 1000
        target_location = Position(agent_location.x,agent_location.y)
        
        #Find closest unexplored location

        u_Y, u_X = np.where(agent_map == -1)
        u_X = u_X.tolist()
        u_Y = u_Y.tolist()
        min_dist_array = [mh_distance(Position(i,j),agent_location) for i,j in zip(u_X,u_Y)]

        while len(min_dist_array) > 0:
            min_dist = min(min_dist_array)
            min_index = min_dist_array.index(min_dist)
            temp = Position(u_X[min_index],u_Y[min_index])
            if check_obstacle_closeness(agent_map,temp):
                min_dist_array.pop(min_index)
                u_X.pop(min_index)
                u_Y.pop(min_index)
            else:
                path = get_astar_path(agent_map,agent_location,temp)
                min_dist_array.pop(min_index)
                u_X.pop(min_index)
                u_Y.pop(min_index)


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

