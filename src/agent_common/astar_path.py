import agent_common.astar_3 as astar
import numpy as np

def get_astar_path(agent_map,agent_start,agent_dest, block_start = None, block_dest = None,flag=0,full_path=True):
        
        grid_height, grid_width = agent_map.shape
        # print("in path planner")
        #Get location of obstacles
        
        obs_Y, obs_X = np.where(agent_map == 1)
        ent_Y, ent_X = np.where(agent_map == 8)
        un_Y, un_X = np.where(agent_map == -1)

        obs_Y = obs_Y.tolist() + ent_Y.tolist() + un_Y.tolist()
        obs_X = obs_X.tolist() + ent_X.tolist() + un_X.tolist()



        #Make tuples
        obs_cood = []
        for i in range(len(obs_X)):
            obs_cood.append((obs_X[i],obs_Y[i]))

        
        obs_cood = tuple(obs_cood)
        # if (agent_dest.x,agent_dest.y) in obs_cood:
        #     obs_cood = list(obs_cood)
        #     obs_cood.remove((agent_dest.x,agent_dest.y))
        #     obs_cood = tuple(obs_cood)
        grid = astar.AStar(grid_height,grid_width)
        
        # if block_start is None:
        grid.init_grid(obs_cood,(agent_start.x,agent_start.y),(agent_dest.x,agent_dest.y))
        
        # print("Grid initialized")
        # else:
        #     grid.init_grid(obs_cood,(agent_start.x,agent_start.y),(agent_dest.x,agent_dest.y),
        #                   (block_start.x,block_start.y),(block_dest.x,block_dest.y))

        if full_path:
            path = grid.get_path(True)
        else:
            path = grid.get_path(False)
        # print("path obtained")
        return path