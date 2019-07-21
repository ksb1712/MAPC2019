import agent_common.astar_3 as astar
import numpy as np

def get_astar_path(agent_map,agent_start,agent_dest, block_start = None, block_dest = None,flag=0,full_path=True,allow_overflow=False):
        
        grid_height, grid_width = agent_map.shape
        # print("in path planner")
        #Get location of obstacles
        agent_map = agent_map.astype(int)
        ent_Y, ent_X = np.where(agent_map == 8)
        obs_Y, obs_X = np.where(agent_map == 1)
        un_Y, un_X = np.where(agent_map == -1)
        b_Y, b_X = np.where(agent_map == 10)


        if flag == 1:
            b_X = b_X.tolist()
            b_Y = b_Y.tolist()

            if block_start.x in b_X:
                if block_start.y in b_Y:
                    b_X.remove(block_start.x)
                    b_Y.remove(block_start.y)

            b_X = np.array(b_X)
            b_Y = np.array(b_Y)

        obs_Y = obs_Y.tolist() + ent_Y.tolist() + un_Y.tolist() + b_Y.tolist()
        obs_X = obs_X.tolist() + ent_X.tolist() + un_X.tolist() + b_X.tolist()



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
        if flag == 0:
            grid.init_grid(obs_cood,(agent_start.x,agent_start.y),(agent_dest.x,agent_dest.y),allow_overflow=allow_overflow)
        elif flag == 1:
            grid.init_grid(obs_cood,(agent_start.x,agent_start.y),(agent_dest.x,agent_dest.y),
                          (block_start.x,block_start.y),(block_dest.x,block_dest.y),flag=1)
        
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