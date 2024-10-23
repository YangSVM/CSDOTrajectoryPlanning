#!/usr/bin/env python3
# 东北天坐标系。后轴中心。x范围[0,100] y范围[0,100]
# 现在是后增加模式。是从原来已有的障碍物和车辆起止位置中
# 双层随机。栅格化，然后再在栅格里随机
# TODO 先让起点和终点不重合。这个之后再修改。因为CSDO算法现在还不支持(CL-CBS遗留下来的问题)。
# TODO 原来的benchmark中，存在大量的终点接近碰撞的场景。根本解不出来。
# 现在统一修改所有的场景...md
# function: generate_new
import yaml
import random
from math import pi, ceil, sin, cos
import os
import argparse
import numpy as np
# import tqdm


map_size = 50


grid_size = 2
v_size = int(map_size/grid_size)
num_agents = -1 # iterated by program.
num_maps = 60  # per setting.
num_obstacles = -1 # decided by map size

# Constants
class C:
    rv = 0.8

    car_width = 0 
    LF  = 0
    LB = 0
    f2x = 0
    r2x = 0
    
    r_obs = 0.8
    r_buffer =0.1
    
    def __init__(self) -> None:
        pass


def read_config_file():
    try:
        with open(os.path.abspath(os.getcwd())+
                "/config.yaml") as config_file:
            car_config = yaml.load(config_file, Loader=yaml.FullLoader)
            # global carWidth, LF, LB, obsRadius, framesPerMove
            C.car_width = car_config["carWidth"]

            C.LF = car_config["LF"]
            C.LB = car_config["LB"]
            C.car_width = car_config["carWidth"]
            
            # radius of vehicle
            C.rv = 1.0/2.0 *(  ( (C.LF + C.LB)** 2)/4 +  C.car_width**2 ) ** 0.5

            C.r_obs = car_config["obsRadius"]
            
            C.f2x  =  1/4.0 * (3.0*C.LF - C.LB)
            C.r2x = 1/4.0 * (C.LF - 3.0 * C.LB)

            print("LF", C.LF, "LB", C.LB, 
                  "car width", C.car_width, "rv:", C.rv)
    except IOError:
        # do things with your exception
        print("ERROR loading config file", os.path.abspath(os.getcwd())+
                "/src/config.yaml", " unable to generate scenarios")
    pass

class Obstacle:
    def __init__(self, x, y) :
        self.x = x
        self.y = y
        self.r = C.r_obs
    
    def obs_collision(self, obs):
        d = ((self.x - obs.x) ** 2 + ( self.y - obs.y)**2)**0.5
        return d <= self.r + obs.r
    
    def obstacles_collision(self, obstacles):
        for obs in obstacles:
            if self.obs_collision(obs):
                return True
        return False

# use double circle approximation to detect collision.
class State:
    def __init__(self,x,y,yaw) -> None:
        self.x = x 
        self.y = y 
        self.yaw = yaw
        
        self.xf = x + C.f2x * cos(yaw)
        self.xr = x + C.r2x * cos(yaw)
        self.yf = y + C.f2x * sin(yaw)
        self.yr = y + C.r2x * sin(yaw)
        
    def obs_collision(self, obs : Obstacle):
        d1 = ((self.xf - obs.x)**2 + (self.yf - obs.y)**2) ** 0.5
        d2 = ((self.xr - obs.x)**2 + (self.yr - obs.y)**2) ** 0.5

        if  d1 < obs.r + C.rv + C.r_buffer or d2 < obs.r + C.rv + C.r_buffer:
            return True
        return False
    
    def agent_collsion(self, s):

        r_c = C.rv * 2 + C.r_buffer
        d1 = ((self.xf - s.xf)**2 + (self.yf - s.yf)**2) ** 0.5 - r_c
        d2 = ((self.xr - s.xf)**2 + (self.yr - s.yf)**2) ** 0.5 - r_c
        d3 = ((self.xf - s.xr)**2 + (self.yf - s.yr)**2) ** 0.5 - r_c
        d4 = ((self.xr - s.xr)**2 + (self.yr - s.yr)**2) ** 0.5 - r_c
        if d1 <0 or d2 <0 or d3<0 or d4 < 0 :
            return True
        return False
    
    def state_valid(self, obstacles, agents):
        d =C.r_buffer + C.rv
        if self.xf < d or self.xf > map_size - d or \
            self.xr < d or self.xr > map_size - d or \
            self.yf < d or self.yf > map_size - d or \
            self.yr < d or self.yr > map_size - d:
                return False
            
        
        for obs in obstacles:
            if self.obs_collision(obs):
                return False
        
        for agent in agents:
            if self.agent_collsion(agent):
                return False
        
        return True

def generate_rand_state(visited):
    pos = np.random.random([v_size, v_size])
    pos[visited] = 0
    ind = np.unravel_index(np.argmax(pos), pos.shape)
    if visited[ind]:
        print("cannot generate cfg anymore")
        return None
    visited[ind] = 1
    d_half = 0.5 * grid_size
    x = random.uniform(-d_half, d_half)
    y = random.uniform(-d_half, d_half)
    theta = random.uniform(-pi, pi)
    return State(ind[0]*grid_size+x, grid_size* ind[1]+ y,theta)


def generate_random_obstacle():
    r = C.r_obs
    
    x = random.uniform(r ,  map_size - r)
    y = random.uniform(r ,  map_size - r)
    obs = Obstacle(x, y)
    return obs


def generate_legal_obstacles(n_cfg):
    obstalces = []
    iter_max = 10* num_obstacles
    while n_cfg > 0 and iter_max>0:
        obs = generate_random_obstacle()
        while obs.obstacles_collision(obstalces)  and iter_max >0:
            obs = generate_random_obstacle()
            iter_max -= 1
        n_cfg -= 1
        obstalces.append(obs)
    
    return obstalces

def generate_empty_visited():
    # one ceil one agent
    visited = np.zeros([v_size, v_size], dtype=bool)
    margin = ceil(C.rv/grid_size)
    # not allowed generate in margin
    visited[:, :margin] = 1
    visited[:, v_size - margin:] = 1
    visited[:margin, :] = 1
    visited[v_size - margin:, :] = 1
    return visited

def generate_legal_states(obs, agents, n_cfg, visited=None, cfgs=[]):
    if visited is None:
        visited = generate_empty_visited()

    # cfgs = []
    # # extract the existing cfgs
    # for agent in agents:
    #     if len(agent[key]) > 0:
    #         cfgs.append( agent[key] )
    new_states = []
    
    while n_cfg > 0:
        state = generate_rand_state(visited)
        if state is None:
            return new_states
   
        while not state.state_valid(obs, new_states + cfgs) :
            state = generate_rand_state(visited)
            if state is None:
                return new_states
        
        n_cfg -= 1
        new_states.append(state)
    
    return new_states


def dumpScenarios(map_fname, starts, goals, obs, n_real_agents):
    with open(map_fname, "w") as fmap:
        fmap.write("agents:\n")
        for i in range( n_real_agents ):
            fmap.write("  - start: [{:.2f}, {:.2f}, {:.2f}]\n".format(
                starts[i].x, starts[i].y, starts[i].yaw))
            fmap.write("    name: agent"+str(i)+"\n")
            fmap.write("    goal: [{:.2f}, {:.2f}, {:.2f}]\n".format(
                goals[i].x, goals[i].y, goals[i].yaw))
            
        fmap.write("map:\n")
        fmap.write("  dimensions: [{:d}, {:d}]\n".format(map_size, map_size) )
        fmap.write("  obstacles:\n")
        for ob in obs:
            fmap.write("  - [{:.2f}, {:.2f}, {:.2f}]\n".format(
                    ob.x, ob.y, ob.r
                ))



def generate_new_scenario( ):
    
    # with open(map_fname) as map_file:    
    #     map = yaml.load(map_file, Loader=yaml.FullLoader)
    # obs = map["map"]["obstacles"]
    # agents = map["agents"]
    obs  = generate_legal_obstacles(num_obstacles)
    agents = [{"start":[], "goal":[]}]
    
    n_cfg = num_agents
    # t0 = time.time()
    visited = generate_empty_visited()
    
    cfgs =[]
    starts = generate_legal_states(obs, agents, n_cfg, visited, cfgs)
    goals = generate_legal_states(obs, agents, n_cfg, visited, cfgs+starts)
    
    n_real_agents = min([len(starts), len(goals)])

    return starts, goals, obs
    
    
def test():
    goal = State(22.05, 42.70, 2.07)
    ob = Obstacle(21.96,45.78)
    collision = goal.obs_collision(ob)
    return collision

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map_path", default=None,
                        help="input file path containing map")
    args = parser.parse_args()
    
    read_config_file()
   
    num_obstacles_list = [ 0 ]
    if map_size == 100:
        num_agent_list = [25, 30, 35, 40, 50,60,70,80,90]  # for map 100 x 100. 
        num_obstacles_list.append(100)
    elif map_size == 50:
        num_agent_list = [5,10,15,20,25]  # for map 50 x 50. 
        num_obstacles_list.append(25) 
    
    for n_obs in num_obstacles_list:
        num_obstacles = n_obs
        for na in num_agent_list:
            # global num_agents
            num_agents = na
            
            iter_max = num_maps*2
            i = 0
            for iter in range(iter_max):
                if i >= num_maps:
                    print("generated", num_maps, "maps")
                    break

                # generate new scenario directly
                starts, goals, obs = generate_new_scenario()
                

                n_real_agents = min([len(starts), len(goals)])

                print("found ", n_real_agents, "agents!")
                if n_real_agents < na:
                    print("generate agents less than", na, ". try again.")
                    continue
                
                obs_str = ""
                if num_obstacles > 0 :
                    obs_str = "obstacle"
                else:
                    obs_str = "empty"
                # output_path = os.path.dirname(os.path.abspath(__file__))+\
                #     "/../benchmark/map%dby%d/agents%d/%s/"%(map_size, map_size, na, obs_str)
                
                # test output in build folder.
                output_path = os.path.dirname(os.path.abspath(__file__))+\
                    "/../build/benchmark/map%dby%d/agents%d/%s/"%(
                        map_size, map_size, na, obs_str)
                
                if not os.path.exists(output_path):
                    os.makedirs(output_path)
                    
                output_fname = output_path+\
                    "map_%dby%d_obst%d_agents%d_ex%d.yaml"%(
                        map_size, map_size, num_obstacles, na, i)

                dumpScenarios(output_fname, starts, goals, obs, na)
                i += 1

