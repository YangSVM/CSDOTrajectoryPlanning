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
from math import pi, ceil
import os
import argparse
import numpy as np
# import tqdm



# range of obstacle radius
r_obs_ = [-1,-1]  # load from config.yaml
num_agents = 100
map_size = 100

num_obstacles = 100

LF = -1  # load from config.yaml
CarWidth = -1  # load from config.yaml

grid_size = 2
v_size = int(map_size/grid_size)

# vehicle radius
rv = (LF**2 + CarWidth**2/4)
r_obs = 0.8

def generate_cfg(visited):
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
    return np.array([ind[0]*grid_size+x, grid_size* ind[1]+ y,theta])

# return true if collision.
def obsCollision(obs, cfg, cfg_type='agent'):
    # not so close. otherwise the hybrid a * cannot find the path.
    if cfg_type == 'agent':
        r_cfg = rv
    elif cfg_type == 'obstacle':
        r_cfg = cfg[2]
    for ob in obs:
        if len(ob) == 2:
            x, y = ob
            r = r_obs
        elif len(ob)==3:
            x, y, r = ob
        else:
            print( "unsupported format")
        if (cfg[0] - x)**2 + (cfg[1] -y)**2 - (r+ r_cfg)**2 <0 :
            return True 
    return False

# return true if collision.
def agentCollision(cfgs, cfg, cfg_type='agent'):
    # not so close. otherwise the hybrid a * cannot find the path.
    if cfg_type == 'agent':
        r_cfg = rv
    elif cfg_type == 'obstacle':
        r_cfg = cfg[2]
    for a in cfgs:
        x, y = a[0], a[1]
        if (cfg[0] - x)**2 + (cfg[1] -y)**2 - (rv + r_cfg)**2 <0 :
            return True 
    return False


def generate_obstacle_cfg():
    r = random.uniform(r_obs_[0], r_obs_[1])
    
    x = random.uniform(r ,  map_size - r)
    y = random.uniform(r ,  map_size - r)
    return np.array([x ,y, r])


def generate_legal_obstacles(n_cfg):
    cfgs = []
    iter_max = 10* num_obstacles
    while n_cfg > 0 and iter_max>0:
        cfg = generate_obstacle_cfg()
        while obsCollision(cfgs, cfg)  and iter_max >0:
            cfg = generate_obstacle_cfg()
            iter_max -= 1
        n_cfg -= 1
        cfgs.append(cfg)
    
    return cfgs

def generate_empty_visited():
    # one ceil one agent
    visited = np.zeros([v_size, v_size], dtype=bool)
    margin = ceil(rv/grid_size)
    # not allowed generate in margin
    visited[:, :margin] = 1
    visited[:, v_size - margin:] = 1
    visited[:margin, :] = 1
    visited[v_size - margin:, :] = 1
    return visited

def generate_legal_cfgs(obs, agents, n_cfg, visited=None, cfgs=[]):
    if visited is None:
        visited = generate_empty_visited()

    # cfgs = []
    # # extract the existing cfgs
    # for agent in agents:
    #     if len(agent[key]) > 0:
    #         cfgs.append( agent[key] )
    new_cfgs = []
    
    while n_cfg > 0:
        cfg = generate_cfg(visited)
        if cfg is None:
            return new_cfgs
   
        while obsCollision(obs, cfg) or agentCollision(cfgs + new_cfgs, cfg):
            cfg = generate_cfg(visited)
            if cfg is None:
                return new_cfgs
        
        n_cfg -= 1
        new_cfgs.append(cfg)
    
    return new_cfgs

# 
def dumpCBSScenarios(map_fname, starts, goals, obs, n_real_agents):
    pass

def dumpScenarios(map_fname, starts, goals, obs, n_real_agents):
    with open(map_fname, "w") as fmap:
        fmap.write("agents:\n")
        for i in range( n_real_agents ):
            fmap.write("  - start: [{:.2f}, {:.2f}, {:.2f}]\n".format(
                starts[i][0], starts[i][1], starts[i][2]))
            fmap.write("    name: agent"+str(i)+"\n")
            fmap.write("    goal: [{:.2f}, {:.2f}, {:.2f}]\n".format(
                goals[i][0], goals[i][1], goals[i][2]))
            
        fmap.write("map:\n")
        fmap.write("  dimensions: [{:d}, {:d}]\n".format(map_size, map_size) )
        fmap.write("  obstacles:\n")
        for ob in obs:
            if len(ob) == 3:
                fmap.write("  - [{:.2f}, {:.2f}, {:.2f}]\n".format(
                    ob[0], ob[1], ob[2]
                ))
            elif len(ob) ==2:
                fmap.write("  - [{:.2f}, {:.2f}]\n".format(
                    ob[0], ob[1]
                ))
            else:
                print( "unsupport format")

def extract_cfgs(agents, key):
    cfgs = [] 
    # extract the existing cfgs
    for agent in agents:
        if len(agent[key]) > 0:
            cfgs.append( agent[key] )
    return cfgs

# duplicated method
def add_agent2scenario(map_fname):
    
    with open(map_fname) as map_file:    
        map = yaml.load(map_file, Loader=yaml.FullLoader)
    obs = map["map"]["obstacles"]
    agents = map["agents"]
    
    n_cfg = num_agents - len(agents)
    # t0 = time.time()
    visited = generate_empty_visited()
    cfgs_start = extract_cfgs(agents, "start")
    cfgs_goal = extract_cfgs(agents, "goal")
    cfgs = cfgs_start + cfgs_goal
    starts = generate_legal_cfgs(obs, agents, n_cfg, visited,  cfgs)
    goals = generate_legal_cfgs(obs, agents, n_cfg, visited,  cfgs+starts)
    
    starts_all = cfgs_start + starts
    goals_all = cfgs_goal + goals

    # write as a new file
    return starts_all, goals_all, obs
    # 
        
    
    
    pass


def generate_new_scenario(map_fname):
    
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
    starts = generate_legal_cfgs(obs, agents, n_cfg, visited, cfgs)
    goals = generate_legal_cfgs(obs, agents, n_cfg, visited, cfgs+starts)
    
    n_real_agents = min([len(starts), len(goals)])
    # print("found ", n_real_agents, "agents!")
    # # write as a new file
    # dumpScenarios(map_fname, starts, goals, obs, n_real_agents)
    
    # 
    return starts, goals, obs
    
    

def load_gloabl_variable():
    # load the map arguments. global variable.
    try:
        with open(os.path.abspath(os.getcwd())+
                "/config.yaml") as config_file:
            carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
            # global carWidth, LF, LB, obsRadius, framesPerMove
            carWidth = carConfig["carWidth"]
            global LF, LB, rv, CarWidth, r_obs_
            LF = carConfig["LF"]
            LB = carConfig["LB"]
            CarWidth = carConfig["carWidth"]
            # large radius to avoid collision
            rv = (LF**2 + CarWidth**2/4)**0.5
            # 
            r_obs_[0] = carConfig["obsRadius"]
            r_obs_[1] = carConfig["obsRadius"]
            print("LF", LF, "LB", LB, "car width", carWidth, "rv:", rv)
    except IOError:
        # do things with your exception
        print("ERROR loading config file", os.path.abspath(os.getcwd())+
                "/src/config.yaml", " unable to generate scenarios")
        
        

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map_path", default=None,
                        help="input file path containing map")
    args = parser.parse_args()
    
    load_gloabl_variable()

    n_map = 60

    n_agent_file = 20
    n_obs_file = 50
    fnmae_prefix = "map_%dby%d_obst%d_agents%d_ex"%(map_size, map_size, n_obs_file, n_agent_file)
    
    # num_agent_list = [60,70,80,90,100]
    # num_agent_list = [25, 30, 35, 40]  # for map 50 x 50. append
    num_agent_list = [25, 30, 35, 40, 50,60,70,80,90]  # for map 100 x 100. new
    # num_agent_list = [20, 30, 40, 50, 60,70,80]  # for map 100 x 100. obstacle 200.
    # num_agent_list = [110, 120, 130, 140, 150] # for map 300 x 300
    for na in num_agent_list:
        # global num_agents
        num_agents = na
        
        iter_max = n_map*2
        i = 0
        for iter in range(iter_max):
            if i >= n_map:
                print("generated", n_map, "maps")
                break
            map_file = os.path.join(args.map_path, fnmae_prefix + str(i)+'.yaml')

            # append agent in an old configuration.
            # starts, goals, obs = add_agent2scenario(map_file)
            
            # generate new scenario directly
            starts, goals, obs = generate_new_scenario(map_file)
            

            n_real_agents = min([len(starts), len(goals)])

            print("found ", n_real_agents, "agents!")
            if n_real_agents < na:
                print("generate agents less than", na, ". try again.")
                continue
            
            obs_str = ""
            if num_obstacles > 0 :
                obs_str = "obstacle" + "100"
            else:
                obs_str = "empty"
            # output_path = os.path.dirname(os.path.abspath(__file__))+\
            #     "/../benchmark/map%dby%d/agents%d/%s/"%(map_size, map_size, na, obs_str)
            
            # test output in build folder.
            output_path = os.path.dirname(os.path.abspath(__file__))+\
                "/../build/benchmark/map%dby%d/agents%d/%s/"%(map_size, map_size, na, obs_str)
            
            if not os.path.exists(output_path):
                os.makedirs(output_path)
                
            output_fname = output_path+\
                "map_%dby%d_obst%d_agents%d_ex%d.yaml"%(map_size, map_size, num_obstacles, na, i)

            dumpScenarios(output_fname, starts, goals, obs, na)
            i += 1

