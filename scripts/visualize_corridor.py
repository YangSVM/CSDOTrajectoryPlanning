import numpy as np
import argparse
import yaml
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import os 

MAP_SIZE = 50
# dilated the corridor to check overlap preciously
# not dilated to check the initial guess valid.
DILATED = False

boundary = np.array([
    [0,0],
    [0,MAP_SIZE],
    [MAP_SIZE,MAP_SIZE],
    [MAP_SIZE,0],
    [0,0]
])
def updateBoundary(maxx, maxy):
    boundary[1, 1] = maxy
    boundary[2, 0] = maxx
    boundary[2, 1] = maxy
    boundary[3, 0] = maxx

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

# corridors: [2*Nt, 6]
# plot the original obstacles and the dilated corridors.
def plot_single_agent_corridors(obs, corridors, title=""):
        # plot simulation
        plt.clf()
        plt.plot(boundary[:,0], boundary[:,1], 'r-*')
        ax = plt.gca()

        # plot obstacles
        for obstacle in obs:
            ci = Circle(obstacle[:2], obstacle[2], color='gray')
            ax.add_patch(ci)
        
        # plot corridor
        cmap = get_cmap(len(corridors)//2+1)

        num_t = corridors.shape[0]
        for t in range(num_t):
            
            is_front =( t%2==0)
            if is_front: # front disc 
                line_style = '--'
                face_color = 'none'
                text_suffix = 'f'
            else: # rear disc
                line_style = '-'
                face_color = cmap(t//2+1)
                text_suffix = 'r'

            corridor = corridors[t, :]

            ci = Circle(corridor[:2], 0.1,  linestyle=line_style, 
                        ec=cmap(t//2+1), fc=face_color, label=str(t//2))
            ax.add_patch(ci)
            ax.text(corridor[0], corridor[1]-0.12, str(t//2)+text_suffix, fontsize=12)

            ri = Rectangle((corridor[2], corridor[4]), corridor[3] - corridor[2], 
                        corridor[5] - corridor[4], facecolor='none', edgecolor=cmap(t//2+1),
                        linestyle=line_style)
            ax.text(corridor[2], corridor[4]-0.12, str(t//2)+text_suffix, fontsize=12)

            ax.add_patch(ri)
        # plt.axes("equal")
        plt.title(title)
        plt.show()


def dilated_corridor(corridors: np.ndarray, rv = 1.25):
    dilated = np.zeros(corridors.shape)
    dilated[:, :, 2] = -rv
    dilated[:, :, 3] = rv
    dilated[:, :, 4] = -rv
    dilated[:, :, 5] = rv
    corridors = dilated + corridors
    return corridors

# translate corridors to [Na, 2*Nt, 6] numpy
def translate2np(corridors):
    Na = len(corridors)
    result = []
    for a in range(Na):
        traj = corridors["agent"+str(a)]
        traj_list = []
        for corridor in traj:
            traj_list.append(corridor)
        result.append(traj_list)
    result = np.array(result)
    return result

# 检测是否存在corridor面积为零的情况
def detectZeroAreaCorridor(corridors : np.ndarray):
    num_agent = corridors.shape[0]
    pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-c", "--corridor", default=None,
                        help="corridor file")

    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.corridor) as corridor_file:
        corridors = yaml.load(corridor_file, Loader=yaml.FullLoader)
    
    f_config_name = os.path.dirname(os.path.abspath(__file__))+"/../config.yaml"
    try:
        with open(f_config_name) as config_file:
            carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
            # global carWidth, LF, LB, obsRadius, framesPerMove
            carWidth = carConfig["carWidth"]
            # LF = float(carConfig["wheel_base"]) +float(carConfig["front_hang"])
            LF = float(carConfig["LF"])
            LB = carConfig["LB"]
            obsRadius = carConfig["obsRadius"]
            print("LF", LF, "LB", LB, "car width", carWidth)
    except IOError:
        # do things with your exception
        print("ERROR loading config file", f_config_name, ". using default param to plot")
    
    r_vehicle = (carWidth**2 + ((LF+LB)/2)**2) ** 0.5/2  # calculate the diameter and divide 2.
    
    maxx = map["map"]["dimensions"][0]
    maxy = map["map"]["dimensions"][1]
    updateBoundary(maxx, maxy)
    
    corridors = translate2np(corridors)
    if DILATED:
        corridors = dilated_corridor(corridors, r_vehicle)

    # plot all the circles
    obs = map["map"]['obstacles']
    obs = np.array(obs)
    if obs.shape[1] == 2:
        num_obs = obs.shape[0]
        obs_ = np.zeros([num_obs, 3])
        obs_[:, :2] = obs
        obs_[:, 2] = obsRadius
        obs = obs_

    gap = 1
    agent_id = 3
    title_text ="Agent %d with time interval %d Corridors"%(agent_id, gap)
    plot_single_agent_corridors(obs, corridors[agent_id,:12:gap,:], title_text)

    

