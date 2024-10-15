#!/usr/bin/env python3
# python3 scripts/visualize.py -m [path/to/map_file.yaml] -s [path/to/schedule_file.yaml] 
# If donot input the schedule file, it will plot the map only.

from matplotlib import animation
import matplotlib.animation as manimation
from matplotlib.patches import Circle, Rectangle, Arrow
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt
import math
import os
import argparse
import numpy as np
import yaml
import matplotlib
matplotlib.use("Qt5Agg")

from collision_detection import collision_rect_and_rect, collision_circle_and_rect
from collision_geometry import Rectangle as Rect
from collision_geometry import Circle as CircleColli
PLOTLINE = True  # True

carWidth = 2.0
LF = 2.0
LB = 1.0
obsRadius = 1
framesPerMove = 1

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.rcParams["font.family"] = "Times New Roman"  # global setting

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

# 碰撞检测
def check_collision(pi, pj):
    # 
    rect1 = Rect(pi[0], pi[1], pi[2], length=LF+LB, width=carWidth, pos='rear_axle_center', lb=LB)
    rect2 = Rect(pj[0], pj[1], pj[2], length=LF+LB, width=carWidth, pos='rear_axle_center', lb=LB)
    return collision_rect_and_rect(rect1, rect2)

def check_obs_collision(p, obs):
    rect = Rect(p[0], p[1], p[2], length=LF+LB, width=carWidth, pos='rear_axle_center', lb=LB)
    if len(obs) == 2:
        obs_circle = CircleColli(obs[0], obs[1], obsRadius)
    elif len(obs) == 3:
        obs_circle = CircleColli(obs[0], obs[1], obs[2])
    return collision_circle_and_rect(obs_circle, rect)

def calc_corner_center(x, y, yaw, half_len, half_wid):
    r = (half_len**2 + half_wid**2)**0.5
    dyaw = math.atan2(half_len, half_wid)
    pos =[0,0]
    pos[0] = x + r*math.sin(-dyaw + yaw)
    pos[1] = y - r*math.cos(-dyaw + yaw)
    return pos

def gen_wheels(x, y, yaw, steer, half_len, half_wid, lf, lb,cw):
    wheel_names = ["rear_left", "rear_right", "front_left", "front_right"]
    wps = { name:[0,0] for name in wheel_names} # wheel positions

    wps['rear_left'][0] = x + cw/2*math.cos(yaw + math.pi/2)
    wps['rear_left'][1] = y + cw/2*math.sin(yaw + math.pi/2)
    
    wps['rear_right'][0] = x + cw/2*math.cos(yaw - math.pi/2)
    wps['rear_right'][1] = y + cw/2*math.sin(yaw - math.pi/2)    
    
    # position of front-axis-center
    px= x + lf*math.cos(yaw )
    py = y + lf*math.sin(yaw)
    
    wps['front_left'][0] = px + cw/2*math.cos(yaw + math.pi/2)
    wps['front_left'][1] = py + cw/2*math.sin(yaw + math.pi/2)
    
    wps['front_right'][0] = px + cw/2*math.cos(yaw - math.pi/2)
    wps['front_right'][1] = py + cw/2*math.sin(yaw - math.pi/2)
    
    wheels =[]
    for wp_name in wps:
        wp = wps[wp_name]
        pos = calc_corner_center(wp[0], wp[1], yaw, half_len, half_wid)
        wheel_yaw = yaw
        if 'front' in wp_name:
            wheel_yaw += steer
        wheel = Rectangle((pos[0], pos[1]), half_len*2, half_wid*2, 
                  wheel_yaw/math.pi*180, facecolor='black')
        wheels.append(wheel)
        
    return wheels


class Animation:
    def __init__(self, map, schedule):
        self.map = map
        self.schedule = schedule

        aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

        self.fig = plt.figure(frameon=False, figsize=(8 * aspect, 8))
        self.ax = self.fig.add_subplot(111, aspect='equal')
        self.fig.subplots_adjust(
            left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
        # self.ax.set_frame_on(False)

        self.patches = []
        self.artists = []
        self.lines = []
        self.wheels =[]
        self.list_xdata = [[] for _ in range(0, len(map["agents"]))]
        self.list_ydata = [[] for _ in range(0, len(map["agents"]))]
        self.agents = dict()
        self.agent_names = dict()
        # create boundary patch
        xmin = -1
        ymin = -1
        xmax = map["map"]["dimensions"][0] + 0.5
        ymax = map["map"]["dimensions"][1] + 0.5

        # self.ax.relim()
        plt.xlim(xmin-2, xmax+2)
        plt.ylim(ymin-2, ymax+2)
        # plt.xlim(10, 20)
        # plt.ylim(10, 20)
        # self.ax.set_xticks([])
        # self.ax.set_yticks([])
        # plt.axis('off')
        # self.ax.axis('tight')
        self.ax.axis('off')

        self.patches.append(Rectangle(
            (xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='black', lw=2))
        obs_index = 0
        for o in map["map"]["obstacles"]:
            x, y = o[0], o[1]
            if len(o) == 3:
                r = o[2]
            elif len(o) == 2:
                r = obsRadius
            self.patches.append(
                Circle((x, y), r, facecolor='grey', edgecolor='grey'))
            self.ax.text(x, y, str(obs_index), fontsize=6)
            obs_index += 1
            # Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

        # create agents:
        self.T = 0
        cmap = get_cmap(len(map["agents"])+1)
        # draw goals first
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            cw = carWidth
            lb = LB
            lf = LF
            self.patches.append(Rectangle(
                (d["goal"][0] + math.sqrt(cw/2*cw/2+lb*lb) * math.sin(-math.atan2(lb, cw/2)+d["goal"][2]),
                 d["goal"][1] - math.sqrt(cw/2*cw/2+lb*lb) * math.cos(-math.atan2(lb, cw/2)+d["goal"][2])),
                lb+lf, cw, d["goal"][2] / math.pi * 180,
                facecolor='none', edgecolor=cmap(i+1),  alpha=0.7, lw=1, linestyle=":"))
            self.list_xdata[i].append(d["start"][0])
            self.list_ydata[i].append(d["start"][1])
            line, = self.ax.plot(
                self.list_xdata[i], self.list_ydata[i], color=cmap(i+1),  alpha=0.3, lw=1.5, linestyle="dotted")
            self.lines.append(line)

        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            name = d["name"]
            self.agents[name] = Rectangle(
                (d["start"][0], d["start"][1]), 1, 2, 0.0, edgecolor=cmap(i+1), facecolor='1',alpha=0.7)
            self.agents[name].original_face_color = cmap(i+1)
            self.patches.append(self.agents[name])
            if schedule["schedule"][name] is None:
                continue
            self.T = max(self.T, schedule["schedule"][name][-1]["t"])
            self.agent_names[name] = self.ax.text(
                d["start"][0], d["start"][1], name.replace('agent', ''), fontsize=6)
            self.agent_names[name].set_horizontalalignment('center')
            self.agent_names[name].set_verticalalignment('center')
            self.artists.append(self.agent_names[name])

        # for a in range(num_agents):
        #     start = map["agents"][a]['start']
        #     steer0 = schedule['agent'+str(a)][0]['steer']/180*math.pi
        #     wheels_a = gen_wheels(start[0], start[1], start[2], steer0, lb/4, lb/8, lf, lb, cw)       
        #     self.wheels += wheels_a
        # self.ax.set_axis_off()
        # self.fig.axes[0].set_visible(False)
        # self.fig.axes.get_yaxis().set_visible(False)

        # self.fig.tight_layout()
        self.anim = animation.FuncAnimation(self.fig, self.animate_func,
                                            init_func=self.init_func,
                                            frames=int(self.T+1) *
                                            framesPerMove,
                                            interval=100,
                                            repeat=False,
                                            blit=True)

    def save(self, file_name, speed):
        self.anim.save(
            file_name,
            "ffmpeg",
            fps=10 * speed,
            dpi=300),
        # savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"})

        '''uncomment if want to save as gif'''
        # self.anim.save(file_name.replace('.mp4', '') +
        #                '.gif', writer='imagemagick', fps=15)

    def show(self, show_title=False):
        if show_title:
            plt.title('50m '+r'$\times$'+' 50m map', y=-0.1, fontsize=40)
            # plt.title('100m '+r'$\times$'+' 100m map', y=-0.12, fontsize=40)
            plt.tight_layout()
        plt.show()

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    def animate_func(self, i_frame):
        # check collision while plotting
        pos_list = []
        wheels = []
        for agent_name in self.schedule["schedule"]:
            agent = schedule["schedule"][agent_name]
            pos = self.getState(i_frame / framesPerMove, agent)
            pos_list.append(pos)
            p = (pos[0], pos[1])
            # print(pos[0], pos[1], -pos[2] / 3.15159 * 180)'
            cw = carWidth
            lb = LB
            lf = LF

            self.agents[agent_name]._x0 = pos[0] + \
                math.sqrt(cw/2*cw/2+lb*lb) * \
                math.sin(-math.atan2(lb, cw/2)+pos[2])
            self.agents[agent_name]._y0 = pos[1] - \
                math.sqrt(cw/2*cw/2+lb*lb) * \
                math.cos(-math.atan2(lb, cw/2)+pos[2])
            self.agents[agent_name]._x1 = self.agents[agent_name]._x0 + lb+lf
            self.agents[agent_name]._y1 = self.agents[agent_name]._y0 + cw
            self.agents[agent_name].angle = pos[2] / math.pi * 180
            self.agent_names[agent_name].set_position(p)
            if PLOTLINE:
                self.list_xdata[int(agent_name.replace(
                    "agent", ""))].append(pos[0])
                self.list_ydata[int(agent_name.replace(
                    "agent", ""))].append(pos[1])
                self.lines[int(agent_name.replace(
                    "agent", ""))].set_data(self.list_xdata[int(agent_name.replace(
                        "agent", ""))], self.list_ydata[int(agent_name.replace(
                            "agent", ""))])

            # steer = agent[i_frame]['steer']
            # wheel_i = gen_wheels(pos[0], pos[1], pos[2], steer, lb/4, lb/8, lf, lb, cw)
            # wheels += wheel_i
        # reset all colors
        for _, agent in self.agents.items():
            # agent.set_facecolor(None)
            agent.set_edgecolor(agent.original_face_color)
        

        # inter-agent collision detection
        num_agent = len(pos_list)
        for ai in range(num_agent):
            for aj in range(ai+1, num_agent):
                # check collision with i, j:
                collisioned = check_collision(pos_list[ai], pos_list[aj])
                if collisioned:
                    print('inter collision detected: ', ai, aj)
                    if i_frame % framesPerMove == 0 :
                        print('inter collision in timestep', i_frame/framesPerMove)
                    else:
                        print("inter collision between states", i_frame/framesPerMove)
        
        # check collision with obstacles
        for a in range(num_agent):
            for o in self.map["map"]["obstacles"]:
                x, y = o[0], o[1]
                if len(o) == 3:
                    r = o[2]
                elif len(o) == 2:
                    r = obsRadius
                obs = [x , y , r]
                # check collision.
                collisioned = check_obs_collision(pos_list[a], obs)
                if collisioned:
                    print('static collision detected: ', a, obs)
                    if i_frame % framesPerMove == 0 :
                        print('static collision in file', i_frame/framesPerMove)
                    else:
                        print("static collision between states", i_frame/framesPerMove)                    
        
        for wheel in wheels:
            self.wheels.append(wheel)
        return self.patches + self.artists+self.lines +self.wheels

    def update_wheel_info_by_state(self, ):
        pass
    
    def udpate_wheels(self, t , a, x, y, yaw, steer):
        # rear-left, rear-right, front-left, front-right
        
        self.wheels[4*a]._x0 = 0
        self.wheels[4*a]._x1 = 0
        self.wheels[4*a]._y0 = 0
        self.wheels[4*a]._y1 = 0
        self.wheels[4*a].angle  = yaw + steer*180/math.pi
    
    # get the agent state. Interpolation.
    def getState(self, t, d):
        idx = 0
        while idx < len(d) and d[idx]["t"] < t:
            idx += 1
        if idx == 0:
            return np.array([float(d[0]["x"]), float(d[0]["y"]), float(d[0]["yaw"])])
        elif idx < len(d):
            yawLast = float(d[idx-1]["yaw"])
            yawNext = float(d[idx]["yaw"])
            if (yawLast - yawNext) > math.pi:
                yawLast = yawLast - 2 * math.pi
            elif (yawNext - yawLast) > math.pi:
                yawLast = yawLast + 2 * math.pi
            posLast = np.array(
                [float(d[idx-1]["x"]), float(d[idx-1]["y"]), yawLast])
            posNext = np.array(
                [float(d[idx]["x"]), float(d[idx]["y"]), yawNext])
            dt = d[idx]["t"] - d[idx-1]["t"]
            # print("dt", dt)
            # print("t", t)
            t = (t - d[idx-1]["t"]) / dt
            pos = (posNext - posLast) * t + posLast
            return pos
        else:
            return np.array([float(d[-1]["x"]), float(d[-1]["y"]), float(d[-1]["yaw"])])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-s", "--schedule", default=None,
                        help="schedule for agents")
    parser.add_argument("-v", "--video", dest='video', default=None,
                        help="output video file (or leave empty to show on screen)")
    parser.add_argument("--speed", type=int,
                        default=1, help="speedup-factor")
    parser.add_argument("--title", type=bool,
                        default=0, help="show the title")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)
        if map["map"]["obstacles"] is None:
            map["map"]["obstacles"] = np.array([[-1,-1,0.1]])

    with open(args.schedule) as states_file:
        schedule = yaml.load(states_file, Loader=yaml.FullLoader)
    
    num_agents = len(map["agents"])
    if (schedule is None) or (schedule["schedule"] is None):
        schedule = {}
        schedule["schedule"] ={}
    # complete the missing schedule
    for a in range(num_agents):
        name = "agent"+str(a)
        if ( not name in schedule["schedule"].keys()) or\
                schedule["schedule"][name] is None:
            pos = map["agents"][a]["start"]
            print("complete the missing value for agent", a)
            schedule["schedule"][name] =  [{'x': pos[0], 'y':pos[1], 'yaw':pos[2], 't':0, 'steer':0}]

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

    animation = Animation(map, schedule)

    if args.video:
        matplotlib.use("Agg")
        animation.save(args.video, args.speed)
    else:
        matplotlib.use("Qt5Agg")
        animation.show(args.title)
