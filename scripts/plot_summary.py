import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
import itertools
from matplotlib.lines import Line2D
import matplotlib.image as image
from matplotlib.offsetbox import OffsetImage,AnchoredOffsetbox
import seaborn as sns
import yaml
from matplotlib.patches import Circle, Rectangle
import math

# TODO 这个用来画 地图 以及 结果图 的 拼接图

import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.rcParams["font.family"] = "Times New Roman"  # global setting

carWidth = 2.0
LF = 2.0
LB = 1.0
obsRadius = 1
framesPerMove = 1

def get_cmap(n, name='hsv'):
    '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
    RGB color; the keyword argument name must be a standard mpl colormap name.'''
    return plt.cm.get_cmap(name, n)

# plot the map configuration
def plot_map(fmap, ax, show_title=True, debug=False):
    fmaps = [os.path.dirname(os.path.abspath(__file__)), fmap]
    fmap = os.path.join(*fmaps)

    with open(fmap) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)
        if map["map"]["obstacles"] is None:
            map["map"]["obstacles"] = np.array([[-1,-1,0.1]])


    f_config_names = [os.path.dirname(os.path.abspath(__file__)), "..", "config.yaml"]
    f_config_name = os.path.join(*f_config_names)
    try:
        with open(f_config_name) as config_file:
            carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
            # global carWidth, LF, LB, obsRadius, framesPerMove
            carWidth = carConfig["carWidth"]

            LF = float(carConfig["LF"])
            LB = carConfig["LB"]
            obsRadius = carConfig["obsRadius"]
            print("LF", LF, "LB", LB, "car width", carWidth)
    except IOError:
        # do things with your exception
        print("ERROR loading config file", f_config_name, ". using default param to plot")
        
    map = map
    # schedule = schedule

    aspect = map["map"]["dimensions"][0] / map["map"]["dimensions"][1]

    # fig = plt.figure(frameon=False, figsize=(8 * aspect, 8))
    # if debug:
    #     ax = fig.add_subplot(111, aspect='equal')

    # fig.subplots_adjust(
    #     left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)
    # ax.set_frame_on(False)

    patches = []
    artists = []
    lines = []
    list_xdata = [[] for _ in range(0, len(map["agents"]))]
    list_ydata = [[] for _ in range(0, len(map["agents"]))]
    agents = dict()
    agent_names = dict()
    # create boundary patch
    xmin = -1
    ymin = -1
    xmax = map["map"]["dimensions"][0] + 1
    ymax = map["map"]["dimensions"][1] + 1

    # ax.relim()
    dx = 0
    ax.set_xlim(xmin-dx, xmax+dx)
    ax.set_ylim(ymin-dx, ymax+dx)
    # plt.xlim(10, 20)
    # plt.ylim(10, 20)
    # ax.set_xticks([])
    # ax.set_yticks([])
    # plt.axis('off')
    # ax.axis('tight')
    # ax.axis('off')

    patches.append(Rectangle(
        (xmin, ymin), xmax - xmin, ymax - ymin, facecolor='none', edgecolor='black', lw=2))
    obs_index = 0
    for o in map["map"]["obstacles"]:
        x, y = o[0], o[1]
        if len(o) == 3:
            r = o[2]
        elif len(o) == 2:
            r = obsRadius
        patches.append(
            Circle((x, y), r, facecolor='grey', edgecolor='grey'))
        ax.text(x, y, str(obs_index), fontsize=6)
        obs_index += 1
        # Rectangle((x - 0.5, y - 0.5), 1, 1, facecolor='red', edgecolor='red'))

    # create agents:
    T = 0
    cmap = get_cmap(len(map["agents"])+1)
    # draw goals first
    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
        cw = carWidth
        lb = LB
        lf = LF
        patches.append(Rectangle(
            (d["goal"][0] + math.sqrt(cw/2*cw/2+lb*lb) * math.sin(-math.atan2(lb, cw/2)+d["goal"][2]),
                d["goal"][1] - math.sqrt(cw/2*cw/2+lb*lb) * math.cos(-math.atan2(lb, cw/2)+d["goal"][2])),
            lb+lf, cw, d["goal"][2] / math.pi * 180,
            facecolor='none', edgecolor=cmap(i+1),  alpha=0.7, lw=1, linestyle=":"))
        
        patches.append(Rectangle(
            (d["start"][0] + math.sqrt(cw/2*cw/2+lb*lb) * math.sin(-math.atan2(lb, cw/2)+d["start"][2]),
                d["start"][1] - math.sqrt(cw/2*cw/2+lb*lb) * math.cos(-math.atan2(lb, cw/2)+d["start"][2])),
            lb+lf, cw, d["start"][2] / math.pi * 180,
            facecolor=cmap(i+1), edgecolor="black",  alpha=0.7, lw=1))
        list_xdata[i].append(d["start"][0])
        list_ydata[i].append(d["start"][1])
        line, = ax.plot(
            list_xdata[i], list_ydata[i], color=cmap(i+1),  alpha=0.3, lw=1.5, linestyle="-.")
        lines.append(line)

    for d, i in zip(map["agents"], range(0, len(map["agents"]))):
        name = d["name"]
        # agents[name] = Rectangle(
        #     (d["start"][0], d["start"][1]), 1, 2, 0.0, edgecolor=None, facecolor=None, alpha=0.7)
        # agents[name].original_face_color = cmap(i+1)
        # patches.append(agents[name])
        # if schedule["schedule"][name] is None:
        #     continue
        # T = max(T, schedule["schedule"][name][-1]["t"])
        agent_names[name] = ax.text(
            d["start"][0], d["start"][1], name.replace('agent', ''), fontsize=6)
        agent_names[name].set_horizontalalignment('center')
        agent_names[name].set_verticalalignment('center')
        artists.append(agent_names[name])
    if debug:
        for patch in patches:
            ax.add_patch(patch)
        for a in artists:
            ax.add_artist(a)

        if show_title:
            plt.title('50m '+r'$\times$'+' 50m map', y=-0.1, fontsize=40)
            # plt.title('100m '+r'$\times$'+' 100m map', y=-0.12, fontsize=40)
            plt.tight_layout()
        plt.show()

    return patches, artists



def place_image(im, loc=3, ax=None, zoom=1, **kw):
    if ax==None: ax=plt.gca()
    imagebox = OffsetImage(im, zoom=zoom*0.72)
    ab = AnchoredOffsetbox(loc=loc, child=imagebox, frameon=False, **kw)
    ax.add_artist(ab)


num_agents_list = [25,30,35,40,50,60,70,80,90,100]
def flip(items, ncol):
    return itertools.chain(*[items[i::ncol] for i in range(ncol)])



def load_csv(fname):
    df = pd.read_csv(fname)
    pp = df.to_numpy()
    pp = pp[:, :4]  # num_agents, success_rate, calc_time_mean, makespan_mean
    return pp

def fname_statistic(method, fname):
    result_folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results")
    return os.path.join(result_folder_path, method, fname)



def fit2given_len(data, t_given, num_agent_np):

    data_ = np.zeros([t_given, 2])
    if t_given >= data.shape[0]:
        data_[:data.shape[0], 1] = data[:, 1]
    else:
        data_[:, 1] = data[:t_given, 1]
    data_[:, 0] = num_agent_np[:t_given]
    return data_
    

def plot_summary(fmap, obs=50, map_size=100, title_bottom=True):
    LABEL_FONT_SZ = 20
    LINE_WIDTH = 2
    MARKER_SZ = 10
    csdo = load_csv('results/csdo/'+'obstacle_map'+str(map_size)+'.csv')
    clcbs = load_csv('results/kb2/'+'obstacle_map'+str(map_size)+'.csv')
    random_pp = load_csv('results/batch_size_1/'+'obstacle_map'+str(map_size)+'.csv')
    asco = load_csv('results/asco/'+'obstacle_map'+str(map_size)+'.csv')
    csdo0 = load_csv('results/csdo/'+ 'empty_map'+str(map_size)+'.csv')
    clcbs0 = load_csv('results/kb2/'+ 'empty_map'+str(map_size)+'.csv')
    random_pp0 = load_csv('results/batch_size_1/' + 'empty_map'+str(map_size)+'.csv')
    asco0 = load_csv('results/asco/'+'empty_map'+str(map_size)+'.csv')

    tmax = max([csdo.shape[0], clcbs.shape[0], random_pp.shape[0], asco.shape[0], 
                csdo0.shape[0], clcbs0.shape[0], random_pp0.shape[0], asco0.shape[0]])
    num_agent_np = None
    for data in [csdo, clcbs, random_pp, asco, csdo0, clcbs0, random_pp0, asco0]:
        if data.shape[0] == tmax:
            num_agent_np = data[:, 0]
    # tmax = 6 # 70
    csdo_ = fit2given_len(csdo, tmax, num_agent_np)
    clcbs_ = fit2given_len(clcbs, tmax, num_agent_np)
    random_sp_ = fit2given_len(random_pp, tmax, num_agent_np)
    csdo0_ = fit2given_len(csdo0, tmax, num_agent_np)
    clcbs0_ = fit2given_len(clcbs0, tmax, num_agent_np)
    random_sp0_ = fit2given_len(random_pp0, tmax, num_agent_np)

    asco_ = fit2given_len(asco, tmax, num_agent_np)
    asco0_ = fit2given_len(asco0, tmax, num_agent_np)

    # fig, axs = plt.subplots(1,3, sharex=True)
    if title_bottom:
        # fig = plt.figure(figsize=(18,4),gridspec_kw={'width_ratios': [3, 1]})
        # figsize 调整图的长宽比例， width ratio 调节 每个子图的大小
        fig, axs = plt.subplots(1, 4, figsize=(20,4), gridspec_kw={'width_ratios': [0.6, 1,1,1]})
    else:
        # fig = plt.figure(figsize=(16,4.5))
        fig, axs = plt.subplots(1, 4, figsize=(18,4), gridspec_kw={'width_ratios': [0.8, 1,1,1]})

    # axs = fig.subplots(1,4)
    # fig, axs = plt.subplots(1,4, figsize=(16,5))

    if title_bottom:
        fig.subplots_adjust(bottom=0.2)
        fig.suptitle(r"Experiment Setting and Results of %d m $\times$ %d m Map"%(map_size, map_size), fontsize=LABEL_FONT_SZ, y=0.1)
    else:
        fig.subplots_adjust(top=0.9)
        fig.suptitle(r"Experiment Setting and Results of %d m $\times$ %d m Map"%(map_size, map_size), y=1, fontsize=LABEL_FONT_SZ)

    patches, artists = plot_map(fmap, axs[0])
    for patch in patches:
        axs[0].add_patch(patch)
    for a in artists:
        axs[0].add_artist(a)

    axs[0].set_xlabel(r"%dm $\times$ %dm Map"%(map_size, map_size), fontsize=LABEL_FONT_SZ)


    # im = image.imread("results/map_100by100_obst50_agents60_ex1.pdf")
    # place_image(im, loc=2, ax=axs[1], pad=0, zoom=1)

    # plot success rate
    axs[1].plot(random_sp_[:, 0], random_sp_[:, 1], "-^g", label="PP_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(clcbs_[:, 0], clcbs_[:, 1], "-o", color="orange", label="seq-CL-CBS_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(asco_[:, 0], asco_[:, 1], "-P", color="c", label="ASCO_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].plot(random_sp0_[:, 0], random_sp0_[:, 1], "--^g", label="PP_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(clcbs0_[:, 0], clcbs0_[:, 1], "--o", color="orange", label="seq-CL-CBS_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(asco0_[:, 0], asco0_[:, 1], "--P", color="c", label="ASCO_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].plot(csdo_[:, 0], csdo_[:, 1], "b-d", label="CSDO_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(csdo0_[:, 0], csdo0_[:, 1], "b--d", label="CSDO_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].set_ylabel("Success Rate", fontsize=LABEL_FONT_SZ)
    # plt.sutitle("%dm x %dm random map"%(map_size, map_size))
    # fig.set_title("%dm x %dm random map"%(map_size, map_size))

    # plot makespan
    col_makespan = 3
    axs[2].plot(random_pp[:, 0], random_pp[:, col_makespan], "-^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(clcbs[:, 0], clcbs[:, col_makespan], "-o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(asco[:, 0], asco[:, col_makespan], "-P", color="c", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].plot(random_pp0[:, 0], random_pp0[:, col_makespan], "--^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(clcbs0[:, 0], clcbs0[:, col_makespan], "--o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(asco0[:, 0], asco0[:, col_makespan], "--P", color="c", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].plot(csdo[:, 0], csdo[:, col_makespan]* 2.118, "b-d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(csdo0[:, 0], csdo0[:, col_makespan] * 2.118, "b--d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].set_ylabel("Makespan", fontsize=LABEL_FONT_SZ)

    # plot log runtime
    col_runtime = 2
    axs[3].plot(clcbs[:, 0], np.log10(clcbs[:, col_runtime]), "-o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[3].plot(random_pp[:, 0], np.log10(random_pp[:, col_runtime]), "-^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[3].plot(asco[:, 0], np.log10(asco[:, col_runtime]), "c-P", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[3].plot(clcbs0[:, 0], np.log10(clcbs0[:, col_runtime]), "--o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[3].plot(random_pp0[:, 0], np.log10(random_pp0[:, col_runtime]), "--^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[3].plot(asco0[:, 0], np.log10(asco0[:, col_runtime]), "c--P", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[3].plot(csdo[:, 0], np.log10(csdo[:, col_runtime]), "b-d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[3].plot(csdo0[:, 0], np.log10(csdo0[:, col_runtime]), "b--d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[3].set_ylabel("log Runtime", fontsize=LABEL_FONT_SZ)

    axs[1].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)
    axs[2].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)
    axs[3].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)


    # own defined lines and labels
    labels = ["CSDO", "FastASCO", "SCC", "PP", "Obstacle", "Empty"]
    lines = [Line2D([0], [0], color = "b", marker= 'd', linewidth=LINE_WIDTH, markersize=MARKER_SZ),
                    Line2D([0], [0], color = "c", marker='P', linewidth=LINE_WIDTH, markersize=MARKER_SZ),
                    Line2D([0], [0], color="orange", marker='o', linewidth=LINE_WIDTH, markersize=MARKER_SZ),
                    Line2D([0], [0], color = "g", marker='^', linewidth=LINE_WIDTH, markersize=MARKER_SZ),
                    Line2D([0], [0], color="b", linewidth=LINE_WIDTH, markersize=MARKER_SZ),
                    Line2D([0], [0], color="b", linestyle='--', linewidth=LINE_WIDTH, markersize=MARKER_SZ)                    
                    ]
    # fig.legend(custom_lines, labels)

    # lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
    # lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]

    # Finally, the legend (that maybe you'll customize differently)
    n_col = 3

    if title_bottom:    
        # loc: legend的方框的对齐点。lower right代表方框的右下角。bbox_to_anchor就是方框在画幅中的位置
        # https://stackoverflow.com/questions/55767312/how-to-position-suptitle
        fig.legend(flip(lines, n_col), flip(labels, n_col), loc='lower right', ncol=n_col, bbox_to_anchor=(0.994, 0.235))
    else:
        fig.legend(flip(lines, n_col), flip(labels, n_col), loc='lower right', ncol=n_col, bbox_to_anchor=(0.994, 0.135))
    if map_size == 50:
        plt.setp(axs, xticks=np.linspace(5,25,5, dtype=int), xticklabels=[str(x) for x in np.linspace(5,25,5, dtype=int)])
    elif map_size == 100:
        plt.setp(axs, xticks=np.linspace(30,90,7, dtype=int), xticklabels=[str(x) for x in np.linspace(30,90,7, dtype=int)])

    axs[0].set_xticks([])
    axs[0].set_yticks([])
    axs[0].set_aspect(1.0)
    for ax in axs:
        ax.xaxis.set_label_coords(0.5, -0.07)

    # 图形铺满 rect 代表的位置，标题和label会被挤开，相当于变相裁剪
    if title_bottom:
        fig.tight_layout(rect=(0,0.1,1,1.12))
    else:
        fig.tight_layout(rect=(0,-0.02,1,1.06))



    # plt.title("Success rate and Makespan with number of agents")
    plt.savefig("results/map%dx%d_title_center_bottom%d.png"%(map_size, map_size, title_bottom))
    plt.savefig("results/map%dx%d_title_center_bottom%d.pdf"%(map_size, map_size, title_bottom))
    plt.show()
    print('hold on')



def plot_dsqp_time(map_size=50):
    if map_size == 50:
        obs_list = [0, 25]
    elif map_size == 100:
        obs_list = [0, 50]
    dqps = []
    for obs in obs_list:
        dqp = load_csv('results/csdo/'+'obstacle_map'+str(map_size)+'.csv')
        dqps.append(dqp)
    


    
    pass




if __name__ == '__main__':
    
    map_sizes = [50, 100]

    fmap = ""
    

    for map_size in map_sizes:
        fmap_num_agent = 0
        if map_size == 50:
            obs = 25
            fmap_num_agent = 25
            fmap = "map_50by50_obst25_agents25_ex1.yaml"
        elif map_size == 100:
            obs = 50
            fmap_num_agent = 60
            fmap = "map_100by100_obst50_agents60_ex1.yaml"
        fmaps = ["..", "benchmark", "map%dby%d"%(map_size, map_size), 
                 "agents%d"%(fmap_num_agent), "obstacle", fmap]
        fmap_with_path = os.path.join(*fmaps)
        # plot_map(fmap_with_path)
        plot_summary(fmap_with_path, obs, map_size, title_bottom=True)
        pass