import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
import os
import itertools
from matplotlib.lines import Line2D
import matplotlib.image as image
from matplotlib.offsetbox import OffsetImage,AnchoredOffsetbox
import seaborn as sns


def place_image(im, loc=3, ax=None, zoom=1, **kw):
    if ax==None: ax=plt.gca()
    imagebox = OffsetImage(im, zoom=zoom*0.72)
    ab = AnchoredOffsetbox(loc=loc, child=imagebox, frameon=False, **kw)
    ax.add_artist(ab)

# 使用truetype字体
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
plt.rcParams["font.family"] = "Times New Roman"  # global setting

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
    

def plot_summary(obs=50, map_size=100, title_bottom=True):
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
    fig, axs = plt.subplots(1,3, figsize=(16,5))

    if title_bottom:
        fig.subplots_adjust(bottom=0.2)
        fig.suptitle(r"Experiment Results of %d m $\times$ %d m Map"%(map_size, map_size), fontsize=LABEL_FONT_SZ, y=0.1)
    else:
        fig.subplots_adjust(top=0.9)
        fig.suptitle(r"Experiment Results of %d m $\times$ %d m Map"%(map_size, map_size), y=0.95, fontsize=LABEL_FONT_SZ)

    # axs[0].axis('off')

    # im = image.imread("results/map_100by100_obst50_agents60_ex1.pdf")
    # place_image(im, loc=2, ax=axs[0], pad=0, zoom=1)

    # plot success rate
    axs[0].plot(random_sp_[:, 0], random_sp_[:, 1], "-^g", label="PP_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[0].plot(clcbs_[:, 0], clcbs_[:, 1], "-o", color="orange", label="seq-CL-CBS_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[0].plot(asco_[:, 0], asco_[:, 1], "-P", color="c", label="ASCO_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[0].plot(random_sp0_[:, 0], random_sp0_[:, 1], "--^g", label="PP_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[0].plot(clcbs0_[:, 0], clcbs0_[:, 1], "--o", color="orange", label="seq-CL-CBS_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[0].plot(asco0_[:, 0], asco0_[:, 1], "--P", color="c", label="ASCO_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[0].plot(csdo_[:, 0], csdo_[:, 1], "b-d", label="CSDO_obs", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[0].plot(csdo0_[:, 0], csdo0_[:, 1], "b--d", label="CSDO_empty", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[0].set_ylabel("Success Rate", fontsize=LABEL_FONT_SZ)
    # plt.sutitle("%dm x %dm random map"%(map_size, map_size))
    # fig.set_title("%dm x %dm random map"%(map_size, map_size))

    # plot makespan
    col_makespan = 3
    axs[1].plot(random_pp[:, 0], random_pp[:, col_makespan], "-^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(clcbs[:, 0], clcbs[:, col_makespan], "-o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(asco[:, 0], asco[:, col_makespan], "-P", color="c", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].plot(random_pp0[:, 0], random_pp0[:, col_makespan], "--^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(clcbs0[:, 0], clcbs0[:, col_makespan], "--o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(asco0[:, 0], asco0[:, col_makespan], "--P", color="c", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].plot(csdo[:, 0], csdo[:, col_makespan]* 2.118, "b-d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[1].plot(csdo0[:, 0], csdo0[:, col_makespan] * 2.118, "b--d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[1].set_ylabel("Makespan", fontsize=LABEL_FONT_SZ)

    # plot log runtime
    col_runtime = 2
    axs[2].plot(clcbs[:, 0], np.log10(clcbs[:, col_runtime]), "-o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(random_pp[:, 0], np.log10(random_pp[:, col_runtime]), "-^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(asco[:, 0], np.log10(asco[:, col_runtime]), "c-P", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].plot(clcbs0[:, 0], np.log10(clcbs0[:, col_runtime]), "--o", color="orange", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(random_pp0[:, 0], np.log10(random_pp0[:, col_runtime]), "--^g", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(asco0[:, 0], np.log10(asco0[:, col_runtime]), "c--P", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].plot(csdo[:, 0], np.log10(csdo[:, col_runtime]), "b-d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)
    axs[2].plot(csdo0[:, 0], np.log10(csdo0[:, col_runtime]), "b--d", linewidth=LINE_WIDTH, markersize=MARKER_SZ)

    axs[2].set_ylabel("log Runtime", fontsize=LABEL_FONT_SZ)

    axs[0].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)
    axs[1].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)
    axs[2].set_xlabel("Agents", fontsize=LABEL_FONT_SZ)


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


    if title_bottom:
        fig.tight_layout(rect=(0,0.1,1,1))
    else:
        fig.tight_layout()



    # plt.title("Success rate and Makespan with number of agents")
    plt.savefig("results/map%dx%d.png"%(map_size, map_size))
    plt.savefig("results/map%dx%d.pdf"%(map_size, map_size))
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

    for map_size in map_sizes:
        if map_size == 50:
            obs = 25
        elif map_size == 100:
            obs = 50
        plot_summary(obs, map_size, title_bottom=True)
        pass