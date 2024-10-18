'''
分析求解结果相关性能
'''
import yaml
from tqdm import tqdm
import numpy as np
from matplotlib import pyplot as plt
import os
from typing import Dict

SOLVER_THRESHOLD = 2


class Status:
    # public part. CL-CBS and CSDO.
    cost = []
    makespan = []
    flowtime = []
    runtime = []

    # specific part of CSDO.
    search_success = []  # include minor collision.
    success = []    # generate a feasible solution.    
    runtime_dsqp = [] # runtime of DSQP.
    runtime_search = [] # runtime of Centralize Searching.

    def clear(self):
        self.cost.clear()
        self.makespan.clear()
        self.flowtime.clear()
        self.runtime.clear()

        self.search_success.clear()
        self.success.clear()
        self.runtime_dsqp.clear()
        self.runtime_search.clear()

def status_append_fail_value(status:Status, time_limit=20):
    status.cost.append(-1)
    status.makespan.append(-1)
    status.flowtime.append(-1)
    status.runtime.append(-1)

    status.search_success.append(0)
    status.success.append(0)
    status.runtime_dsqp.append(time_limit)
    status.runtime_search.append(time_limit)

def process_data(data:Dict, status:Status):
    for kw in data:
        if kw == "search_status":
            search_status = int ( data[kw])
            search_success = 0
            if search_status > 0 :
                search_success = 1
            status.search_success.append(search_success)
        elif kw == "solver_status":
            solver_status = int( data[kw] )
            success = True
            if abs(solver_status)  > SOLVER_THRESHOLD :
                success = False
            status.success.append(success)
        else:
            try: # for most keyword, just append in the list in status.
                attr_list = getattr(status, kw)
                attr_list.append( data[kw] )
            except:
                continue
    
    if "solver_status" not in data:  # for CL-CBS. True if it has file.
        status.success.append(True)
    
    if "runtime_preprocess" in data and \
        "runtime_decentralized_optimization" in data:
        runtime_dsqp = data["runtime_preprocess"] + \
            data["runtime_decentralized_optimization"]
        status.runtime_dsqp.append(runtime_dsqp)

    

def process_solution_file(fname, status:Status, with_solver_status=True):
    keywords = ["cost", "makespan", "flowtime", "runtime", "runtime_search",
                "runtime_preprocess", "runtime_optimization", 
                "runtime_decentralized_optimization", "search_status", "solver_status"]

    end_word = "schedule"
    data = {} # dump all the data in this dictionary.
    with open(fname, "r") as f:
        kw = ""
        f.readline() # skip first line. "statistic"
        line_num = 1 # avoid deadlock
        while kw != end_word and line_num < 100:
            l = f.readline() 
            line_num += 1
            kws = l.split() 
            kw = kws[0][:-1] # remove the ":"
            if kw not in keywords :
                if kw != end_word:
                    print("warning. no such attribute.", kw)
                continue
            data[kw] = float(kws[1]) # dump all the data in this dictionary.
            
    process_data(data, status)


# read from the solution files and save to "status".
def generate_algo_status(file_folder_path, fname_prefix="", 
                         is_CSDO=False, num_scene=60, screen=0):
    # 分析求解结果的成功率
    status = Status()
    status.clear()
    num_success = 0
    
    for i_file in tqdm(range(0, num_scene)):

        try:
            fname = os.path.join(file_folder_path, fname_prefix + str(i_file) + ".yaml") 
            if not os.path.exists(fname) or os.path.getsize(fname) == 0:
                # algorithm cannot solve the MVTP instance.
                if screen > 1:
                    print("file", fname, "doesnot exist.")
                status_append_fail_value(status)
                continue
            # read_solution_status(fname, status, is_CSDO)
            process_solution_file(fname, status, is_CSDO)

        except :
            print("error!")
            print("read file", fname, "failed. Check file format.")
            status_append_fail_value(status)


    print('number of success solutions:', sum(status.success))    
    
    return status


def get_result_folder_fname_prefix(result_path, obstruced, map_size, num_agent):

    if obstruced:
        folder = os.path.join(result_path, "map"+str(map_size)+"by"+str(map_size), 'agents' + str(num_agent), 'obstacle')
        if map_size == 100:
            temp_str =  "map_100by100_obst50_agents"
        elif map_size == 300:
            temp_str =  "map_300by300_obst100_agents"
        elif map_size == 50:
            temp_str = "map_50by50_obst25_agents"
    else:
        folder = os.path.join(result_path, "map"+str(map_size)+"by"+str(map_size), 'agents' + str(num_agent), 'empty')
        if map_size == 100:
            temp_str =  "map_100by100_obst0_agents"
        elif map_size == 300:
            temp_str =  "map_300by300_obst0_agents"
        elif map_size == 50:
            temp_str = "map_50by50_obst0_agents"

    fname_prefix = temp_str + str(num_agent) + "_ex"
    return folder, fname_prefix

def dump_summary(result_path, is_csdo, map_size = 50, obstruced = False):
    result_folder_path = os.path.dirname(os.path.abspath(__file__))+"/../results/"

    
    result_path = result_folder_path + result_path
    if map_size == 50:
        num_agent_list = [5,10,15,20,25]

    elif map_size == 100:
        num_agent_list = [25, 30, 35,40,50,60,70,80,90] # overall test

    runtimes_benchmark = []
    success_rates = []
    makespans_list =[]

    search_rates = []
    runtime_search =[]
    runtime_dqp = []

    num_scene = 60
        
    # 2 output files per map size: obstacle or empty. 
    if obstruced:
        obstruced_str = "obstacle_map"+str(map_size)
    else:
        obstruced_str = "empty_map"+str(map_size)
    fname_stat = os.path.join(result_path, obstruced_str + ".csv") 
    
    for num_agent in num_agent_list:

        folder, fname_prefix = get_result_folder_fname_prefix(result_path, obstruced, map_size, num_agent)
        # print("folder", folder)
        # print("fname_prefix", fname_prefix)

        status = generate_algo_status(folder, fname_prefix, is_CSDO=is_csdo)
        
        success_flags = np.array( status.success , dtype=np.bool)

        # sum up the runtime only the solution is successful.
        runtimes = np.array( status.runtime)
        runtimes_success = runtimes[success_flags]
        success_rate = success_flags.sum()/num_scene
        success_rates.append(success_rate)
        makespans = np.array(status.makespan)
        makespans_list.append(makespans[success_flags])
        # TODO calc the cost of qp solver methods.
        # assert( np.all(costs_list[-1]>0)) 

        runtimes_benchmark.append(runtimes_success)

        if is_csdo:
            search_rates.append( sum(status.search_success)/num_scene )
            rt_search_np = np.array(status.runtime_search)
            rt_dqp_np = np.array(status.runtime_dsqp)
            runtime_search.append(rt_search_np[success_flags].mean())
            runtime_dqp.append(rt_dqp_np[success_flags].mean())
        
    plt.boxplot(runtimes_benchmark)
    # plt.show()
    print('success rates:', [round(x, 2) for  x in success_rates])
    print("average runtime:", [x.mean() for x in runtimes_benchmark])
    print("average costs:", [x.mean() for x in makespans_list])
    if is_csdo:
        print('search success rate', [round(x, 2) for x in search_rates])
        print('runtime search', [round(x, 2) for x in runtime_search])
        print('runtime dqp', [round(x, 2) for x in runtime_dqp])
        
    
    print('done')

    # dump files for plotting.
    with open(fname_stat, "w") as fstat:
        if not is_csdo:
            fstat.write("num_agents, success_rate, calc_time_mean, makespan_mean\n")
            for i in range( len(num_agent_list) ):
                try:
                    line = "%d, %.4f, %.2f, %.2f\n"%(
                        num_agent_list[i], success_rates[i], 
                        runtimes_benchmark[i].mean(), makespans_list[i].mean()               
                        )
                    fstat.write(line)
                except:
                    break
        else: # is CSDO
            fstat.write("num_agents, success_rate, calc_time_mean, makespan_mean, search_success_rate, runtime_of_search, runtime_of_DSQP\n")
            for i in range( len(num_agent_list) ):
                try:
                    line = "%d, %.4f, %.2f, %.2f, %.4f, %.2f, %.2f\n"%(
                        num_agent_list[i], success_rates[i], 
                        runtimes_benchmark[i].mean(), makespans_list[i].mean(),
                        search_rates[i], runtime_search[i], runtime_dqp[i]
                        )
                    fstat.write(line)
                except:
                    break
        print("summary file generated")



if __name__=='__main__':

    # csdo
    # result_path = "csdo"
    # is_csdo = True
    
    # wpbs
    result_path = "wpbs"
    is_csdo = False

    # random priority planner. implemented by CLCBS with batch size 1.
    # result_path = "batch_size_1/"
    # is_csdo = False

    # CL-CBS in paper
    # result_path = "kb2/"
    # is_csdo = False

    map_sizes = [50]
    obstalces = [True, False]


    for map_size in map_sizes:
        for obs in obstalces:
            dump_summary(result_path, is_csdo, map_size = map_size, obstruced=obs)

