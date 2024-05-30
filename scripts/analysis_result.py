'''
分析求解结果相关性能
'''
import yaml
from tqdm import tqdm
import numpy as np
from matplotlib import pyplot as plt
import os

SOLVER_THRESHOLD = 2

# CSDO最后情况。

class Status:
    # public part. 
    cost = []
    makespan = []
    flowtime = []
    runtime = []

    # specific part of CSDO.
    search_success = []  # include minor collision.
    success = []    # generate a feasible solution.
    runtime_dqp = [] # runtime of DSQP.
    runtime_search = [] # runtime of Centralize Searching.

    def clear(self):
        self.cost.clear()
        self.makespan.clear()
        self.flowtime.clear()
        self.runtime.clear()

        self.search_success.clear()
        self.success.clear()
        self.runtime_dqp.clear()
        self.runtime_search.clear()



def status_append_fail_value(status:Status, time_limit=20):
    status.cost.append(-1)
    status.makespan.append(-1)
    status.flowtime.append(-1)
    status.runtime.append(-1)

    status.search_success.append(0)
    status.success.append(0)
    status.runtime_dqp.append(time_limit)
    status.runtime_search.append(time_limit)
    
# two types of yaml. 
# 直接读取文件前几行的统计信息得出结果。而不使用yaml load
def read_solution_status(fname, status:Status, with_solver_status=True):
    with open(fname, "r") as f:
        f.readline() # "statistic"
        l = f.readline()
        cost = float(l.split()[1])        
        l = f.readline()
        makespan = float(l.split()[1])        
        l = f.readline()
        flowtime = float(l.split()[1])        
        l = f.readline()
        runtime = float(l.split()[1])
        
        # with specific info of CSDO.
        if with_solver_status:
            l = f.readline()
            runtime_search = float(l.split()[1])            
            l = f.readline()
            runtime_preprocess = float(l.split()[1])
            l = f.readline()
            l = f.readline()
            runtime_decentralized_optimization = float(l.split()[1])            
            
            # 0: failed; 1: minor collision; 2: success.
            l = f.readline()
            search_status = int(float(l.split()[1]))
            search_success = 0
            if search_status > 0 :
                search_success = 1
            
            l = f.readline()
            solver_status = int(float(l.split()[1]))
            if abs(solver_status)  > SOLVER_THRESHOLD :
                success = False
            else:
                success = True
            
            status.runtime_search.append(runtime_search)
            status.runtime_dqp.append(runtime_preprocess + runtime_decentralized_optimization) 
            status.search_success.append(search_success)
            status.success.append(success)
        else:
            # CL-CBS. not empty file is successful.
            status.success.append(True)

        status.cost.append( cost )
        status.makespan.append(makespan)
        status.flowtime.append(flowtime)
        status.runtime.append(runtime)
    return 


# 读取并对比成功率和计算时间
def analysis_success_runtime(file_folder_path, fname_prefix="", is_CSDO=False, num_scene=60):
    # 分析求解结果的成功率
    status = Status()
    status.clear()
    num_success = 0
    
    for i_file in tqdm(range(0, num_scene)):

        try:
            fname = os.path.join(file_folder_path, fname_prefix + str(i_file) + ".yaml") 
            if not os.path.exists(fname) or os.path.getsize(fname) == 0:
                # algorithm cannot solve the MVTP instance.
                print("file", fname, "doesnot exist.")
                status_append_fail_value(status)
                continue
            read_solution_status(fname, status, is_CSDO)

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
        # num_agent_list = [25, 30, 35,40,50] # fast test
        num_agent_list = [25, 30, 35,40,50,60,70,80,90] # overall test

    runtimes_agents = []
    success_rates = []
    makespans_list =[]

    search_rates = []
    runtime_search =[]
    runtime_dqp = []

    num_scene = 60
    for num_agent in num_agent_list:

        folder, fname_prefix = get_result_folder_fname_prefix(result_path, obstruced, map_size, num_agent)
        # print("folder", folder)
        # print("fname_prefix", fname_prefix)

        status = analysis_success_runtime(folder, fname_prefix, is_CSDO=is_csdo)
        
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

        runtimes_agents.append(runtimes_success)

        if is_csdo:
            search_rates.append( sum(status.search_success)/num_scene )
            rt_search_np = np.array(status.runtime_search)
            rt_dqp_np = np.array(status.runtime_dqp)
            runtime_search.append(rt_search_np[success_flags].mean())
            runtime_dqp.append(rt_dqp_np[success_flags].mean())
        
    plt.boxplot(runtimes_agents)
    # plt.show()
    print('success rates:', [round(x, 2) for  x in success_rates])
    print("average runtime:", [x.mean() for x in runtimes_agents])
    print("average costs:", [x.mean() for x in makespans_list])
    if is_csdo:
        print('search success rate', [round(x, 2) for x in search_rates])
        print('runtime search', [round(x, 2) for x in runtime_search])
        print('runtime dqp', [round(x, 2) for x in runtime_dqp])
        
    
    print('done')
    

    with open(fname_stat, "w") as fstat:
        if not is_csdo:
            fstat.write("num_agents, success_rate, calc_time_mean, makespan_mean\n")
            for i in range( len(num_agent_list) ):
                try:
                    line = "%d, %.4f, %.2f, %.2f\n"%(
                        num_agent_list[i], success_rates[i], 
                        runtimes_agents[i].mean(), makespans_list[i].mean()               
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
                        runtimes_agents[i].mean(), makespans_list[i].mean(),
                        search_rates[i], runtime_search[i], runtime_dqp[i]
                        )
                    fstat.write(line)
                except:
                    break
        print("summary file generated")


# dump the result for seaborn plotting. 
def dump_for_seaborn(result_path, is_csdo, map_size = 50, obstruced = False):
    result_folder_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "results") 

    
    result_path = os.path.join(result_folder_path, result_path)
    if map_size == 50:
        num_agent_list = [5,10,15,20,25]

    elif map_size == 100:
        # num_agent_list = [25, 30, 35,40,50] # fast test
        num_agent_list = [25, 30, 35,40,50,60,70,80,90] # overall test

    runtimes_agents = []
    success_rates = []
    makespans_list =[]

    search_rates = []
    runtime_search =[]
    runtime_dqp = []

    num_scene = 60
    
    #output: 存 在文件里。文件摆放方式。方法目录下。
    if obstruced:
        obstruced_str = "obstacle_map"+str(map_size)
    else:
        obstruced_str = "empty_map"+str(map_size)
    fname_stat = os.path.join(result_path, obstruced_str + "_seaborn.csv") 
    with open(fname_stat, "w") as fstat:
        fstat.write("num_agents,success,runtime,makespan\n")

    for num_agent in num_agent_list:

        folder, fname_prefix = get_result_folder_fname_prefix(result_path, obstruced, map_size, num_agent)
        # print("folder", folder)
        # print("fname_prefix", fname_prefix)

        status = analysis_success_runtime(folder, fname_prefix, is_CSDO=is_csdo)
        
        success_flags = np.array( status.success , dtype=np.bool)
        # 直接全部写入文件中

        with open(fname_stat, "a") as fstat:
            for i in range( len(status.success) ):
                try:
                    line = "%d, %d, %.2f, %.2f\n"%(
                        num_agent, status.success[i], 
                        status.runtime[i], status.makespan[i]             
                        )
                    fstat.write(line)
                except:
                    break

        # sum up the runtime only the solution is successful.
        runtimes = np.array( status.runtime)
        runtimes_success = runtimes[success_flags]
        success_rate = success_flags.sum()/num_scene
        success_rates.append(success_rate)
        makespans = np.array(status.makespan)
        makespans_list.append(makespans[success_flags])
        # TODO calc the cost of qp solver methods.
        # assert( np.all(costs_list[-1]>0)) 

        runtimes_agents.append(runtimes_success)


        
    # plt.show()
    print('success rates:', [round(x, 2) for  x in success_rates])
    print("average runtime:", [x.mean() for x in runtimes_agents])
    print("average costs:", [x.mean() for x in makespans_list])

        
    
    print('done')
    


if __name__=='__main__':

    # csdo
    result_path = "csdo"
    is_csdo = True

    # random priority planner. implemented by CLCBS with batch size 1.
    # result_path = "batch_size_1/"
    # is_csdo = False

    # CL-CBS in paper
    # result_path = "kb2/"
    # is_csdo = False

    map_sizes = [50, 100]
    obstalces = [True, False]


    for map_size in map_sizes:
        for obs in obstalces:
            # dump_summary(result_path, is_csdo, map_size = map_size, obstruced=obs)
            dump_for_seaborn(result_path, is_csdo, map_size = map_size, obstruced=obs)


    
