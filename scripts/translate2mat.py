from scipy.io import savemat
import yaml
import numpy as np
import os

if __name__ == '__main__':
    fpath_abs = os.path.dirname(os.path.abspath(__file__))

    fpath = fpath_abs + '/../benchmark/map50by50/agents'
    outputpath = fpath_abs + '/../build/map50by50/agents'
    num_agents_list = [5,10,15,20,25]
    f_map_prefix = 'map_50by50_obst'
    n_obs_list = [0, 25]

    # fpath = fpath_abs + '/../benchmark/map100by100/agents'
    # outputpath = fpath_abs + '/../build/map100by100/agents'
    # num_agents_list = [25, 30, 35, 40, 50, 60, 70, 80, 90, 100]
    # f_map_prefix = 'map_100by100_obst'
    # n_obs_list = [0, 50]

    num_file = 60
    r_obs = 0.8
    for na in num_agents_list:
        for n_obs in n_obs_list:
            obstruced_str = ''
            if n_obs == 0:
                obstruced_str = 'empty'
            else:
                obstruced_str = 'obstacle'

            f_prefix = f_map_prefix + '%d_agents%d_ex'%(n_obs, na)
            
            for ifile in range(num_file):
                fname = fpath + str(na)+'/'+ obstruced_str +'/' + f_prefix + str(ifile) + '.yaml'
                with open( fname) as f:

                    scene = yaml.load(f)
                    profiles = np.zeros([na ,6])

                    for ia in range(na):
                        profiles[ia, :3] = scene["agents"][ia]["start"]
                        profiles[ia, 3:] = scene["agents"][ia]["goal"]

                        # 调整方向角
                        profiles[ia, 2] = -profiles[ia, 2] 
                        profiles[ia, -1] = -profiles[ia, -1] 

                    if n_obs == 0:
                        obstacles = np.array([[-1,-1,0.8]])
                    else:
                        obstacles = np.zeros([n_obs, 3])
                        obstacles[:, 2] = 0.8
                        for iobs in range(n_obs):
                            obstacles[iobs, :2] = scene["map"]["obstacles"][iobs]
                    
                    
                    data = {"obstacles": obstacles, "profiles": profiles}

                    foutput_path = outputpath + str(na)+'/'+ obstruced_str +'/' 
                    if not os.path.exists(foutput_path):
                        os.makedirs(foutput_path, exist_ok=True)

                    foutput_name = foutput_path + f_prefix + str(ifile) + ".mat"
                    savemat(foutput_name, data)

        pass
    pass

