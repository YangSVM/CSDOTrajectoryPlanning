# CSDO-Trajectory-Planning
## Overview
A fast suboptimal solver for multi vehicle trajectory planning.

CSDO is a hierarchical planner using Centralized Searching and Decentralized Optimization. The efficient MAPF solver PBS (priority based search) is used to search a coarse initial guesses representing a specific homotopy class. An efficient and decentralized quadratic programming is used to get a feasible refine solution.


## Source Code
### Requirement
```bash
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
python3-matplotlib libompl-dev libeigen3-dev
```
> Note: Please make sure your `matplotlib` version is above `2.0`, otherwise it may show weird image while visualization. You can upgrade it by `pip3 install -U matplotlib`.


### Build
```bash
mkdir build 
cd build
cmake -DCMAKE_BUILD_TYPE=Release  ..
make -j8
```



### Run example instances
```bash
# make sure your are in build folder
# run one instance
./csdo -i ../benchmark/map100by100/agents50/obstacle/map_100by100_obst50_agents50_ex13.yaml -o output.yaml 
# make sure you are in scripts folder
# test through the benchmark. 
./test_through_benchmark
```

### Visualize Results
```bash
# make sure your are in build folder
python3 ../scripts/visualize.py -m  ../benchmark/map100by100/agents50/obstacle/map_100by100_obst50_agents50_ex13.yaml  -s output.yaml
```

### Agent and Solver Configuration
The agent and solver configurations, including the size, the kinematic constraints, and penalty functions can be changed in `src/config.yaml`.

## Benchmark

Benchmark for evaluating CL-MAPF problem are available in `benchmark` folder. It contains 3000 unique instances with different map size and agents number.

The folder are arranged like follows, each mapset contains 60 instances:

```
benchmark
├── map100by100
│   ├── agents10
│   │   ├── empty
│   │   └── obstacle
│   ...
└── map50by50
    ├── agents10
    │   ├── empty
    │   └── obstacle
    ...
```

The instance are in `yaml` format.

A typical result from benchmark acts like below:

<img src="img/dataset.gif" width="60%" height="60%">

## Citation 
This work is submitted to RA-L.


## Related Resources
We acknowledge all the open-source contributors for the following projects to make this work possible:

- [CL-CBS](https://opensource.org/licenses/MIT).
- [Priority based Search](https://github.com/Jiaoyang-Li/PBS). Check its license for commercial usage.
- [Fast ASCO](https://github.com/libai1943/MVTP_benchmark)
- [Motion Planning](https://github.com/zhm-real/MotionPlanning)
