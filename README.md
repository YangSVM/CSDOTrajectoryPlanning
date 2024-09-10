# CSDO-Trajectory-Planning
## Overview
A fast suboptimal solver for multi vehicle trajectory planning.

CSDO is a hierarchical planner using Centralized Searching and Decentralized Optimization. The efficient MAPF solver PBS (priority based search) is used to search a coarse initial guesses representing a specific homotopy class. An efficient and decentralized quadratic programming is used to get a feasible refine solution.


## Source Code
### Requirement
```bash
sudo apt-get install g++ cmake libboost-program-options-dev libyaml-cpp-dev \
python3-matplotlib libompl-dev
```
> Note: Please make sure your `matplotlib` version is above `2.0`, otherwise it may show weird image while visualization. You can upgrade it by `pip3 install -U matplotlib`.

- Make sure install the osqp **version 0.63**, **not** 1.0.0. You can build from source or used the compiled files. [Download link](https://github.com/osqp/osqp/releases). [Instruction link 1](https://osqp.org/docs/get_started/C.html), [Instruction link 2](https://osqp.org/docs/get_started/sources.html#build-from-sources). **Make sure** you do the `sudo make install` at the end.
- Make sure install the eigen **version 3.4.0**. You can download it from the [offical website](https://eigen.tuxfamily.org/index.php?title=Main_Page) or [gitlab](https://gitlab.com/libeigen/eigen/-/releases) . Then build it from source. Follow the instruction in the downloaded `INSTALL` file. Specifically, you should do the followings.
  ```bash
  mkdir build
  cd build
  cmake ..
  sudo cmake install
  ```

> Other module version references: ompl version 1.5.2; YamlCpp 0.5.2;

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
./csdo -i ../benchmark/map50by50/agents25/obstacle/map_50by50_obst25_agents25_ex0.yaml -o output.yaml
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
This work was accepted by RA-L.
```
@article{yang2024csdo,
  title={CSDO: Enhancing Efficiency and Success in Large-Scale Multi-Vehicle Trajectory Planning},
  author={Yang, Yibin and Xu, Shaobing and Yan, Xintao and Jiang, Junkai and Wang, Jianqiang and Huang, Heye},
  journal={arXiv preprint arXiv:2405.20858},
  year={2024}
}
```


## Related Resources
We acknowledge all the open-source contributors for the following projects to make this work possible:

- [CL-CBS](https://opensource.org/licenses/MIT).
- [Priority based Search](https://github.com/Jiaoyang-Li/PBS). Check its license for commercial usage.
- [Fast ASCO](https://github.com/libai1943/MVTP_benchmark)
- [Motion Planning](https://github.com/zhm-real/MotionPlanning)
