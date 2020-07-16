# frenet_path_planner
C++ implementation of [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame](https://www.researchgate.net/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame)

Actually, it seems to work good in f1tenth but the code is a little bit messy... Still in development 

### Dependencies
[Eigen/Dense](https://eigen.tuxfamily.org/dox/GettingStarted.html)
[matplotlib-cpp](https://github.com/lava/matplotlib-cpp)

### Install dependencies
```
sudo apt-get install build-essential cmake libeigen3-dev python-matplotlib python-numpy python2.7-dev

```


### Usage

```
mkdir build
cd build
cmake ..
make

./FRENET_TEST
```

### TO-DO

- [ ] Manage and clean the Frenet class
- [ ] Load parameters from yaml
- [ ] Correct the matplot demo
- [ ] Create a documentation

### Developers
* **Ion Grigoras** - *Main developer for the f1tenth use case*
* **Ayoub Raji** - *Corrections and refactoring* - [ayoubraji](https://github.com/ayoubraji)

### Credits
The implementation is mostly taken from [frenet_planner_agv](https://github.com/arvindjha114/frenet_planner_agv) inspired by [frenet_optimal_trajectory](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory).
