# IFAC 2020 F1tenth virtual race - HiPeRT team repository
Grand prix submission

Navigation stack for the f1/10 [ifac2020 virtual race](https://f1tenth.org/ifac2020.html). It has also been used as a starting point for the [iros2020 virtual race](https://f1tenth.org/iros2020.html).

### Known issues
* The obstacle detector node detects as circles also some parts of the track boundaries.
* The Spline function used in the Frenet frame based planner takes an huge amount of time to interpolate the points in the beginning. You can solve it using other more efficient ways to [initialize](https://github.com/HiPeRT/ifac2020_f1tenth_submission/blob/a8db95dd512716d150652a83a14387f49ba56f16/src/frenet_planner_f1tenth/include/frenet_path_planner/src/cubic_spline_planner.cpp#L63) the spline.
* The strategy node, called Muretto, has a too naive way to determine if the opponent is behind or ahead the ego vehicle.

### General prerequisites
* [Ubuntu 16.04.4](http://releases.ubuntu.com/16.04/)
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
Tested also on Ubuntu 18 and ROS Melodic

### Packages prerequisites
To install them you can launch [install_deps.sh](https://github.com/HiPeRT/ifac2020_f1tenth_submission/blob/master/install_deps.sh)
* [RangeLibc](https://github.com/kctess5/range_libc)
* [Armadillo](http://arma.sourceforge.net/)
* [yaml-cpp](https://github.com/jbeder/yaml-cpp)
* [CasAdi](https://web.casadi.org/)


### Usage

`roslaunch launcher hipert.launch`

## Team members
* **Gavioli Federico** - [fgavioli](https://github.com/fgavioli)
* **Ayoub Raji** - [ayoubraji](https://github.com/ayoubraji)
* **Biagio Licari** - [biagio7xD](https://github.com/biagio7xD)
* **Andrea Serafini** - [AndreaSerafini](https://github.com/AndreaSerafini)

## Other developers from our lab whose works have been used
* **Gatti Francesco** - [ceccocats](https://github.com/ceccocats)
* **Ion Grigoras**


