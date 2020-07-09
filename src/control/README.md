# Control

Control scripts for path following using pure pursuit, mpc or manual control.

### Prerequisites
* [CasAdi](https://web.casadi.org/)
maybe can be installed with:
```bash
pip install casadi
```
### Usage

- In order to follow a path using pure pursuit, inside the /pp folder:
```bash
python ros_pp.py
```

- In order to follow a path using simple bicycle kinematic mpc, inside the /mpc folder:
```bash
python ros_mpc_kin.py
```

- or using dynamic bicycle model with pacejka tire model:

```bash
python ros_mpc_dyn.py
```

- Publish the following message to let the car start:
```bash
rostopic pub /commands/stop std_msgs/Bool "data: false" 
```

- To control the car manually (with keyboard arrow keys)
```bash
python ros_manual.py
```

For more details check the code

## Authors
* **Gatti Francesco** - *Developer* - [fgatti](https://git.hipert.unimore.it/fgatti)
* **Ayoub Raji** - *Developer* - [araji](https://git.hipert.unimore.it/araji)
* **Gavioli Federico** - *Developer (manual controller)* - [fgavioli](https://git.hipert.unimore.it/fgavioli)