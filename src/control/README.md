# Control

Control scripts for path following using pure pursuit, mpc or manual control.

### Prerequisites
* [CasAdi](https://web.casadi.org/)
can be installed with:
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
python ros_mpc.py
```

- To control the car manually (with keyboard arrow keys)
```bash
python ros_manual.py
```

For further details check the code

## Authors
* **Gatti Francesco** - [ceccocats](https://github.com/ceccocats)
* **Ayoub Raji** - [ayoubraji](https://github.com/ayoubraji)
* **Gavioli Federico** - [fgavioli](https://github.com/fgavioli)
