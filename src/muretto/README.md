# Muretto node

Behavioral planning node 

### Parameters
You can adjust the parameters in the `conf/settings.yaml` file.

```
max_range - Maximum LiDAR range index to consider for the left/right mean space computation 
min_range - Minimum LiDAR range index to consider for the left/right mean space computation
race_type - 1=qualifying time attack or obstacle avoidance (autodetect) 2=head to head race
safe_dist_overtake - Safe distance to restart using global path after overtaking
overtaking_range - Maximum distance to the other car at which the overtake can be started
safe_path_dist - Maximum distance to keep if the agent is behind waiting for a chance to overtake
no_overtake_speed - lower bound on the global trajectory speed where it is considered too low to conduct a safe overtake (corners)
path_point_horizon - pointwise look-ahead for the overtake safety check 
```

### Strategy message

The node publishes a message of the type `Strategy.msg` at a regular interval.

```
Header header
int8 race_type - The race type, where 0 = qualifying submission time attack, 1 = obstacle avoidance test, 2 = head to head race
int8 overtake_strategy - The overtake strategy, where -1 = drive at maximum speed following global path, 0 = adjust speed and follow the opponent, 1 = overtake on the left, 2 = overtake on the right
bool ahead - True if the opponent is behind and false if the opponent is ahead of the agent
bool follow_local_path - Boolean to instruct to follow local/global path
float64 right_distance - Mean space at the left of the car
float64 left_distance - Mean space at the left of the car
int16 ego_path_index - Closest index of the global path points to the agent
int8 speed_limit - Maximum speed to follow for the control algorithm
```

### Usage

- To launch the muretto node, use:

```bash
roslaunch muretto muretto.launch
```

For further details check the code

## Authors
* **Ayoub Raji** - [ayoubraji](https://github.com/ayoubraji)
* **Gavioli Federico** - [fgavioli](https://github.com/fgavioli)
