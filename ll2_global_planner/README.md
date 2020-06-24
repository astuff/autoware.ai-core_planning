# ll2_global_planner

This ROS package provides a global planner that finds routes in Lanelet2 maps.

## Dependencies

- Lanelet2


## ROS API

#### Subs

- `lanelet_map_bin` (autoware_lanelet2_msgs/MapBin)  
This topic is used to load the lanelet2 map.
- `move_base_simple/goal` (geometry_msgs/PoseStamped)  
This topic is used to receive a waypoint from RViz.

#### Pubs

- `lane_waypoints_array` (autoware_msgs/LaneArray)
The waypoints that make up the planned route.

#### Transform Listeners

- `map` -> `base_link` dynamic TF  
Used to determine the current position of the vehicle within the map.

#### Transform Broadcasters


#### Configuration Parameters

See the `config/params.yaml` file for a list of parameters and their descriptions.

## Notes
