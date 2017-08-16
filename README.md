# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.


---

## Reflections

The implementation of the path planner involves three major tasks. The first step is to identify the target lane at the end of the path. In the case of a slow car in the front, the ego car should consider a lane change when safe and feasible. The second task is to set the target speed at the end of the new path. The ego car should drive as close to the speed limit as possible unless being blocked by other cars, which should then trigger a gradual deceleration to avoid collision. The third task is to generate a smooth trajectory based on the target lane and target speed, making sure the movement has acceptable acceleration and jerk. Now let's look into the details of each step:

####1. Determine Target Lane
The target_lane variable is defined and initialized outside the loop in line 58. To help update its value in each communication cycle, we defined three boolean flags in the loop. "too_close" is raised when a front car is getting too close and lane changing needs to be considered; "left_lane_clear" and "right_lane_clear" signal the availability of the neighboring lanes.

Starting from line 117 we loop through all cars in sensor fusion to identify cars that can potentially interfere with the ego car. We first read the position and velocity of each car and predict its future position at the end of the previous path (to be more precise, it's prev_size * TIME_STEP seconds later). We then compare the value with the future position of the ego car. If the car is in the same lane with the ego car and within 30m in the front, we raise the "too_close" flag. If the car is in a neighboring lane but within (-5m, +32m) in s coordinate relative to the ego car's future position, we set the corresponding "lane_clear" flags to false.
 
 With the three flags we can then determine the target_lane (line 151), whether to shift right or left, or keep in the same lane.
 
 ####2. Set Target Speed
 If the front car is too slow and the neighboring lanes busy, we'll need to reduce the speed gradually until reaching the same speed of the front car (line 160).
 
 If the lane is clear in front of us and the ego car is not running at the full speed, we'll need to gradually accelerate (line165).
 
 ####3. Trajectory Generation
 This part has pretty much followed Aaron Brown's lecture. There are three major steps:
 
 First of all, we prepare a list of (4 in this case) points to fit a spline. The first two points are the last two points from the previous path, which ensures good continuity. The third and forth points are some distance away down the road, in the center of the target_lane. Before fitting the spline we transform all the points to the ego car's local coordinates to reduce computational complexity.
 
 The second step is to determine how we partition the spline. We first look at some horizon distance (hori_x) ahead of us, where the ego car would end up at (hori_x, hori_y) in its local frame. The arc length can be approximated using the Euclidean distance of sqrt(hori_x^2 + hori_y^2)
    