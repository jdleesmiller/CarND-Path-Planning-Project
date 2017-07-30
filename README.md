# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

# Model Documentation

## Map

The `Map` is responsible for translating from Frenet (s, d) coordinates to Cartesian (x, y) coordinates, based on the provided waypoint data. To smooth the map data, it fits four cubic splines to the waypoint data: one on each of (s, x), (s, y), (s, dx) and (s, dy). It uses these splines to look up the position and direction for a given `s`, and then uses the map's unit normal vector to translate for `d`.

## Trajectory

The `Trajectory` class is responsible for fitting a 1D jerk-minimizing trajectory given boundary conditions on position, speed and acceleration. For efficiency reasons, the jerk minimization is implemented as a functor; its constructor builds the LU decomposition for the weights matrix for a given time horizon, which is reused for multiple sets of boundary conditions. (This also avoids the need to explicitly invert the matrix.)

## Car and Cost Function

The `Car` class combines two Trajectories, one for each of the two Frenet coordinates, `s` and `d`. There is one Car instance for the car being controlled, for which we use jerk-minimizing trajectories for both coordinates.

The `Car` class is also used to model the other cars on the roads. For them we simply use an affine model for `s` by setting the first two terms of the quintic from the sensor fusion package and setting the rest to zero. (That is, the model is `s_0 + v * t` for the observed initial `s` position `s_0` and observed speed `v`). Similarly, the trajectory for `d` uses only the first term --- for planning purposes, it is assumed that the other cars do not change lanes.

Copies of the main `Car` instance are used in the Planner to evaluate possible trajectories. Each `Car` instance computes its cost, which is calculated at each planning time step; the planner adds these costs to obtain the total cost for the car's proposed `s` and `d` trajectories. The cost function comprises:

1. A very heavily weighted penalty for collisions with other vehicles. Collisions are detected in (`s`, `d`) space, which is simple to implement but does require fairly large safety margins to avoid glancing collisions.

1. Heavily weighted penalties for leaving the road surface or violating constraints on jerk, acceleration or speed.

1. A moderately weighted penalty for the absolute difference between the instantaneous and target speeds.

1. Lightly weighted penalties for longitudinal and lateral jerk and acceleration, to encourage straighter trajectories, and lane centering and lane preference to keep the car in lane and incentivize quick lane changes.

Profiling showed that ~75% of time was spent in collision detection when using a naive approach (compare positions at all timesteps), so we can try to improve that.

## Planner

The planner uses a combination of sampling and hill climbing to find good trajectories.

Testing with sampling alone showed occasional erratic behavior, even when the number of samples was relatively large.

Testing with hill climbing alone showed that it was very unlikely to change lanes --- it needs to both change lanes and try a higher speed in order to find out that it's better, so simple coordinate descent does not work.

The combined approach is:

1. Choose a lane. Fix the `d` coordinate goal to the center of the lane and require that the lateral speed and acceleration be zero.

2. Generate a small number of sample `s` trajectories (~10) in that lane. Each sample is based on the state at the end of the currently planned trajectory with Gaussian noise added to each component (`s`, speed, acceleration).

3. For each lane and sample trajectory, do a few iterations of hill climbing (up to 10) on the `s` goal state to try to reduce the cost function.

The sampling makes it likely that the planner will discover that it can go faster in another lane, and the hill climbing lets it refine the trajectory to make its cost competitive with the more refined trajectory for the current lane based on the existing planned trajectory.

# Project Brief

### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

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

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!
