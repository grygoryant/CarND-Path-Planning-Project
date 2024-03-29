# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Reflection

The main module in this project is Behavior Planner, which outputs resulting trajectory given localization and sensor fusion data. It uses finite-state machine and cost functions to select the proper trajectory from all generated ones.

### Finite-state machine

For this project I've decided to use only three states: Keep Lane, Lane Change Left and Lane Change Right, and it appeared to be enough for the given task.

<img src="img/fsm.png">

### Cost functions

It was pretty interesting to design cost functions, and I've came up with the following:

- Drivable area cost - used for keeping ego vehicle in drivable part of the road;
- Collision cost - penalizes trajectory for being dangerous for surrounding vehicles during lane change;
- Occupied lane cost - maked car select free lane when it's possible;
- Speed cost - helps car to select the lane with the maximum speed.

Weights for the above four functions have been chosen empirically.

### Trajectory generation

Trajectory is generated in two steps:

- Generating spline using anchor points and untraveled points of the previous path;
- Splitting trajectory into segments to maintain desired speed.

It is necessary to mention that trajectory is generated for ego-related coordinate system, and then translated to global one.

### Resulting video

During the run showed in a [video](https://youtu.be/sy7zSuForg0), the car was able to travel 16+ miles without incedents, performing some nice lane changes and lane choices.

