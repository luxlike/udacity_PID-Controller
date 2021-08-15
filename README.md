# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


[//]: # "Image References"

![](./pid_driving.mp4)]  "PID Driving"
[image1]: ./best_driving.png "Best Driving"


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Project Instructions and Rubric

### The code compiles correctly

```
root@765c90dd2af5:/home/workspace/CarND-PID-Control-Project/build# cmake .. && make
-- Configuring done
-- Generating done
-- Build files have been written to: /home/workspace/CarND-PID-Control-Project/build
Scanning dependencies of target pid
[ 25%] Building CXX object CMakeFiles/pid.dir/src/Twiddle.cpp.o
[ 50%] Building CXX object CMakeFiles/pid.dir/src/main.cpp.o
[ 75%] Linking CXX executable pid
[100%] Built target pid
```



### Reflection

- Describe the effect each of the P, I, D components had in your implementation.

#### Proportional (P):

This parameter controls the error proportionally. When I increasing the proportional gain, the car oscillate very much and out of load. Then decrease the P gain, it's oscillation was reduced. 

#### Integral (I):

This parameter controls the accumulating error. Addition of this term reduces the steady state error. If there is a bias in the system, the integrator builds and builds, thereby increasing the control signal and driving the error down.

#### Derivative (D):

This parameter controls the rate of change of error. When I increasing the proportional gain, the car oscillate was reduced. 

![alt text][movie1]

- Describe how the final hyperparameters were chosen.

The final PID parameters are
| PID      | `Kp` | `Ki`  | `Kd` |
| -------- | ---- | ----- | ---- |
| Steering | 0.13 | 0.002 | 5.5  |
| Speed    | 0.1  | 0.0   | 2.0  |

![alt text][image1]

I firstly initialized PID parameters Kp, Ki and Kd with `[0.15, 0.001, 3.0]` and manualy tunning the parameters.
Decrease Kp, increase Kd then I got a best value.

The speed parameters are tuned by target speed algorithm.
First, I got target speed and got error speed by difference between speed and target speed.

Twiddle tuning was used, but the performance was not satisfactory.

