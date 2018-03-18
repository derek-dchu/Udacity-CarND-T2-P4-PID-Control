# Term 2 Project 4 - PID Control [![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

## Overview

In this project, I use and tune PID controllers to control a autonomous car drive through the track in simulator.

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

### Other Important Dependencies

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


## [Rubric](https://review.udacity.com/#!/rubrics/824/view) Points
### Compiling
#### Your code should compile.
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./pid `

The program executes without errors.

### Implementation
#### The PID procedure follows what was taught in the lessons.

The base algorithm of PID to generate control signal can be found in [src/PID.cpp](./src/PID.cpp).

### Reflection
#### Describe the effect each of the P, I, D components had in your implementation.

To use PID controller to control our autonomous car, we first need to know the effect of the P, I, D component of the PID algorithm.

We use PID to generate steering control signal as following:

> `steering_value = -(Kp * P_error + Ki * I_error + Kd * D_error)`

We define the distance between car and central line of the track is crosstrack error (CTE). 

##### P the Proportional Error
> `P_error = CTE`

The steering value to correct car's driving direction should be proprotional to the CTE. The closer to the central line, the less  negative steering the car has. However, there will be some overshooting, becaue when the car cross the central line, CTE = 0, then steering_value = 0, but it may not in a correct driving angle. Eventually, without other correction, the car will oscillcate along the central line.

Increase Kp will increase overshoot.
Kp = 1:
![P-1](P_1.gif)

Kp = 3:
![P-3](P_3.gif)

##### I the Integral Error
The integral is the running sum of previous errors.

> `I_error = sum(CTE)`

As a accumulated error, this error is help to fine tuned the PID control for even smaller errors.

Note: reset I_error to 0 if the CTE reach 0. Becaue it indicates that the control has corrected previous accumulated error. It should start over.

Increase Ki will increase overshoot.
Kp = 1, Ki = 0.001, Kd = 1:
![PID-001](PID_001.gif)

Kp = 1, Ki = 0.1, Kd = 1:
![PID-01](PID_01.gif)

##### D the Derivative Error
The derivative error is a prediction of the
future error.

> `D_error = (CTE - prev_CTE) / dT`

Increase Kd will decrease overshoot.
Kp = 1, Kd = 1:
![PD-1](PD_1.gif)

Kp = 1, Kd = 3:
![PD-3](PD_3.gif)

#### Describe how the final hyperparameters were chosen.

I use two PIDs to control the car. One is for steering, and the other one is for throttle. Both is tuned using Twiddle. The implementation of twiddle can be found at [src/twiddle.cpp](./src/twiddle.cpp).

### Simulation
#### The vehicle must successfully drive a lap around the track.

Evatually, The steer PID controller is tuned at `0.437229, -9.99989e-05, 0.795459`, and the throttle PID controller is tuned at `0.213868, 0.000957822, 0.138551`. Under these PID controllers, the vehicle successfully drive a lap around the track.