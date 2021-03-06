# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

---

## Project Rubric

### Model

I used a bicycle kinematic model for the vehicle to predict its behavior. The state of the system includes the vehicle's position *x, y*, heading *psi*, speed *v*, cross track error *cte*, and heading error *epsi*. The actuator is defined by the steering angle *delta* and acceleration *a*. *Lf* is defined as the distance between the center of mass of the vehicle and it's front axle.

```
x_[t+1] = x[t] + v[t]*cos(psi[t])*dt
y_[t+1] = y[t] + v[t]*sin(psi[t])*dt
psi_[t+1] = psi[t] + v[t]/Lf*delta[t]*dt
v_[t+1] = v[t] + a[t]*dt
cte[t+1] = f(x[t]) - y[t] + v[t]*sin(epsi[t])* dt
epsi[t+1] = psi[t] - psides[t] + v[t]*delta[t]/Lf*dt
```
![model](./img/model.png)

### Timestep Length (*N*) and Elapsed Duration (*dt*)

MPC approximates a continuous reference trajectory using discrete paths. A larger values of *dt* results in a less frequent actuation, which makes it harder to a continuous trajectory but less computation. And a smaller value of *dt* results in a more frequent actuation, which more accurately approximates a continuous reference trajectory but more computation. 

The prediction horizon *T* is the duration over which future predictions are made. *T* is the product of two variables, *N* and *dt*. A good approach to setting *N*, *dt*, and *T* is to first determine a reasonable range for *T* and then tune *dt* and *N* appropriately, keeping the effect of each in mind. I started with 1.0 sec for *T*. The initial combination of *N* was chosen as 10 then *dt* is automatically defined as 0.01. It worked OK until the track meets continuous sharp turns. I increased the timestep to 20, then again *dt* is automatically selected as 0.05. This time the driving is a little bit smoother and manages to drive the whole track. See the [demo](https://youtu.be/UjYNqLwhUvc).

### Polynomial Fitting and MPC Preprocessing

First of all, the equation for f0 and psides0 must be redefined from the Quiz for this project where the 3rd order polynomial is used.

```
// f0 and psides0 from Quiz are for 1st order polynomial
//AD<double> f0 = coeffs[0] + coeffs[1] * x0;
//AD<double> psides0 = CppAD::atan(coeffs[1]);
// We are using 3rd order polynomial here.
AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*x0*x0 + coeffs[3]*x0*x0*x0;
AD<double> psides0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0 + 3*coeffs[3]*x0*x0);
```

The next step is to convert the map coordinates to the car coordinates. The car coordinates are much simpler in terms of consequent calculations. 

```
// Convert map coordinates of (ptsx and ptsy) to car coordinates in Eigen::VectorXd
Eigen::VectorXd car_ptsx(ptsx.size());
Eigen::VectorXd car_ptsy(ptsy.size());
// (px, py) is a car location in the map.
double dx, dy;
for(size_t i = 0; i < ptsx.size(); i++) {
  dx = ptsx[i] - px, dy = ptsy[i] - py;
  car_ptsx(i) =  dx*cos(psi) + dy*sin(psi);
  car_ptsy(i) = -dx*sin(psi) + dy*cos(psi);
}

// ----
// Find coefficients of the polynomial
Eigen::VectorXd coeffs = polyfit(car_ptsx, car_ptsy, 3);
// Calcuate cte and epsi
double cte = polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
```


### Model Predictive Control with Latency

Latency affects actuator dynamics; (e.g) the time elapsed between when a steering command is sent and when that angle is actually achieved. This can be modeled by a simple dynamic system and a vehicle model. One approach would be running a simulation using the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC.

```
latency_x = v*latency; // v*cos(0)*latency;
latency_y = 0;         // v*sin(0)*latency;
latency_psi = -v*delta*latency/Lf;
latency_v = v + a*latency;
latency_cte = cte + v*sin(epsi)*latency;
latency_epsi = epsi - v*delta*latency/Lf;
```

See the [demo](https://youtu.be/SmoW_LOfEv4). Note that the prediction horizon *T* is 0.5 and *N* is 10 in this demo.

---

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
