# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

### Welcome to MPC ###
[//]: # (Image References)
[image1]: ./report/loop.png "Loop"
[image2]: ./report/waypoints.png "Waypoints"
[image3]: ./report/architecture.png "Architecture"
[image4]: ./report/result.png "Result"
[image5]: ./report/waypoints2.png "Waypoints 2"


This is the Last Project of Term 2 in SDC Nanodegree !
This Project is about MODEL PREDICTIVE CONTROLLERS.This is more sophisticated than PID because it takes into account trajectory ahead of time, can deal with latency, and can take constraints as entries.

Let's start with a basic picture of what MPC is.

![alt text][image1]

Here are the things we can observe here :
* N is the number of timesteps a horizon of predictions.
* dt is the time elapsed between actuations.
* MPC takes into account the *MODEL* , the *CONSTRAINTS* and the *COST* unlike PID that only had a simple Model and Cost.

Control is a step that happens after Path Planning. The goal of a controller is to define a throttle value and a steering angle value to fit to a reference trajectory and reference velocity. Path Planning is a step that gives this trajectory.
![alt text][image3]

The trajectory defined by the controller has to be as close as possible to the trajectory given by the Path Planner.
![alt text][image5]

The points have to fit the line.

### MAIN Code 

First, let's talk about the __MAIN__ function.
* This function starts by asking the simulator what are the waypoints, the current state (X,Y,PSI,V,CTE,EPSI), but also the steering angle DELTA and the throttle A.
* Then, we do some state space change because we want our initial coordinates to be X=0,Y=0 and PSI=0 since the simulator orientation and position are shifted.
To do so, we do the following translation and rotation of the waypoints.
```
for(int i =0; i<ptsx.size();i++)
          {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x*cos(-psi) - shift_y*sin(-psi);
            ptsy[i] = shift_x*sin(-psi) + shift_y*cos(-psi);
          }
```
* Then, we fit a 3rd order polynomial to the waypoints:
![alt text][image2]
* We calculate our two errors : __Cross-Track Error__ and __Orientation Error__.
* Then, we define a model but to deal with 100ms latency we calculate 100ms ahead of time. This state is now our initial state.
* We send __state__ and __coeffs__ to __MPC::SOLVE__. That function returns the best steering and acceleration depending on our actual state and our trajectory.
* We display reference and predicted lines.
![alt text][image5]

----
### MPC::SOLVE Code

Now, let's talk about the __Solve__ function.
* This function takes in entry the state and coefficients fitted to the polynomial.
* We first define global variables that are going to be __N__,__dt__,__Reference Velocity__,__Reference CTE__, __Reference Epsi__
I set __N = 10__ and __dt = 0.1__ because initially.
I looked at CTE in terminal at the moment of entering the bridge and took the lowest depending on my parameters.
__Trials with dt =0.1:__
 ** N=10, dt=0.1 --> CTE = 6400
 ** N=11, dt=0.1 --> CTE = 6200
 ** N=12, dt=0.1 --> CTE = 5600
 ** N=13, dt=0.1 --> CTE = 6600
N=12 seems to be the better choice.

__Trials with N=12:__
 ** dt=0.10 --> CTE=5600
 ** dt=0.09 --> CTE=4300
 ** dt=0.08 --> CTE=4100
 ** dt=0.07 --> CTE=2600
 ** dt=0.06 --> CTE=2550
 ** dt=0.05 --> CTE=60600
dt=0.06 seems to be the better choice.
However, I noticed after few laps that my car went a bit off the road so I should add more cost later when turning.
I set reference CTE and reference Epsi to 0 because that is the goal of our MPC
I set reference Velocity to 100 miles per hour, this could be different but we need to be at least at 88 mph to see into the future. (Please don't mind the joke).
* We define a __VARS__ (x1...xn,y1...yn,psi1...psin,v1...vn,cte1...cten,epsi1...epsin, delta1...deltan,a1...an) with a size of 6xN + 2x(N-1;
and __CONSTRAINTS__ with a size of 6xN.
* We create fill vars with 0 and set constraints depending on what we want to do.
 ** a should be between -1 and 1 so we set lowerbounds to -1 and upperbounds to 1
 ** delta should be between -25° and 25°
 ** the rest of the values can be between a really low number -10e19 and a really large number 10e19
 * Finally, we create __MPC::FG_EVAL__ and call the function with __coeffs__. This function returns cost and constraints.
 * We pass these values to the IPOPT solver that returns the best delta and acceleration.
 
 ----
 ### MPC::FG_EVAL Code
Finally, let's study the __FG_Eval__ function.
* FG eval contains cost and constraints. The goal of this function is to returns this.
* FG[0] contains our COST. We initially set it to 0.
```
    for (int t = 0; t<N;t++){
      fg[0] += 2000*CppAD::pow(vars[cte_start + t] - ref_cte,2);
      fg[0] += 2000*CppAD::pow(vars[epsi_start +t] - ref_epsi,2);
      fg[0] += CppAD::pow(vars[v_start +t] - ref_v,2);
    }
    //THESE TWO LINES DEAL WITH STEARING
    for (int t =0; t<N-1;t++){
      fg[0]+= 5*CppAD::pow(vars[delta_start + t],2);
      fg[0]+= 5*CppAD::pow(vars[a_start + t],2);
    }
   // THESE TWO WITH STEARING AT TIME T+1
    for(int t=0; t<N-2;t++){
      fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
      fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2); // Smoother transition
    }
```
* We then add some values to this cost depending on *velocity error, trajectory error and cross-track error*.
The parameters are chosen from the Walkthrough, I tweak those a bit.
The first parameter : 2000 went to 3000 because I realized I could deal with 2 laps
The second parameter : 2000 went to 3000 because I realized I could never crash. *This is how much the tuning parameter is important*. 
I then ajusted the last parameter to have smoother transitions and a more comfortable driving.

Important Note : These parameters were tweaked to fit a reference velocity of 100 MPH. If the velocity would be different (faster) due to location or traffic sign detection, we would need the parameters to be tweaked by Twiddle or some more sophisticated algorithm to still fit the line.
* Finally, we setup the rest of the model and constraints with the formulas above.
Important Note : We need to mind the difference of a positive Psi (turn left) meaning turning right for the simulator so the model need to change from
```
fg[2 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
```
to 
```
fg[2 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
```
------

We now have our final car going to the track at 100 mph for hundreds of laps.
![alt text][image4]

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
