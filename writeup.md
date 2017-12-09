
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

NOTE :
My first submission was wrong because it crashed on the reviewer's computer. Here is a proof of the project working on mine.
https://github.com/Jeremy26/mpc-controller/blob/master/report/drive_a_lap.mov
---
