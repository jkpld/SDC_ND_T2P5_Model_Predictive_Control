# Model Predictive Controls Project
Self-Driving Car Engineer Nanodegree Program

**The goal of this project is to use a model predictive controller to drive a car around the Udacity simulators lake-track, while also handling latency in the controls.**

## Result
The car successfully drives around the track without leaving the road, but crossing the lane lines. The minimum car speed is >~60 and the maximum car speed is just over 100.

![example_top_speed](example_run_image.png)

## Vehicle model
The vehicle model used was the global kinematic model discussed in the lessons.
* Variables :
  - **r** = (r<sub>x</sub>, r<sub>y</sub>) : car position
  - &psi; : car orientation
  - v : speed; _**v** = v(cos(&psi;), sin(&psi;))_
  - cte : cross-track error
  - e&psi; : the error in the car orientation
  - &delta; : car steering control
  - a : car acceleration
  - L<sub>f</sub> : distance between the front of the car and the center of gravity
  - dt : time step
  - f(x) : polynomial describing the path we want to follow
* Car state : **x** = (r<sub>x</sub>, r<sub>y</sub>, &psi;, v, cte, e&psi;)
* Actuators : **u** = (&delta;, a)
* Update equations :
  - **r**<sub>t+1</sub> = **r**<sub>t</sub> + **v**<sub>t</sub>  dt
  - &psi;<sub>t+1</sub> = &psi;<sub>t</sub> + v<sub>t</sub> * &delta;<sub>t</sub> * dt / L<sub>f</sub>
  - v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> dt
  - cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> sin(e&psi;<sub>t</sub>) dt
  - e&psi;<sub>t+1</sub> = &psi;<sub>t</sub> - atan(df/dx)|<sub>x<sub>t</sub></sub> + v<sub>t</sub> * &delta;<sub>t</sub> * dt / L<sub>f</sub>
* Constraints :
  - &delta; &in; [-25&deg;, 25&deg;]
  - a &in; [-1, 1]

## Model cost function
The cost, C, I used to optimize the actuator values has three terms.
* The direct error given by the cross-track error and orientation error (in degrees)
  - C<sub>direct, t</sub> = (t+1) w<sub>cte</sub> cte<sub>t</sub><sup>2</sup> + w<sub>e&psi;</sub> e&psi;<sub>t</sub><sup>2</sup>.

  The (t+1) term on the cte error weights the error more heavily at time steps further into the future, and the orientation error is put in units of degrees so that the order of magnitudes of the two errors are closer.
* The derivative of the steering angle (in degrees)
  - C<sub>d&delta;, t</sub> = w<sub>d&delta;</sub> (d&delta;/dt)<sup>2</sup>.
* A speed control term
  - C<sub>v, t</sub> = w<sub>v</sub> (v - v<sub>ref</sub>)<sup>2</sub>

  where v<sub>ref</sub> is a reference speed.

#### Cost parameters
The weight factors, w<sub>(.)</sub> terms, and the reference speed are given by
* w<sub>cte</sub> = 20
* w<sub>e&psi;</sub> = 20
* w<sub>d&delta;</sub> = 2 + 80 RoC<sup>1/2</sup>
* w<sub>v</sub> = 30
* v<sub>ref</sub> = 20 + 170 RoC

where RoC is the "normalized" radius of curvature of the polynomial fitted to the waypoints and has a value &in; [0,1] (see below). The RoC value is close to zero for sharp turns and close to one on very straight roads. Thus, when driving on a straight road, w<sub>d&delta;</sub> is very large, which significantly damps out any change in the cars steering angle; however, when the road has a sharp turn, w<sub>d&delta;</sub> becomes small, allowing for large, but smooth, changes in the steering angle. Likewise, the reference speed is very large when the road is straight, but during a turn, it becomes quite small.

#### Road curvature
If the path we want to travel along (the road) is given by a polynomial f(x), then

  roc &leftarrow; (1 + (df/dx)<sup>2</sup>)<sup>3/2</sup> / |d<sup>2</sup>f/dx<sup>2</sup>|  
  RoC &leftarrow; min(roc, 500)/500

The radius of curvature is evaluated 25 meters in front of the car's current location; this gives lookahead capabilities.

## Modeled time steps and latency compensation
The number of time-steps modeled was a fixed value of N = 10. This allowed for sufficient resolution, while also decreasing the increased modeling time that comes with modeling more time-steps.

#### Latency compensation
Latency is very simple to compensate for using the MPC because the very first time step is simply set to the expected latency, dt<sub>1</sub> = t<sub>latency</sub>.

#### Distance controlled time steps

The duration of the remaining N-1 time-steps, dt<sub>n>1</sub> [sec], was implicitly set by changing the distance, d [m], in front of the car that was modeled.

d = 50 + 30 RoC - t<sub>latency</sub> v<sub>0</sub>  
dt<sub>n>1</sub> = d / ((N-1)*v<sub>0</sub> + &epsilon;),  

where RoC is the same as above. This means that, for a straight road, I model 80 m in front of the car, but when the road turns sharply, I only model 50 m in front of the car.

_Note. When I model with a fixed time, dt=0.1, then the car runs off the road._


## Waypoint fitting
The waypoints returned by the simulator were fit with a 3rd order polynomial.
