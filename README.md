# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## PID Definition
![PID](./img/PID.png) 

PID is designed to continuously calculates an error ( based on a setpoint and the real measure), use this error to compute an adjustement to the process. There are three components involved in a PID controller.

* P : Proportional to the error => Kp * e
* I : Integral to the error => Ki * sum(e)
* D : Derivative to the error => Kd * de/dt

Kp, Ki, Kd are the hyperparameters that needs to be tweak for every process we want to use a PID controller.

* P has the most effect on the controller, being directly proportional to the error. If the error is large the control output will be large as well which tend to overshoot the target.

* I has an effect when the system has a bias that make the error moving slowly. In that case the integral term can correct the situation

* D has an effect when there is a rate change in the error. So it is used to stabilize the controller, especially the P parameter that tends to give strong control parameter. 

## Implementation

For this project i implemented two PID controllers. One for the steering and one for the throttle. The PID controller has a build-in optimization function using Twiddle. 

The PID for the steering use the CTE (Cross track error) to provide a control command for the steering. I didn't use the Integral term because there is no bias on the contol in the simulator (i keep it during optmization but it always ended at zero). 
The PID for the throttle is not using a target speed as usual but the derivative CTE only if we are going away from the center. I used a target speed at the beginning but was not able to achieve high speed, so it use another approach just to increase the top speed.

The optimization process does not try to reduce the error but to increase the average spees around the track.

To find the value of the hyperparameter, i first use a manual approach by fixing the throttle to 0.3 and then increase P of the steering PID until it starts to oscilate, then reduce it a bit and increase D of the steering PID until i can do laps.

After this manual setup, i let the automatic optimization run to find first some value to drive around the track with a max CTE so he car doesn't go too wide on the track. That's the one you will get if you just run `./pid`
Then i did it again to achieve higher speed, to do so you have to start it with parameters `./pid 0.068 1.28 5.57 -10-75`

## Results

* Here is the video with safe parameters : [Video low speed](https://youtu.be/jPWIDyRLVmY) 
* Here is the video with maximum speed that i could get : [Video high speed](https://youtu.be/AmmU8ckJJ1I)

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 
