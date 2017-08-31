# udacity-carnd-pid-controller
Implementation of PID controller for CarND term2 project 4

---

# Understanding Control
The basic idea is to be able to control a vehicle motion. What does this really mean? A real good way to think about this is to be able to steer a vehicle so that it drives as close to a reference line as possible.


# Understanding a P controller
Given how far the vehicle is from the reference lane (Cross Track Error) - a basic controller can be implemented that manipulates steering angle proportional to the distance from the reference line.

Such a steering angle defined as:

```cpp
double steer_value = -Cp * cte; //constant Cp and Cross Track Error cte
```

As intuitive as this approach seems, the vehicle with this approach doesn't quite converge to the reference line . In theory, it oscillates a good amount - as seen here, it actually gets off track: `Calculated here with a constant of .25`

[P controller video](./videos/P_controller.mp4)


# Understanding a PD controller
Adding a differential part to compute the steering angle helps the angle adjust back as the error gets smaller. This gives slightly accurate results. The differential is generally computed as a diff from the previous error.

Such a steering angle is defined as:

```cpp
double diff_cte = cte - last_cte; //differential as a diff from previous
double steer_value = -Cp * cte  - Cd * diff_cte; //new differential term with constant Cd
```
This makes a slight improvement to the control of the car. Still seems unstable but makes a lap around the track `Calculated with Cp as .25 and Cd as 3`

[PD controller video](./videos/PD_controller.mp4)

# Understanding a PID controller
Now, adding an integral term to our compute above helps to account for a vehicle's systematic bias, eg: natural steering angle always is a few degrees off from the center. This is important since, PD controller cannot handle such a situation and will lead to a vehicle that never converges to the reference line.
The integral term is computed by keeping a running sum of all the errors seen so far - This way, as the error grows, the vehicle automatically adjusts the steering angle proportional to that sum.

```cpp
int_cte += cte; //integral computed as a running sum
double diff_cte = cte - last_cte; //differential as diff from previous
double steer_value = -Cp * cte - Kd * diff_cte - Ci * int_cte; //new integral term with constant Ci
```

`Calculated with Cp as .3158, Cd as 3.5467 and Ci as 0 (no systemic bias) - Look at twiddling below`

[Final PID controller with params from twiddling](./videos/PID_controller.mp4)

## Project, Challenge and Twiddling

Even though I conceptually understood the PID controller and twiddling process and its implementation in python. It took me a little while to get it done in C++. This was only half battle won. The configuring of the params to get the car to drive around the track was the rest of the challenge. For this I added a routine to implement the twiddle algorithm.  With some manual selected params, and a quite a few iterations, I was able to get the needed result. As always the student community and mentors were really helpful in getting the final results.

### Learnings:
I think I could've first tried to implement smaller pieces as done in the chapter. The P controller first, PD next and so on. I made many mistakes while trying to implement the whole PID controller along with the twiddling algorithm. This took me much longer to finish the project. This becomes an important point since, we realize that there is no systemic bias in the simulator, so PD controller really gets you across the track.
Also, I could have tweaked some parameter manually to get the car to finish 1 lap and then see the effect of twiddle.

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
