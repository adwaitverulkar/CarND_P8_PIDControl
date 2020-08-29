# Udacity PID Controller Project

## Author: Adwait Verulkar. Date: 08/28/2020.

### Project Overview

The aim of the project is to implement a PID controller for steering inputs of a vehicle, such that it is able to safely and efficiently maneuver around a test track. The crux is the project lies in hyperparameter tuning, or selecting proper values of Proportional, Integral and Derivative gains such that the car is able to maintain itself at the center of the road throughout the lap. The PID controller is given an input of cross-track error, or the lateral offset of the vehicle longitudinal center and the center of the road. The output of this PID is the steering angle, which is bounded in the region [-1, 1]. In addition to this PID controller for steering, a throttle/brake controller has also been implemented.

### Hyperparameter Tuning for PID Controller

The gains of the PID controller were tuned manually. An optimization algorithm such as twiddle and SGD can be implemented, but running optimization iterations with the simulator in the loop is not feasible. Such algorithms can be run offline, by using a kinematic model of the vehicle. However, as the kinematics of the vehicle are not known, an adaptive controller or a model-predictive controller are more suitable than a PID for this project.  The limits on the steering angle introduce an additional level of complexity as choosing aggressive gains to improve performance leads to control input saturation.

#### Effect of Proportional Gain

The proportional gain hyperparameter produces a control input that increases with increase in cross-track error. Proportional gain alone cannot stabilize a system as it leads to oscillatory response due to perpetual overshoot. This parameter was chosen such that a cross-track error of half the lane width (or more) should saturate the input using proportional gain alone. This ensures that the steering is gradual when the car stays in the track. Very high proportional gains lead to bipolar nature of the vehicle, where the steering angle constantly changes from one extreme to the other.

#### Effect of Integral Gain

The integral gain gives characteristics of a lag controller, or in other words, a controller where the control input associated with an error is applied with a certain delay.  This gain is mainly used to reduce offset error. As the track is constantly changing, there is no constant "goal" position that the controller needs to reach. Hence, this gain has been kept minimum, just enough to ensure no offset error. High values lead to erroneous behavior of the vehicle, as it reacts to errors in the past, when in actuality the track and reference paths have completely changed.

#### Effect of Derivative Gain

The derivative gain is the most important hyperparameter of this controller. It gives rise to lead controller characteristics. This is very important for controlling systems where the reference is constantly changing. In this project, the controller is trying to match the reference of the center of the road. As the road keeps changing constantly (with turns and changing road width), it is this hyperparameter that ensures that the cross-track error stays bounded.

Also, the derivative gain helps the vehicle achieve a steady state when the road is relatively constant (on straight patches). This gain was the only one that required manual tuning. The other gains were held constant throughout all iterations.

#### PID Implementation

The PID controller is implemented as a class in the ```pid.cpp``` file. There are 3 methods in this class, one each for initialization, error calculation and output generation respectively.

The ```PID::Init(double Kp_, double Ki_, double Kd_)``` method is used to initialize the PID controller by setting the gains and errors.

```cpp
void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  is_cte_init = false;
}
```

The ```PID::UpdateError(double cte)``` method, takes the cross track error as input and generates the 3 errors, for proportional, integral and derivative control respectively. It has been assumed that the time step between any two error updates is unity. Time step value is irrelevant and it acts only as a scaling factor. A time step of 0.1 seconds would increase the I gain by a factor of 10 and reduce the D gain by the same factor.

```cpp
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  p_error = cte;
  i_error += cte;
  if(is_cte_init) 
    d_error = cte - cte_old;
  cte_old = cte;
  is_cte_init = true;

}
```

The final method ```PID::TotalError()``` returns the steering angle, by multiplying the gains with their respective errors. It also implements input saturation, by constraining the returned steering value in between -1 and 1.

```cpp
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double steering;
  steering = -Kp * p_error - Ki * i_error - Kd * d_error; 
  if(steering < -1) steering = -1;
  if(steering > 1) steering = 1; 
  return steering;// TODO: Add your total error calc here!
}
```

### Throttle/Brake Controller

The simulator seems to imitate real-life physics, and hence the handling performance deteriorates at high speeds. Also the rate of change of cross-track errors is significantly higher at high vehicle speeds. Due to these reasons, it is important to control the throttle and brake inputs along with the steering inputs.

A simple controller is implemented for the throttle/brake inputs, that depends on the steering value. The boundary conditions for designing this conditions are summarized below.

* apply full throttle when steering is zero
* apply 25% brakes when steering is + / - 1

This logic can be implemented by a linear equation, which takes steering as input and outputs the throttle/brake value. However, there is one problem with this implementation. If the ```steer_value``` is high at the start of the simulation, in other words when the vehicle is at rest, it might make the vehicle go in reverse. Hence the throttle always needs to be a positive value below a threshold vehicle speed. This can be implemented by an ```if-else``` block as follows.

```cpp
if(speed < 10.0)
    throttle_value = 1.0;
else
    throttle_value = 0.75 - abs(steer_value);
```

To maximize performance, the throttle is set to maximum below vehicle speeds of 10 mph.

### Conclusions

The PID controller manages to keep the vehicle on the track while trying to go around the track as fast as possible. However, it is difficult to obtain optimal performance from the controller as the cross-track error tends to abruptly change, which destabilizes the vehicle after every 3-4 seconds in the run. Also, the dynamics of the vehicle are significantly different at high speeds and it would be better if the gains are updated with respect to vehicle speed. An adaptive controller or a model predictive controller are much more appropriate for such applications.