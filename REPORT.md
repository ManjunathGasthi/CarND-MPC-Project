# Model Predictive Control
## Compilation
The code compiles with warnings which were mostly already in the original code. The warnings are about comparing signed and unsigned integers.

## Implementation

### The model

The model used is the same model presented in class. The state is composed of six variables:
* x position of the vehicle.
* y position  of the vehicle.
* Psi orientation of the vehicle.
* v linear velocity of the vehicule, in the direction of psi.
* Cross Track Error: the shortest distance to the trajectory where the vehicle should be.
* epsi: the difference between current orientation and the orientation that the vehicule should have.

And we have two actuators:
* Steer value: controls (By using some transforms implying the constant Lf) the steering of the front wheels and thus the orientation of the car.
* Throttle: controls acceleration and braking, changes to the linear velocity.

The equations for the update between two timesteps *t* and *t-1* separated by *dt* is as follows:

![Update Equations](https://github.com/hectorratia/CarND-MPC-Project/blob/master/updateequations.png?raw=true "Update equations")

In the project the equations are applied in car coordinates and set all inputs to the angle reference used in class, which is sometimes different than the reference used by the simulator. This allows to keep all signs in the equation the same as in class, but requires to make sure that the signs for steering are taking care of before returning to the simulator. The function *f(x)* is the grade 3 polynomial aproximation of the waypoints trajectory in car coordinates.

### Timestep length and elapsed duration

The values used in the MPC quizzes are used as starting point. After that, the values are tuned for performance, taking into account as well response time and length of the predicted trajectory. The interval *dt=0.05* is deemed as appropiate resolution for this simulation. *N* is then adjusted to be able to cover a distance enough to plan a recovery trajectory from most mistakes the model makes, but still within adequate response time. Thus *N=15* is chosen. In my particular hardware values *N>30* will cause fatal delays bigger than the project limitation of 100ms delay.

### Model Predictive Control with Latency

Two strategies were tried out to deal with latency, one worked the other not:
* **Modeling sent actuations and their latency in the MPC module**: this consists of keeping a record of past commands and trying to set them as constraints in the linear equation solver. One should then return a couple timesteps ahead of the initial timestep, to take into account the latency. This idea didn't work out very well. Causes for failure involve the delicacy of timing when to use which actuation (As the controller is queried at irregular intervals) and the unknown relation between actuations and their effects in the simulator.
* **Updating the state by _dt=latency_ before the MPC module**: this strategy consists of simulating a single timestep with *dt=100ms* and sending the result as starting point for the MPC module. The actuations returned to the simulator are the ones corresponding to the first timestep. This worked out fine for all the speeds which were tried, i.e. up to 80mph.

### Polynomial fitting and MPC preprocessing
Each timestep the waypoints received from the simulator were transformed to car coordinates **after** updating the step with the latency *dt=100ms*. Then a polynomial of 3rd degree is fitted to the points. Since there are always six waypoints, it is always possible to fit a 3rd degree polynomial. The coefficients are passed to the MPC module to perform *cte* and *epsi* calculations.

The required preprocessing is thus:
* Updating the stated based on the latency.
* Convert variables to the right units and convention.
* Transform all coordinates into car coordinates.

## Simulation

My code has been lightly modified to allow the weights for the error function used in the linear equation solver to be parametrised from main.cpp. This allowed me faster compilation times while finetuning the weights. The weights I used are contained in line 165 of file main.cpp. Those are the weights for the car to drive at close 80mph. Lines 166-172 contain the weights required to drive at other speeds in 5mph intervals.

In other projects there were problems for the reviewers to attain the same results on their machines as I was getting on mine. To prevent this I have uploaded videos of my MPC Controller driving the car around the track at speeds 60 and 80mph:
* 60 mph:
* 80 mph:
