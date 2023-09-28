# README
* Edmund Lam, Fa 2022/ Sp 2023
* Alex Drake, Fa 2021/ Sp 2022

## File Organization

### Modeling

* `rhs.m` (and `rhs_init.m`) perform the physics timesteps used to simulate the bike.
* `findLQR.m` finds the optimal gains matrix for the `lqr` model given the bicycle parameters.

### Testing
* `currentTests.m` contains the current set of runnable tests.
* `fitnessTest.m` generates graphs (and saves them to `results/<timestamp>/fitness` measuring the fitness of different LQR gain matrices).
* `SpeedTest.m` tests the lowest speed at which the bicycle dynamics model can stay balanced.
* `runBicycleTestR.m` and `runBicycleTestNonlinear.m` run bicycle tests given an initial state, parameters, a gains matrix, and some lag information, returning whether the bicycle succeeded in balancing.

Primary credit to original files in Matlab-Optimization-master. 
Adapted to have flexible parameters and to allow factoring in delays in the control loop. 
animateBike, CircleAboutZ, drawBike, and rhs are identical to the files in Matlab-Optimization-master. 
