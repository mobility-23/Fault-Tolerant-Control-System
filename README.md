# Fault-Tolerant-Control-System

We propose a novel data-driven optimal adaptive fault-tolerant control method for path tracking control .

### Instructions to run:

- Clone the repository: `git clone https://github.com/mobility-23/Fault-Tolerant-Control-System`
- Open in MATLAB and open the `A_main.m`.
- Set fault types as needed.
- Comment out section `10. GP_training` before execution.
- Train GP after running the `A_main.m` and save the training data.
- Execute the `A_main.m` and store data.

### Results:

- Results include  `path curvature`  `path tracking performance`  `control commands`  `lateral error`  `heading error`  `station error`  `velocity error`  and  `fault detection results`.  

### Observations and Points to Remember:

- The main program and GP training part need to be separated for computation.
- The parameters of MPC, Kalman filter, fault detector and GP need to be reset.
