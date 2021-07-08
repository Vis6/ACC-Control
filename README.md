# ACC-Control

## Description
- In this project, two steering control methods, PID control and reinforcement learning (Q-learning) control, are implemented.
- The simulation environment is based on CARLO (circular road). Two cars are placed in the environment. The CARLO GitHub is here: https://github.com/Stanford-ILIAD/CARLO
- Two main program files are included in the project.
  - "main_pid.py" uses PID steering control.
  - "main_rl.py" uses reinforcement learning for steering control.
- The implementations of PID control and reinforcement learnning method are in the "steering_control.py" file.

## How to run the code
- You can run the programs either in the command line or in an IDE.
- PID steering control requires no additional input parameters. The tuned PID parameters have already been given in the program ("steering_control.py").
- Reinforcement learning steering control requires an additional parameter "rl_mode" whose value is "train" or "test". The parameter needs to be specified in the program.
  - If "rl_mode" is "train", the program will learn a policy for every car and the learned policy will be stored in the current directory with the name of "car#_rl.pkl" (# represents the car number).
  - If "rl_mode" is "test", the program will load pre-learned policies for cars and run the simulation.
