## Description

Control `YouBot` to perform pick and place tasks.

## Project

### Structure

* `code` folder: contains the code for each milestone and the full program
    - `main`: the full program
    - `odometry`: implementation for calculating the state of mobile base and robot arm, which estimates the pose of robot
    - `trajectory`: implementation for trajectory planning
    - `controller`: implementation for feedforward and PI controller

### Run

In order to run the program, you will need following packages:
- `python` (ver.3.12.3 is used when building the project)
- `numpy`
- `matplotlib`
- `modern_robotics`

After all the packages are installed, you can run `main.py`. And if you want to try different input, you can edit following parameters accordingly:

```python
# Settings for full program
robot_config = [-0.75959, -0.47352, 0.058167, 0.80405, -0.91639, -0.011436, 0.054333, 0.00535, 1.506, -1.3338, 1.5582, 1.6136, 0]

T_sc_init = np.array([
    [1, 0, 0, 1],
    [0, 1, 0, 1.5],
    [0, 0, 1, 0.025],
    [0, 0, 0, 1]
])

T_sc_final = np.array([
    [ 0, 1, 0, -1.5],
    [-1, 0, 0, -1],
    [ 0, 0, 1,  0.025],
    [ 0, 0, 0,  1]
])

T_se_init = np.array([
    [ 0, 0, 1, 0],
    [ 0, 1, 0, 0],
    [-1, 0, 0, 0.5],
    [ 0, 0, 0, 1]
])

kp = 80
Kp = kp * np.identity(6)

ki = 100
Ki = ki * np.identity(6)

dt = 0.01

speed_max = 200.0

full_trajectory_filename = "./full_trajectory.csv"
end_effector_error_filename = "./end_effector_error.csv"
```

### Program output

After `main.py` is finished, it will give following outputs:
1. A CSV file that can be used in CoppeliaSim
2. A CSV file that contains end-effector errors
3. A log file that contains input settings and log program messages

### Additional

A very naive joint protection is implemented in `test_joint_limits` in `main.py`. This function will collect offending joint indexes, and pass these indexes to controller. In controller, it will set each column of arm jacobian matrix corresponding to an offending joint to all zeros.

## Results

### Structure

* `result` folder: contains the result of different tasks
    - `best`: run the task with a tuned controller
    - `overshoot`: run the task with a poorly tuned controller
    - `newtask`: run the task with a tuned controller, and turn on/off the protection to see its effect

<video src="./results/newTask/new_task_no_protection_animation.mp4" controls></video>

* protection

<video src="./results/newTask/new_task_protection_animation.mp4" controls></video>
