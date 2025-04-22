## Settings

* Speed max: 200
* cube gettings:
    - start: (x, y, theta) = (1, 1.5, 0)
    - end: (x, y, theta) = (-1.5, -1, -90 deg)
* Coppeliam sim file:
    - joint limit protection turned on: full_trajectory_protection.csv
    - joint limit protection turned off: full_trajectory_no_protection.csv
* End effector error file:
    - joint limit protection turned on: end_effector_error_protection.csv
    - joint limit protection turned off: end_effector_error_no_protection.csv
* End effector error plot:
    - joint limit protection turned on: error_plot_protection.png
    - joint limit protection turned off: error_plot_no_protection.png
* Program log: 
    - joint limit protection turned on: log-2025-04-03 21:21:17.745412.log
    - joint limit protection turned off: log-2025-04-03 21:25:47.882854.log
* Animation: 
    - joint limit protection turned on: new_task_protection_animation.mp4
    - joint limit protection turned off: new_task_no_protection_animation.mp4

## Controller

* Type: feedforward + PI controller
* P gains: 1.5
* I gains: 8
