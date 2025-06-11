import sys
import math
import datetime
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
import controller
import trajectory
import odometry

def test_joint_limits(joint_angles: list[float]) -> list[int]:
    """
    ## Description

    Check `joint_angles[2]` and `joint_anagles[3]`, see if they exceed pre-defined limits

    ## Input

    Robot arm joint angles

    ## Output

    A list that contains offending joint index

    """

    joint2_min, joint2_max = -1.0472, 2.53
    joint3_min, joint3_max = -1.780, 1.780

    collision_joints = []
    if joint_angles[2] < joint2_min or joint_angles[2] > joint2_max:
        collision_joints.append(2)

    if joint_angles[3] < joint3_min or joint_angles[3] > joint3_max:
        collision_joints.append(3)

    return collision_joints

def get_transformation_from_trajectory(curr: list) -> np.array:
    """
    ## Description

    A helper function that unpacks trajectory function into transformation matrix
    """

    r11 = curr[0]
    r12 = curr[1]
    r13 = curr[2]
    r21 = curr[3]
    r22 = curr[4]
    r23 = curr[5]
    r31 = curr[6]
    r32 = curr[7]
    r33 = curr[8]
    px = curr[9]
    py = curr[10]
    pz = curr[11]

    return np.array([
        [r11, r12, r13, px],
        [r21, r22, r23, py],
        [r31, r32, r33, pz],
        [0, 0, 0, 1]
    ])

def full_program(robot_config: list[float],
                 T_sc_init: np.array,
                 T_sc_final: np.array,
                 T_se_init: np.array,
                 Kp: np.array,
                 Ki: np.array,
                 dt: float,
                 speed_max: float,
                 full_trajectory_filename: str,
                 end_effector_error_filename: str) -> None:
    """
    ## Description

    A full program that generate trajectory for simulation

    ## Input

    1. The actual configuration of robot that contains 13 vectors, robot_config
    2. The initial resting configuration of the cube object, T_sc_init
    3. The final resting configuration of the cube object, T_sc_final
    4. The initial configuration of the reference trajectory for the end-effector, T_se_init
    5. The proportional gains for feedback controller, Kp
    6. The integral gains for feedback controller, Ki
    7. The time interval (this should be kept as 0.01, otherwise the function will raise exception), dt
    8. The maximum speed when calculation odometry, speed_max
    9. The CSV filename for trajectory, full_trajectory_filename
    10. The CSV filename for end-effector errors, end_effector_error_filename

    ## Output

    1. A CSV file that saves to `full_trajectory_filename`, this file contains the trajectory for simulation
    2. A CSV file that saves to `end_effector_error_filename`, this file contains the error of end-effector from `controller`
    """

    # Settings
    print("Full program, start")

    odom = odometry.Odometry()
    traj_generator = trajectory.Trajectory()
    ctrler = controller.Controller()

    angle_grasp = 3 * math.pi / 4
    T_ce_grasp = np.array([
        [math.cos(angle_grasp),  0, math.sin(angle_grasp), 0],
        [0,                      1, 0,                     0],
        [-math.sin(angle_grasp), 0, math.cos(angle_grasp), 0],
        [0,                      0, 0,                     1]
    ])

    T_ce_standoff = np.copy(T_ce_grasp)
    T_ce_standoff[2, 3] = 0.2

    F = odom.get_F_matrix()
    T_b0 = ctrler.get_base_to_arm_configuration()
    M_0e = ctrler.get_arm_home_configuration()
    B_list = ctrler.get_arm_screw_axes()

    # Generate reference trajectory
    print("Full program, generate trajectory")
    curr_trajs = traj_generator.generate_trajectory(T_se_init,
                                                    T_sc_init,
                                                    T_sc_final,
                                                    T_ce_grasp,
                                                    T_ce_standoff)
    
    # Loop reference trajectory and run controller
    print("Full program, run controller")

    full_trajs = []
    end_effector_errs = []

    full_trajs.append(robot_config)

    N = len(curr_trajs)
    for i in range(N - 1):
        # Unpacke joint angles
        base_config = robot_config[0:3]
        joint_angles = robot_config[3:8]

        # Generate end-effector transformation from trajectory
        T_se_d = get_transformation_from_trajectory(curr_trajs[i])
        T_se_d_next = get_transformation_from_trajectory(curr_trajs[i + 1])

        # Generate base transformation from trajectory
        phi = base_config[0]
        base_x = base_config[1]
        base_y = base_config[2]

        T_sb = np.array([
            [math.cos(phi), -math.sin(phi), 0, base_x],
            [math.sin(phi),  math.cos(phi), 0, base_y],
            [0,              0,             1, 0.0963],
            [0,              0,             0, 1]
        ])

        # Get end-effector configuration relative to space frame
        T_0e = mr.FKinBody(M_0e, B_list, joint_angles)
        T_be = T_b0 @ T_0e
        T_se = T_sb @ T_be

        # Check if there are collision joints
        collision_joints = test_joint_limits(joint_angles)
        collision_joints = []

        # Run controller
        curr_output, curr_end_effector_err = ctrler.feedback_control(robot_config,
                                                                     F,
                                                                     T_se,
                                                                     T_se_d,
                                                                     T_se_d_next,
                                                                     Kp,
                                                                     Ki,
                                                                     dt,
                                                                     collision_joints)
        
        # Collect control output and run odometry
        u = curr_output[:4]
        theta_dot = curr_output[4:]
        speed = theta_dot + u

        robot_config = odom.next_state(robot_config, speed, dt, speed_max)

        # Store current result
        full_trajs.append(np.concatenate((robot_config, curr_trajs[i][12]), axis=None).tolist())
        end_effector_errs.append(curr_end_effector_err.tolist())

    
    # Save data
    print("Full program, save result")
    np.savetxt(full_trajectory_filename, full_trajs, delimiter=',')
    np.savetxt(end_effector_error_filename, end_effector_errs, delimiter=',')

    print("Full program, done")
    return

# Test
if __name__ == '__main__':
    print("Start running full program. After this line, all the console output will be saved to log file.")

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

    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    with open(f"./log-{timestamp}.log", 'w') as log:
        sys.stdout = log

        # Print settings
        print(f"robot_config: \n{robot_config}")
        print(f"T_sc_init: \n{T_sc_init}")
        print(f"T_sc_final: \n{T_sc_final}")
        print(f"T_se_init: \n{T_se_init}")
        print(f"Kp: \n{Kp}")
        print(f"Ki: \n{Ki}")
        print(f"dt: \n{dt}")
        print(f"speed_max: \n{speed_max}")
        print(f"full_trajectory_filename: \n{full_trajectory_filename}")
        print(f"end_effector_error_filename: \n{end_effector_error_filename}")

        full_program(robot_config,
                     T_sc_init,
                     T_sc_final,
                     T_se_init,
                     Kp,
                     Ki,
                     dt,
                     speed_max,
                     full_trajectory_filename,
                     end_effector_error_filename)
        
    # Open end effector error file to plot errors
    errors = np.genfromtxt(end_effector_error_filename, delimiter=',')

    N = errors.shape[1]
    M = errors.shape[0]
    times = np.linspace(0, M * dt, M)

    for i in range(N):
        plt.plot(times, errors[:, i], label = f"Xerr{i}")
    plt.legend()
    plt.show()