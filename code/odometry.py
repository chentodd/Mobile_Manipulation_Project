import math
import numpy as np
import matplotlib.pyplot as plt

class Odometry:
    def __init__(self):
        # Base mechanical parameters and pre-compute 'F', book equation(13.33)
        w = 0.15
        l = 0.235
        r = 0.0475

        self.F = (r / 4) * np.array([
            [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
            [1, 1, 1, 1],
            [-1, 1, -1, 1]
        ])

    def get_F_matrix(self) -> np.array:
        return self.F
    
    def next_state(self, curr_config: list[float], speed: list[float], dt: float, max_speed: float = 1e9) -> list[float]:
        """
        ## Description

        Calculate next robot state

        ## Input

        1. 12-vector representing the current configuration of the robot:
            * 3 variables for chassis configuration
            * 5 variables for arm configuration
            * 4 variables for wheel angels
        2. 9-vector of controls indicating the wheel speeds and joint speeds
            * 5 variables for arm speed
            * 4 variables for base speed
        3. Timestep, dt
        4. A positive real value indicating the maximum angular speed of the arm joints and the wheels
            * By default, it is set to a large value
        ## Output

        1. A 12-vector representing the configuration of the robot after dt:
           chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4

        ## Implementation

        Mainly follow book chapter 13.4, and use equation(13.35) to update robot configuration
        """

        q_k = np.array([
            [curr_config[0]],
            [curr_config[1]],
            [curr_config[2]],
        ])

        curr_joint_angles = np.array([
            [curr_config[3]],
            [curr_config[4]],
            [curr_config[5]],
            [curr_config[6]],
            [curr_config[7]],
        ])

        curr_wheel_angles = np.array([
            [curr_config[8]],
            [curr_config[9]],
            [curr_config[10]],
            [curr_config[11]],
        ])

        # Get next joint configuration after dt
        limited_speeds = np.array(speed)
        for i in range(len(speed)):
            limited_speeds[i] = speed[i] if math.fabs(speed[i]) <= max_speed else np.sign(speed[i]) * max_speed
        joint_theta_dot = np.array([
            [limited_speeds[0]],
            [limited_speeds[1]],
            [limited_speeds[2]],
            [limited_speeds[3]],
            [limited_speeds[4]]
        ])

        delta_joint_theta = dt * joint_theta_dot
        next_joint_angles = curr_joint_angles + delta_joint_theta

        # Get next wheel configuration after dt
        wheel_theta_dot = np.array([
            [limited_speeds[5]],
            [limited_speeds[6]],
            [limited_speeds[7]],
            [limited_speeds[8]],
        ])

        delta_wheel_theta = dt * wheel_theta_dot
        next_wheel_angles = curr_wheel_angles + delta_wheel_theta

        # Get chassis configuration, equation(13.35)
        V_b = self.F @ delta_wheel_theta

        wbz = V_b[0][0]
        vbx = V_b[1][0]
        vby = V_b[2][0]

        if wbz == 0.0:
            delta_qb = np.array([
                [0],
                [vbx],
                [vby]
            ])
        else:
            delta_qb = np.array([
                [wbz],
                [(vbx * math.sin(wbz) + vby * (math.cos(wbz) - 1)) / wbz],
                [(vby * math.sin(wbz) + vbx * (1 - math.cos(wbz))) / wbz]
            ])

        transform = np.array([
            [1, 0, 0],
            [0, math.cos(curr_config[0]), -math.sin(curr_config[0])],
            [0, math.sin(curr_config[0]), math.cos(curr_config[0])],
        ])

        delta_q = transform @ delta_qb
        q_k_1 = q_k + delta_q

        return [
            # Chasis configuration
            q_k_1[0][0],
            q_k_1[1][0],
            q_k_1[2][0],
            # Arm configuration
            next_joint_angles[0][0],
            next_joint_angles[1][0],
            next_joint_angles[2][0],
            next_joint_angles[3][0],
            next_joint_angles[4][0],
            # Wheel configuration
            next_wheel_angles[0][0],
            next_wheel_angles[1][0],
            next_wheel_angles[2][0],
            next_wheel_angles[3][0],
        ]


# Test
if __name__ == '__main__':
    odom = Odometry()

    dt = 0.01
    number_of_iterations = 100

    curr_config = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    speed = [0, 0, 0, 0, 0, -10, 10, 10, -10]
    x_values = [0]
    y_values = np.array([curr_config])

    for i in range(number_of_iterations):
        next_state = odom.next_state(curr_config, speed, dt)
        x_values.append(i)
        y_values = np.vstack([y_values, next_state])
        curr_config = next_state

    plt.plot(x_values, y_values[:,0], label = 'base_phi')
    plt.plot(x_values, y_values[:,1], label = 'base_x')
    plt.plot(x_values, y_values[:,2], label = 'base_y')
    plt.legend()
    plt.show()
