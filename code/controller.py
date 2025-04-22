import math
import numpy as np
import odometry
import modern_robotics as mr

class Controller:
    def __init__(self):
        # Fixed offset from chassis frame {b} to base frame of the arm {0}
        self.T_b0 = np.array([
            [1, 0, 0, 0.1662],
            [0, 1, 0, 0],
            [0, 0, 1, 0.026],
            [0, 0, 0, 1]
        ])

        # When the arm is at its home configuration, the end-effector frame {e} relative to the arm base frame {0}
        self.M_0e = np.array([
            [1, 0, 0, 0.033],
            [0, 1, 0, 0],
            [0, 0, 1, 0.6546],
            [0, 0, 0, 1]
        ])

        # When aram is at its home configuration, the screw axes `B` for the five joints are expressed in the
        # end-effector frame {e}
        self.B_list = np.array([
            [0,  0, 1,  0,      0.033, 0],
            [0, -1, 0, -0.5076, 0,     0],
            [0, -1, 0, -0.3526, 0,     0],
            [0, -1, 0, -0.2176, 0,     0],
            [0,  0, 1,  0,      0,     0]
        ]).T

        # Member that records integral error
        self.integral_err = np.zeros((6,), dtype=float)
    
    def get_base_to_arm_configuration(self) -> np.array:
        return self.T_b0

    def get_arm_home_configuration(self) -> np.array:
        return self.M_0e
    
    def get_arm_screw_axes(self) -> np.array:
        return self.B_list

    def feedback_control(self, 
                         robot_config: list[float],
                         F: np.array,
                         T_se: np.array,
                         T_se_d: np.array,
                         T_se_d_next: np.array,
                         Kp: np.array,
                         Ki: np.array,
                         dt: float,
                         collision_joints: list[int] = None) -> tuple[list[float], np.array]:
        """
        ## Description

        Calculate desired twist to control mobile base and robot arm

        ## Input

        1. The configuration of robot arm which is obtained from odometry, robot_config
        2. The pseudo inverse of H(0) of mobile base, F
        3. The current actual end-effector configuration X (or T_se)
        4. The current end_effector reference configuration X_d (or T_se_d)
        5. The end-effector reference configuration at the next timestep in the reference trajectory, X_d_next (or
           T_se_d_next) at `dt` later
        6. The PI gain matrices K_p and K_i
        7. The timestep `dt` between reference trajectory configurations
        8. The configuration of the robot, calculated by `Odometry`

        ## Output

        The control output for mobile base and robot arm, and the error of end-effector
        
        """

        # Extract joint angles from robot configuration
        joint_angles = robot_config[3:8]

        # Calculate `Vd` which is feedforward twist
        Vd = (1 / dt) * mr.MatrixLog6(mr.TransInv(T_se_d) @ T_se_d_next)
        
        Vd = mr.se3ToVec(Vd)

        # Calculate error between end-effector actual configuration and reference configuration
        V_e_err = mr.MatrixLog6(mr.TransInv(T_se) @ T_se_d)
        V_e_err = mr.se3ToVec(V_e_err)

        # Apply equation (13.37) to calculate control output twist
        Ad_T_ed = mr.Adjoint(mr.TransInv(T_se) @ T_se_d)
        Vd_e = Ad_T_ed @ Vd

        self.integral_err += V_e_err * dt
        V = Vd_e + Kp @ V_e_err + Ki @ self.integral_err

        # Calculate jacobian matrix [J_base, J_arm]. Also, if `collision_joint` is given, the corresponding column of
        # offending joints will be set to 0
        J_arm = mr.JacobianBody(self.B_list, joint_angles)

        if collision_joints != None and len(collision_joints) > 0:
            for bad_joint in collision_joints:
                J_arm[:, bad_joint] = 0
        
        T_0e = mr.FKinBody(self.M_0e, self.B_list, joint_angles)
        T_b0 = self.T_b0
        Ad_T_eb = mr.Adjoint(mr.TransInv(T_0e) @ mr.TransInv(T_b0))
        
        F6 = np.array([
            np.zeros((F.shape[1],)),
            np.zeros((F.shape[1],)),
            F[0],
            F[1],
            F[2],
            np.zeros((F.shape[1],)),
        ])
        J_base = Ad_T_eb @ F6

        Je = np.concatenate((J_base, J_arm), axis=1)
        Je_inv = np.linalg.pinv(Je)

        # Calculate final twist command
        command = Je_inv @ V

        return (command.tolist(), V_e_err)
    

# Test
if __name__ == '__main__':
    controller = Controller()
    odom = odometry.Odometry()

    robot_config = [0, 0, 0, 0, 0, 0.2, -1.6, 0, 0, 0, 0, 0, 0]

    F = odom.get_F_matrix()

    T_se = np.array([
        [ 0.170, 0, 0.985, 0.387],
        [ 0,     1, 0,     0],
        [-0.985, 0, 0.170, 0.570],
        [ 0,     0, 0,     1]
    ])

    T_se_d = np.array([
        [ 0, 0, 1, 0.5],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0.5],
        [ 0, 0, 0, 1]
    ])

    T_se_d_next = np.array([
        [ 0, 0, 1, 0.6],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0.3],
        [ 0, 0, 0, 1]
    ])

    Kp = np.zeros((6, 6))

    Ki = np.zeros((6, 6))

    dt = 0.01

    U = controller.feedback_control(
        robot_config,
        F,
        T_se,
        T_se_d,
        T_se_d_next,
        Kp,
        Ki,
        dt
    )