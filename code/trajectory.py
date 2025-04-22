import math
import numpy as np
import modern_robotics as mr
import matplotlib.pyplot as plt
from enum import IntEnum

class GripperState(IntEnum):
    OPEN = 0
    CLOSE = 1

class TimeScalingMethod(IntEnum):
    CUBIC = 3
    QUINTIC = 5

class Trajectory:
    def __init__(self):
        self.Tf = 1
        self.N = 100
        self.method = TimeScalingMethod.QUINTIC

    def __flatten(self, all_trajectories: list, curr_trajectories: list[np.array], gripper_state: GripperState) -> None:
        for traj in curr_trajectories:
            curr = np.zeros((1, 13))
            idx = 0

            R = traj[:3, :3]
            for i, j in np.ndindex(R.shape):
                curr[0, idx] = R[i, j]
                idx += 1

            P = traj[:3, 3]
            for val in P:
                curr[0, idx] = val
                idx += 1
            
            curr[0, idx] = gripper_state
            all_trajectories.append(curr[0].tolist())

    def generate_trajectory(self, 
                            T_se_init: np.array,
                            T_sc_init: np.array,
                            T_sc_final: np.array,
                            T_ce_grasp: np.array,
                            T_ce_standoff: np.array,
                            k = 10) -> list:
        """
        ## Description

        Generate trajectory for end-effector that contains 8 concatenated trajectory segments, and each trajectory 
        segment begins and ends at rest. The 8 segments are as follows:

        1. A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
        2. A trajectory to move the gripper down to the grasp position.
        3. Closing of the gripper.
        4. A trajectory to move the gripper back up to the "standoff" configuration.
        5. A trajectory to move the gripper to a "standoff" configuration above the final configuration.
        6. A trajectory to move the gripper to the final configuration of the object.
        7. Opening of the gripper.
        8. A trajectory to move the gripper back to the "standoff" configuration.

        ## Input

        1. The initial configuration of the end-effector in the reference trajectory: T_se_init
        2. The cube's initial configuration: T_sc_init
        3. The cube's desired final configuration: T_sc_final
        4. The end-effector configuration relative to the cube when it is grasping the cube: T_ce_grasp
        5. The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube: T_ce_standoff
        6. The number of trajectory reference configurations per 0.01 seconds: k

        By default, N is set to 100 and k is set to 10, so this function generates 1000 reference points in each segement. You can
        adjust k to control the number of reference points in each segment.

        Note, k is positive integer that is greater than or equal to 1, so if the value is less than 1, it will be kept as 1

        ## Output

        1. A representation of the N configurations of the end-effector along the entir concatenated 8-segment reference trajectory.
           Each of these N reference points represents a transformation matrix T_se plus gripper state (0 or 1).
           r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper state
        """

        all_trajectories = []

        # Segment 1:
        # A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above the block.
        T_se_standoff_pick = T_sc_init @ T_ce_standoff
        curr_trajectories = mr.CartesianTrajectory(T_se_init, T_se_standoff_pick, self.Tf, self.N * 8, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.OPEN)

        # Segment 2:
        # A trajectory to move the gripper down to the grasp position.
        T_se_grasp = T_sc_init @ T_ce_grasp
        curr_trajectories = mr.CartesianTrajectory(T_se_standoff_pick, T_se_grasp, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.OPEN)

        # Segment 3:
        # Closing of the gripper.
        curr_trajectories = mr.CartesianTrajectory(T_se_grasp, T_se_grasp, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.CLOSE)

        # Segment 4:
        # A trajectory to move the gripper back up to the "standoff" configuration.
        curr_trajectories = mr.CartesianTrajectory(T_se_grasp, T_se_standoff_pick, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.CLOSE)

        # Segment 5: 
        # A trajectory to move the gripper to a "standoff" configuration above the final configuration.
        T_se_standoff_place = T_sc_final @ T_ce_standoff
        curr_trajectories = mr.CartesianTrajectory(T_se_standoff_pick, T_se_standoff_place, self.Tf, self.N * 5, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.CLOSE)

        # Segment 6:
        # A trajectory to move the gripper to the final configuration of the object.
        T_se_final = T_sc_final @ T_ce_grasp
        curr_trajectories = mr.CartesianTrajectory(T_se_standoff_place, T_se_final, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.CLOSE)

        # Segment 7:
        # Opening of the gripper.
        curr_trajectories = mr.CartesianTrajectory(T_se_final, T_se_final, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.OPEN)

        # Segment 8:
        # A trajectory to move the gripper back to the "standoff" configuration.
        curr_trajectories = mr.CartesianTrajectory(T_se_final, T_se_standoff_place, self.Tf, self.N * 2, self.method)
        self.__flatten(all_trajectories, curr_trajectories, GripperState.OPEN)

        return all_trajectories
    

# Test
if __name__ == '__main__':
    T_se_init = np.array([
        [ 0, 0, 1, 0.5518],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0.4012],
        [ 0, 0, 0, 1]
    ])

    T_sc_init = np.array([
        [1, 0, 0, 1],
        [0, 1, 0, 0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    T_sc_final = np.array([
        [ 0, 1, 0,  0],
        [-1, 0, 0, -1],
        [ 0, 0, 1,  0.025],
        [ 0, 0, 0,  1]
    ])

    T_ce_grasp = np.array([
        [ 0, 0, 1, 0],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0],
        [ 0, 0, 0, 1]
    ])

    T_ce_standoff = np.array([
        [ 0, 0, 1, 0],
        [ 0, 1, 0, 0],
        [-1, 0, 0, 0.25],
        [ 0, 0, 0, 1]
    ])

    tragectory_generator = Trajectory()
    result = tragectory_generator.generate_trajectory(
        T_se_init, 
        T_sc_init,
        T_sc_final,
        T_ce_grasp,
        T_ce_standoff)
    np.savetxt('test_traj.csv', result, delimiter=',')