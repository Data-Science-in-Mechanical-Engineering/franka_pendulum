#!/usr/bin/freecadcmd
import control, rospkg, numpy as np
import sys
sys.path.append(rospkg.RosPack().get_path("franka_pole") + "/scripts/")
import parts

g = 9.81

def lqr():
    lower, upper = parts.get_parts()

    A = np.zeros((4,4))
    A[0,1] = 1
    A[2,3] = 1
    A[1,0] = g * upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[0,0]

    B = np.zeros((4,1))
    B[3,0] = 1
    B[1,0] = -upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[0,0]

    Q = np.diag([100.0, 100.0, 100.0, 100.0])
    R = np.eye(1)
    K,S,E = control.lqr(A, B, Q, R)
    return -K

def lqr_2d():
    lower, middle, upper = parts.get_parts_2d()
    middle_and_upper = parts.PartGroup([middle, upper])
    upper_position = 0.012

    A = np.zeros((8,8))
    A[0,1] = 1
    A[2,3] = 1
    A[4,5] = 1
    A[6,7] = 1
    A[1,0] = g * upper.mass * (upper.position[2] - upper_position) / upper.inertia_at(np.array([0,0,upper_position]))[0,0]
    A[5,4] = g * middle_and_upper.mass * middle_and_upper.position[2] / middle_and_upper.inertia_at(np.zeros(3))[1,1]

    B = np.zeros((8,2))
    B[3,0] = 1
    B[7,1] = 1
    B[1,0] = -upper.mass * (upper.position[2] - upper_position) / upper.inertia_at(np.array([0,0,upper_position]))[0,0]
    B[5,1] = -middle_and_upper.mass * middle_and_upper.position[2] / middle_and_upper.inertia_at(np.zeros(3))[1,1]

    Q = np.diag([100.0, 100.0, 100.0, 100.0, 25.0, 25.0, 25.0, 25.0])
    R = np.eye(2)
    K,S,E = control.lqr(A, B, Q, R)
    return -K

def lqr_2db():
    lower, upper = parts.get_parts_2db()

    A = np.zeros((8,8))
    A[0,1] = 1
    A[2,3] = 1
    A[4,5] = 1
    A[6,7] = 1
    A[1,0] = g * upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[0,0]
    A[5,4] = g * upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[1,1]

    B = np.zeros((8,2))
    B[3,0] = 1
    B[7,1] = 1
    B[1,0] = -upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[0,0]
    B[5,1] = -upper.mass * upper.position[2] / upper.inertia_at(np.zeros(3))[1,1]

    Q = np.diag([100.0, 100.0, 100.0, 100.0, 25.0, 25.0, 25.0, 25.0])
    R = np.eye(2)
    K,S,E = control.lqr(A, B, Q, R)
    return -K

if __name__ == "__main__":
    if sys.argv[-1] == '1D': print(lqr())
    elif sys.argv[-1] == '2D': print(lqr_2d())
    elif sys.argv[-1] == '2Db': print(lqr_2db())
    else: raise Exception("Invalid usage")