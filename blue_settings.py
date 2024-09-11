import numpy as np

from common_settings import *
import calculate_funcs as calf


TEAM_COLOR = 'blue'
WORK_AREA_THETA2_REVERSE = False

THETA1_RANGE = np.radians(np.array((-160, 50)))

ROBOT_AREA_POINT = np.array((165, -135))
TRAJECTION_TYPE_SEPARATE_POINT = np.array((100, -5))
TRAJECTION_CHANGE_POINT = np.array((303.75, -355.0))
TRAJECTION_CHANGE_THETA_HAND_UP = calf.solve_ik(
    TRAJECTION_CHANGE_POINT,
    L1, L2_HAND_UP, WORK_AREA_THETA2_REVERSE,
    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
    THETA1_RANGE, THETA2_RANGE,
)
TRAJECTION_CHANGE_THETA_HAND_DOWN = calf.solve_ik(
    TRAJECTION_CHANGE_POINT,
    L1, L2_HAND_DOWN, WORK_AREA_THETA2_REVERSE,
    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
    THETA1_RANGE, THETA2_RANGE,
)

HOME_POS = np.array((70, -230))
WORK_AREA_POINTS = (
    np.array((566.25, -355.0)),
    np.array((566.25, -135.0)),
    np.array((303.75, -135.0)),
    np.array((303.75, -355.0)),
    np.array((566.25, 102.5)),
    np.array((566.25, 317.5)),
    np.array((303.75, 317.5)),
    np.array((303.75, 102.5)),
)
SHOOTING_AREA_POINTS = (
    np.array((-110.0, -420.0)),
    np.array((-110.0, -270.0)),
    np.array((-300.0, -420.0)),
    np.array((-300.0, -270.0)),
    np.array((-490.0, -420.0)),
    np.array((-490.0, -270.0)),
)
HOME_THETA = calf.solve_ik(
    HOME_POS,
    L1, L2_HAND_UP, WORK_AREA_THETA2_REVERSE,
    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
    THETA1_RANGE, THETA2_RANGE
)
WORK_AREA_THETAS_HAND_UP = [
    calf.solve_ik(
        position,
        L1, L2_HAND_UP, WORK_AREA_THETA2_REVERSE,
        SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        THETA1_RANGE, THETA2_RANGE,
    ) for position in WORK_AREA_POINTS
]
WORK_AREA_THETAS_HAND_DOWN = [
    calf.solve_ik(
        position,
        L1, L2_HAND_DOWN, WORK_AREA_THETA2_REVERSE,
        SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        THETA1_RANGE, THETA2_RANGE,
    ) for position in WORK_AREA_POINTS
]
SHOOTING_AREA_THETAS_HAND_UP = [
    calf.solve_ik(
        position,
        L1, L2_HAND_UP, not WORK_AREA_THETA2_REVERSE,
        SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        THETA1_RANGE, THETA2_RANGE,
    ) for position in SHOOTING_AREA_POINTS
]
SHOOTING_AREA_THETAS_HAND_DOWN = [
    calf.solve_ik(
        position,
        L1, L2_HAND_DOWN, not WORK_AREA_THETA2_REVERSE,
        SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        THETA1_RANGE, THETA2_RANGE,
    ) for position in SHOOTING_AREA_POINTS
]

for t in WORK_AREA_THETAS_HAND_DOWN:
    print(np.degrees(t))