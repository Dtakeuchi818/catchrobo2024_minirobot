import math

import numpy as np


def check_position(
        position,
        shere_area_x, robot_area_point, team_color
):
    x, y = position[0], position[1]

    shere_area_check = x < shere_area_x
    robot_area_check_x = robot_area_point[0] < x
    if team_color == 'blue':
        robot_area_check_y = y < robot_area_point[1]
    elif team_color == 'red':
        robot_area_check_y = robot_area_point[1] < y
    robot_area_check = robot_area_check_x or robot_area_check_y

    if not (shere_area_check and robot_area_check):
        print(f'座標： {position} は動作禁止エリア内です')
        raise ValueError
    else:
        return


def check_theta(
        theta, theta1_range, theta2_range,
):
    theta1_check = theta1_range[0] < theta[0] < theta1_range[1]
    theta2_check = theta2_range[0] < abs(theta[1]) < theta2_range[1]

    if not (theta1_check and theta2_check):
        print(f'関節角度: {np.degrees(theta)} は関節角度の動作範囲外です')
        raise ValueError
    else:
        return


def solve_fk(
        theta, l1, l2,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
):
    # 関節角度動作範囲外にならないかチェック
    check_theta(
        theta, theta1_range, theta2_range
    )

    # 順運動学の計算
    theta1, theta2 = theta[0], theta[1]
    theta12 = theta1 + theta2
    x = l1 * np.cos(theta1) + l2 * np.cos(theta12)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta12)
    position = np.array([x, y])

    # 動作禁止領域に入らないかチェック
    check_position(
        position, shere_area_x, robot_area_point, team_color
    )

    return position


def solve_ik(
        position, l1, l2, theta2_reverse,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range,
):
    # 動作禁止領域に入らないかチェック
    check_position(
        position, shere_area_x, robot_area_point, team_color
    )

    # theta2（第2関節角度）の計算
    x, y = position[0], position[1]
    cos2 = (x ** 2 + y ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    try:
        theta2 = np.arccos(cos2) if not theta2_reverse else -np.arccos(cos2)
    except ValueError:
        print(f'座標: {position} は到達不可能な座標です')
        raise
    else:
        sin2 = np.sin(theta2)

    # theta1（第1関節角度）の計算
    k1 = l2 * sin2
    k2 = l1 + l2 * cos2
    sin1 = -k1 * x + k2 * y
    cos1 = k2 * x + k1 * y
    theta1 = np.arctan2(sin1, cos1)

    theta = np.array([theta1, theta2])
    # 関節角度動作範囲外にならないかチェック
    check_theta(
        theta, theta1_range, theta2_range
    )

    return theta


def calc_lspb_params(acc_time, move_time):
    max_vel = 1 / (move_time - acc_time)
    max_acc = max_vel / acc_time
    return max_vel, max_acc


def decide_trajection_type(
        start_pos, goal_pos,
        trajection_type_separate_point, team_color
):
    sep_point_x = trajection_type_separate_point[0]
    sep_point_y = trajection_type_separate_point[1]
        
    trajection_area = []
    for pos in (start_pos, goal_pos):
        x, y = pos[0], pos[1]
        if x < sep_point_x:
            trajection_area.append('C')
        elif team_color == 'blue':
            if sep_point_y < y:
                trajection_area.append('A')
            else:
                trajection_area.append('B')
        elif team_color == 'red':
            if y < sep_point_y:
                trajection_area.append('A')
            else:
                trajection_area.append('B')

    print(trajection_area)

    if trajection_area[0] == trajection_area[1]:
        workspace_trajection = True
        two_steps_trajection = False
    elif set(trajection_area) == {'A', 'B'}:
        workspace_trajection = True
        two_steps_trajection = False
    elif set(trajection_area) == {'B', 'C'}:
        workspace_trajection = False
        two_steps_trajection = False
    elif trajection_area == ['A', 'C']:
        workspace_trajection = True
        two_steps_trajection = True
    elif trajection_area == ['C', 'A']:
        workspace_trajection = False
        two_steps_trajection = True
    else:
        raise RuntimeError

    return workspace_trajection, two_steps_trajection


def calc_lspb_s(
        time, acc_time, move_time, max_vel, max_acc
):
    if 0 <= time < acc_time:
        s = max_acc * time ** 2 / 2
    elif acc_time <= time < move_time - acc_time:
        s = max_vel * (time - acc_time / 2)
    elif move_time - acc_time <= time <= move_time:
        s = 1 - max_acc * (move_time - time) ** 2 / 2
    else:
        print('目標地点に到達しました')
        raise RuntimeError
    return s


def jointspace_lspb(
        start_theta, goal_theta, time,
        acc_time, move_time, max_vel, max_acc,
        l1, l2,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
):
    s = calc_lspb_s(
        time, acc_time, move_time, max_vel, max_acc
    )
    theta = start_theta * (1 - s) + goal_theta * s

    position = solve_fk(
        theta, l1, l2,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
    )

    return theta, position


def workspace_lspb(
        start_pos, goal_pos, time,
        acc_time, move_time, max_vel, max_acc,
        l1, l2, theta2_reverse,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
):
    s = calc_lspb_s(
        time, acc_time, move_time, max_vel, max_acc
    )
    position = start_pos * (1 - s) + goal_pos * s

    theta = solve_ik(
        position, l1, l2, theta2_reverse,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
    )

    return theta, position
    
    

