import os
import sys

import pygame as pg

from common_settings import *
import utils
import dynamixel_funcs as dxlf
import calculate_funcs as calf


os.environ["SDL_VIDEODRIVER"] = "dummy"

# =========== PS4コントローラ初期化
TEAM_COLOR = 'blue'
if TEAM_COLOR == 'blue':
    from blue_settings import *

joystick = utils.init_ps4_controler()
print('PS4コントローラの初期化が完了しました')

# =========== Dynamixel初期化・ロボットの初期化
print('-' * 8)
print(f'現在、{TEAM_COLOR} モードで起動しています')
print('PSボタンを押すとロボットが起動します')
print('ホームポジションへの移動を行うので、周囲に注意してください')
utils.wait_until_psbutton_pressed(joystick)
joint_dxls, hand_dxl = dxlf.dynamixels_setup(
    DEVICENAME, BAUDRATE, PROTOCOL_VERSION,
    DXL_PLOFILE_VELOCITY, DXL_PLOFILE_ACCELERATION,
)
hand_up = True
_, L2 = dxlf.hand_updown(
    hand_dxl, hand_up,
    THETA_HAND_UP, THETA_HAND_DOWN, DXL_MAX_POS_VALUE,
    HOME_THETA, L1, L2_HAND_UP, L2_HAND_DOWN,
    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
    THETA1_RANGE, THETA2_RANGE
)
dxlf.rotate_joint_dxls(
    joint_dxls, HOME_THETA, DXL_MAX_POS_VALUE
)
current_theta = HOME_THETA
current_pos = HOME_POS
print('ロボットの起動が完了しました')

# =========== ロボット操作開始
print('-' * 8)
print('PSボタンを押すとロボットの操作を開始します')
utils.wait_until_psbutton_pressed(joystick)

clock = pg.time.Clock()
time = 0.0
lspb_start_theta, lspb_goal_theta = None, None
lspb_start_pos, lspb_goal_pos = None, None
lspb_final_goal_pos, lspb_final_goal_theta = None, None
workspace_trajection, two_steps_trajection = None, None
while True:
    pg.event.pump()

    current_theta2_reverse = current_theta[1] < 0

    # 定点移動モード
    if joystick.get_button(BUTTON_WORKSPACE_TRIGER) or joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER):
        # あらかじめ設定された定点から、各ボタンに割り振られた目標座標と関節角度を選択する
        work_area_thetas = WORK_AREA_THETAS_HAND_UP if hand_up else WORK_AREA_THETAS_HAND_DOWN
        shooting_area_thetas = SHOOTING_AREA_THETAS_HAND_UP if hand_up else SHOOTING_AREA_THETAS_HAND_DOWN

        # # シューティングエリアへ移動する場合、手先を上げる
        # if joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and not hand_up:
        #     hand_up = True
        #     current_pos, L2 = dxlf.hand_updown(
        #         hand_dxl, hand_up,
        #         THETA_HAND_UP, THETA_HAND_DOWN, DXL_MAX_POS_VALUE,
        #         HOME_THETA, L1, L2_HAND_UP, L2_HAND_DOWN,
        #         SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        #         THETA1_RANGE, THETA2_RANGE
        #     )

        # ワークスペース上の定点を選択する
        if joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P1):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[0]
                lspb_goal_pos = WORK_AREA_POINTS[0]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P2):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[1]
                lspb_goal_pos = WORK_AREA_POINTS[1]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P3):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[2]
                lspb_goal_pos = WORK_AREA_POINTS[2]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P4):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[3]
                lspb_goal_pos = WORK_AREA_POINTS[3]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P5[0]] == BUTTON_P5[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[4]
                lspb_goal_pos = WORK_AREA_POINTS[4]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P6[0]] == BUTTON_P6[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[5]
                lspb_goal_pos = WORK_AREA_POINTS[5]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P7[0]] == BUTTON_P7[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[6]
                lspb_goal_pos = WORK_AREA_POINTS[6]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P8[0]] == BUTTON_P8[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[7]
                lspb_goal_pos = WORK_AREA_POINTS[7]

        # シューティングエリア上の定点を選択する
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P1):
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[0]
                lspb_goal_pos = SHOOTING_AREA_POINTS[0]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P2):
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[1]
                lspb_goal_pos = SHOOTING_AREA_POINTS[1]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P3):
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[2]
                lspb_goal_pos = SHOOTING_AREA_POINTS[2]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P4):
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[3]
                lspb_goal_pos = SHOOTING_AREA_POINTS[3]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_hat(0)[BUTTON_P5[0]] == BUTTON_P5[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[4]
                lspb_goal_pos = SHOOTING_AREA_POINTS[4]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_hat(0)[BUTTON_P6[0]] == BUTTON_P6[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = shooting_area_thetas[5]
                lspb_goal_pos = SHOOTING_AREA_POINTS[5]
        
        # TRIGER_BUTTON以外は押されていない場合、何もしない
        else:
            clock.tick(FPS)
            continue
        
        # 現在の座標と関節角度を保存する
        if lspb_start_theta is None:
            lspb_start_theta = current_theta
            lspb_start_pos = current_pos

        # どの軌跡を用いるかを決定する
        if (lspb_goal_pos is not None) and (workspace_trajection is None):
            workspace_trajection, two_steps_trajection = calf.decide_trajection_type(
                lspb_start_pos, lspb_goal_pos,
                TRAJECTION_TYPE_SEPARATE_POINT, TEAM_COLOR
            )

        # ２段階軌跡を用いる場合、目的座標を軌跡切り替わり点の座標に変更する
        # 同時に、本来の目標座標を保存しておく
        if two_steps_trajection and (lspb_final_goal_pos is None):
            lspb_final_goal_pos = lspb_goal_pos
            lspb_final_goal_theta = lspb_goal_theta
            lspb_goal_pos = TRAJECTION_CHANGE_POINT
            lspb_goal_theta = TRAJECTION_CHANGE_THETA_HAND_UP if hand_up else TRAJECTION_CHANGE_THETA_HAND_DOWN

        # 作業空間上の直線軌跡を計算する
        if workspace_trajection:
            try:
                goal_theta, goal_pos = calf.workspace_lspb(
                    lspb_start_pos, lspb_goal_pos, time,
                    ACC_TIME, MOVE_TIME, MAX_VEL, MAX_ACC,
                    L1, L2, current_theta2_reverse,
                    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                    THETA1_RANGE, THETA2_RANGE,
                )

            # 動作禁止エリアに侵入 or 関節角度が範囲外などの場合、無視する
            except ValueError:
                clock.tick(FPS)
                continue

            # 目標座標に到達した場合
            except RuntimeError:
                # ２段階軌跡を用いる場合、２段階目の軌跡に移行する
                if two_steps_trajection:
                    time = 0.0
                    lspb_start_pos = TRAJECTION_CHANGE_POINT
                    lspb_start_theta = TRAJECTION_CHANGE_THETA_HAND_UP if hand_up else TRAJECTION_CHANGE_THETA_HAND_DOWN
                    lspb_goal_pos = lspb_final_goal_pos
                    lspb_goal_theta = lspb_final_goal_theta
                    workspace_trajection = not workspace_trajection
                    two_steps_trajection = False

                clock.tick(FPS)
                continue

        # 関節角度上の直線軌跡を計算する
        else:
            try:
                goal_theta, goal_pos = calf.jointspace_lspb(
                    lspb_start_theta, lspb_goal_theta, time,
                    ACC_TIME, MOVE_TIME, MAX_VEL, MAX_ACC,
                    L1, L2,
                    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                    THETA1_RANGE, THETA2_RANGE,
                )

            # 関節角度が範囲外などの場合、無視する
            except ValueError:
                clock.tick(FPS)
                continue

            # 目標座標に到達した場合
            except RuntimeError:
                # ２段階軌跡を用いる場合、２段階目の軌跡に移行する
                if two_steps_trajection:
                    time = 0.0
                    lspb_start_pos = TRAJECTION_CHANGE_POINT
                    lspb_start_theta = TRAJECTION_CHANGE_THETA_HAND_UP if hand_up else TRAJECTION_CHANGE_THETA_HAND_DOWN
                    lspb_goal_pos = lspb_final_goal_pos
                    lspb_goal_theta = lspb_final_goal_theta
                    workspace_trajection = not workspace_trajection
                    two_steps_trajection = False

                clock.tick(FPS)
                continue

        # 時刻を更新する
        time += 1 / FPS

        current_pos = goal_pos
        current_theta = goal_theta
        dxlf.rotate_joint_dxls(
            joint_dxls, goal_theta, DXL_MAX_POS_VALUE
        )
        print(current_pos, current_theta)

        clock.tick(FPS)

    # Joystick操作モード
    else:
        # パラメータ初期化
        time = 0.0
        lspb_start_theta, lspb_goal_theta = None, None
        lspb_start_pos, lspb_goal_pos = None, None
        lspb_final_goal_pos, lspb_final_goal_theta = None, None
        workspace_trajection, two_steps_trajection = None, None

        # 速度（スティック傾き→指令値の変換倍率）の設定
        mapping_ratio = (joystick.get_axis(AXIS_SPEED_ADJUST) + 1) * 5 + 3
        axis_values = np.array([
            -joystick.get_axis(axis_num) * mapping_ratio for axis_num in (
                AXIS_X, AXIS_Y,
            )
        ])
        goal_pos = current_pos + axis_values

        try:
            goal_theta = calf.solve_ik(
                goal_pos,
                L1, L2, current_theta2_reverse,
                SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                THETA1_RANGE, THETA2_RANGE,
            )
        except ValueError:
            clock.tick(FPS)
            continue

        current_pos = goal_pos
        current_theta = goal_theta
        dxlf.rotate_joint_dxls(
            joint_dxls, goal_theta, DXL_MAX_POS_VALUE
        )

        # 手先の上げ下げ
        if joystick.get_button(BUTTON_HANDS_UP) and not hand_up:
            hand_up = True
            current_pos, L2 = dxlf.hand_updown(
                hand_dxl, hand_up,
                THETA_HAND_UP, THETA_HAND_DOWN, DXL_MAX_POS_VALUE,
                HOME_THETA, L1, L2_HAND_UP, L2_HAND_DOWN,
                SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                THETA1_RANGE, THETA2_RANGE
            )
        if joystick.get_button(BUTTON_HANDS_DOWN) and hand_up:
            hand_up = False
            current_pos, L2 = dxlf.hand_updown(
                hand_dxl, hand_up,
                THETA_HAND_UP, THETA_HAND_DOWN, DXL_MAX_POS_VALUE,
                HOME_THETA, L1, L2_HAND_UP, L2_HAND_DOWN,
                SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                THETA1_RANGE, THETA2_RANGE
            )
            
        print(current_pos, current_theta)

        clock.tick(FPS)


        


