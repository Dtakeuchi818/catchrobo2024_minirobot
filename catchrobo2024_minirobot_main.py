import os
import sys

import numpy as np
import pygame as pg
import serial

from common_settings import *
import utils
import dynamixel_funcs as dxlf
import calculate_funcs as calf


os.environ["SDL_VIDEODRIVER"] = "dummy"


# ====== コマンドライン引数の処理
argv = sys.argv
if (len(argv) != 2) or (argv[1] not in ('red', 'blue')):
    print('コマンドライン引数でチーム名(red or blue)を渡してください')
TEAM_COLOR = argv[1]
# パラメータ設定
if TEAM_COLOR == 'blue':
    from blue_settings import *


# ====== PS4コントローラーの初期化
joystick = utils.init_ps4_controler()
print('PS4コントローラの初期化が完了しました')


# ====== ラズパイピコとの接続
clock = pg.time.Clock()
print('-' * 8)
print('PSボタンを押すとラズパイピコとのシリアル通信を初期化します')
print('スキップする場合はOPTIONボタンを押してください')
while True:
    pg.event.pump()
    if joystick.get_button(10):
        ser = serial.Serial(RASPICO_DEVNAME, RASPICO_BAUDRATE)
        print('ラズパイピコとのシリアル通信を初期化しました')
        break
    if joystick.get_button(9):
        ser = None
        print('ラズパイピコとの接続をスキップしました')
        break
    clock.tick(60)


# =========== Dynamixel初期化・ロボットの初期化
print('-' * 8)
print(f'現在、{TEAM_COLOR} モードで起動しています')
print('PSボタンを押すとロボットが起動します')
print('ホームポジションへの移動を行うので、周囲に注意してください')
utils.wait_until_psbutton_pressed(joystick)
# Dynamixel初期化
joint_dxls, hand_dxl = dxlf.dynamixels_setup(
    DEVICENAME, BAUDRATE, PROTOCOL_VERSION,
    DXL_PLOFILE_VELOCITY, DXL_PLOFILE_ACCELERATION,
)
# 手先をあげる
hand_up = True
dxlf.hand_updown(
    hand_dxl, hand_up,
    DXL_POS_HAND_UP, DXL_POS_HAND_DOWN,
)
# 速度・加速度プロファイル設定
for dxl in joint_dxls:
    dxl.set_profile(
        MOVE_TO_HOME_PLOFILE_VEL, MOVE_TO_HOME_PLOFILE_ACC
    )
# ホームポジションへ移動
dxlf.rotate_joint_dxls(
    joint_dxls, HOME_THETA, DXL_MAX_POS_VALUE,
)
# 速度・加速度プロフファイルを戻す
for dxl in joint_dxls:
    dxl.set_profile(
        DXL_PLOFILE_VELOCITY, DXL_PLOFILE_ACCELERATION,
    )
# 現在位置の初期化
current_theta = HOME_THETA
current_pos = HOME_POS
print('ロボットの起動が完了しました')


# =========== ロボット操作開始
print('-' * 8)
print('PSボタンを押すとロボットの操作を開始します')
#utils.wait_until_psbutton_pressed(joystick)

# クロック初期化
clock = pg.time.Clock()

# 定点位置モード関連パラメータ初期化
time = 0.0
lspb_start_theta, lspb_goal_theta = None, None
lspb_start_pos, lspb_goal_pos = None, None
lspb_final_goal_pos, lspb_final_goal_theta = None, None
workspace_trajection, two_steps_trajection = None, None

# メインループ
while True:
    pg.event.pump()

    current_theta2_reverse = current_theta[1] < 0

    # ------ 定点移動モード ------
    if joystick.get_button(BUTTON_WORKSPACE_TRIGER) or joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER):
        # シューティングエリア泳動する場合、手先を上げる
        if joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and not hand_up:
            hand_up = True
            dxlf.hand_updown(
                hand_dxl, hand_up,
                DXL_POS_HAND_UP, DXL_POS_HAND_DOWN,
            )

        # ワークスペース上の定点を選択する
        if joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P1):
            if lspb_goal_pos is None and lspb_start_pos is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[0]
                lspb_goal_pos = WORK_AREA_POINTS[0]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P2):
            if lspb_goal_pos is None and lspb_start_pos is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[1]
                lspb_goal_pos = WORK_AREA_POINTS[1]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P3):
            if lspb_goal_pos is None and lspb_start_pos is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[2]
                lspb_goal_pos = WORK_AREA_POINTS[2]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_button(BUTTON_P4):
            if lspb_goal_pos is None and lspb_start_pos is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[3]
                lspb_goal_pos = WORK_AREA_POINTS[3]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P5[0]] == BUTTON_P5[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[4]
                lspb_goal_pos = WORK_AREA_POINTS[4]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P6[0]] == BUTTON_P6[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[5]
                lspb_goal_pos = WORK_AREA_POINTS[5]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P7[0]] == BUTTON_P7[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[6]
                lspb_goal_pos = WORK_AREA_POINTS[6]
        elif joystick.get_button(BUTTON_WORKSPACE_TRIGER) and joystick.get_hat(0)[BUTTON_P8[0]] == BUTTON_P8[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = WORK_AREA_THETAS_HAND_DOWN[7]
                lspb_goal_pos = WORK_AREA_POINTS[7]

        # シューティングエリア上の定点を選択する
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P1):
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[0]
                lspb_goal_pos = SHOOTING_AREA_POINTS[0]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P2):
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[1]
                lspb_goal_pos = SHOOTING_AREA_POINTS[1]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P3):
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[2]
                lspb_goal_pos = SHOOTING_AREA_POINTS[2]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_button(BUTTON_P4):
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[3]
                lspb_goal_pos = SHOOTING_AREA_POINTS[3]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_hat(0)[BUTTON_P5[0]] == BUTTON_P5[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[4]
                lspb_goal_pos = SHOOTING_AREA_POINTS[4]
        elif joystick.get_button(BUTTON_SHOOTINGAREA_TRIGER) and joystick.get_hat(0)[BUTTON_P6[0]] == BUTTON_P6[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = SHOOTING_AREA_THETAS_HAND_UP[5]
                lspb_goal_pos = SHOOTING_AREA_POINTS[5]

        # TRIGER_BUTTON以外に何も押されていない場合、何もしない
        else:
            clock.tick(FPS)
            continue


        # 現在の関節角度と位置をlspbのスタート位置として保存する
        if lspb_start_theta is None and lspb_start_pos is None:
            lspb_start_theta = current_theta
            lspb_start_pos = current_pos

        # どの軌跡を用いるかを決定する
        if (lspb_goal_pos is not None) and (workspace_trajection is None):
            workspace_trajection, two_steps_trajection = calf.decide_trajection_type(
                lspb_start_pos, lspb_goal_pos,
                TRAJECTION_TYPE_SEPARATE_POINT, TEAM_COLOR
            )

       # print(workspace_trajection, two_steps_trajection)

        # ２段階軌跡を用いる場合、目的座標を軌跡切り替わり点の座標に変更する
        # 同時に、本来の目標座標を保存しておく
        if two_steps_trajection and (lspb_final_goal_pos is None):
            lspb_final_goal_pos = lspb_goal_pos
            lspb_final_goal_theta = lspb_goal_theta
            lspb_goal_pos = TRAJECTION_CHANGE_POINT
            lspb_goal_theta = TRAJECTION_CHANGE_THETA_HAND_DOWN

        # 作業空間上で軌跡を計算する
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

            # 目標座標に達した場合
            except RuntimeError:
                # 2段階軌跡を用いる場合、2段階目の軌跡に移行する
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

        # 関節角度空間上の直線軌跡を計算する
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

        # 関節回転の実行
        dxlf.rotate_joint_dxls(
            joint_dxls, goal_theta, DXL_MAX_POS_VALUE
        )

        # 現在位置の更新
        current_theta = goal_theta
        current_pos = goal_pos

        #print(current_pos, np.degrees(current_theta))
    

    # ------ Joystick操作モード ------
    else:
        # 定点位置モード関連パラメータ初期化
        time = 0.0
        lspb_start_theta, lspb_goal_theta = None, None
        lspb_start_pos, lspb_goal_pos = None, None
        lspb_final_goal_pos, lspb_final_goal_theta = None, None
        workspace_trajection, two_steps_trajection = None, None

        # ボタンの押し具合から移動速度（変換倍率）を設定
        mapping_ratio = (joystick.get_axis(AXIS_SPEED_ADJUST) + 1) * 5 + 3
        
        # Joystickの傾きを取得
        axis_values = []
        for axis_num in (AXIS_X, AXIS_Y):
            axis_value = -joystick.get_axis(axis_num)
            axis_value = axis_value if abs(axis_value) > JOYSTICK_DEDZONE_THRESH else 0.0
            axis_value *= mapping_ratio
            axis_values.append(axis_value)
        axis_values = np.array(axis_values)

        # 目標手先位置の計算
        goal_pos = current_pos + axis_values

        # 逆運運動学計算（関節角度算出）
        try:
            goal_theta = calf.solve_ik(
                goal_pos,
                L1, L2, current_theta2_reverse,
                SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                THETA1_RANGE, THETA2_RANGE,
            )
        # 不適切な関節角度になった場合無視する
        except ValueError:
            clock.tick(FPS)
            continue

        # 関節回転の実行
        dxlf.rotate_joint_dxls(
            joint_dxls, goal_theta, DXL_MAX_POS_VALUE,
        )

        # 現在位置の更新
        current_pos = goal_pos
        current_theta = goal_theta

    # 手先を上げる
    if joystick.get_button(BUTTON_HANDS_UP) and not hand_up:
        hand_up = True
        dxlf.hand_updown(
            hand_dxl, hand_up,
            DXL_POS_HAND_UP, DXL_POS_HAND_DOWN,
        )

    # 手先を下げる
    if joystick.get_button(BUTTON_HANDS_DOWN) and hand_up:
        hand_up = False
        dxlf.hand_updown(
            hand_dxl, hand_up,
            DXL_POS_HAND_UP, DXL_POS_HAND_DOWN
        )

    # 吸引を始める
    if joystick.get_button(BUTTON_VACUME_START) and ser is not None:
        ser.write('1'.encode())

    # 吸引を止める
    if joystick.get_button(BUTTON_VACUME_END) and ser is not None:
        ser.write('0'.encode())

    # present_dxlpos = np.array([
    #     dxl.read_position() for dxl in joint_dxls
    # ])
    # present_dxlpos = dxlf.calc_theta_from_dxl_pos(
    #     present_dxlpos, DXL_MAX_POS_VALUE,
    # )
    # present_pos = calf.solve_fk(
    #     present_dxlpos,
    #     L1, L2,
    #     SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
    #     THETA1_RANGE, THETA2_RANGE,

    # )

    #goal_pos = dxlf.calc_dxl_pos_from_theta(goal_theta, DXL_MAX_POS_VALUE)

    pos_ = calf.solve_fk(
        goal_theta,
        L1, L2,
        SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
        THETA1_RANGE, THETA2_RANGE,
    )
    print(current_pos, np.degrees(current_theta), np.degrees(goal_theta), pos_)
    #print(lspb_goal_theta)
    #print(current_theta2_reverse)
    clock.tick(FPS)




