import os
import sys

import pygame as pg

from common_settings import *
import utils
import dynamixel_funcs as dxlf
import calculate_funcs as calf


os.environ["SDL_VIDEODRIVER"] = "dummy"

TEAM_COLOR = 'blue'
if TEAM_COLOR == 'blue':
    from blue_settings import *

joystick = utils.init_ps4_controler()
print('PS4コントローラの初期化が完了しました')

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

    if joystick.get_button(BUTTON_WORKSPACE_TRIGER):
        work_area_thetas = WORK_AREA_THETAS_HAND_UP if hand_up else WORK_AREA_THETAS_HAND_DOWN
        if joystick.get_button(BUTTON_PW1):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[0]
                lspb_goal_pos = WORK_AREA_POINTS[0]
        elif joystick.get_buttons(BUTTON_PW2):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[1]
                lspb_goal_pos = WORK_AREA_POINTS[1]
        elif joystick.get_buttons(BUTTON_PW3):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[2]
                lspb_goal_pos = WORK_AREA_POINTS[2]
        elif joystick.get_buttons(BUTTON_PW4):
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[3]
                lspb_goal_pos = WORK_AREA_POINTS[3]
        elif joystick.get_hat(0)[BUTTON_PW5[0]] == BUTTON_PW5[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[4]
                lspb_goal_pos = WORK_AREA_POINTS[4]
        elif joystick.get_hat(0)[BUTTON_PW6[0]] == BUTTON_PW6[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[5]
                lspb_goal_pos = WORK_AREA_POINTS[5]
        elif joystick.get_hat(0)[BUTTON_PW7[0]] == BUTTON_PW7[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[6]
                lspb_goal_pos = WORK_AREA_POINTS[6]
        elif joystick.get_hat(0)[BUTTON_PW8[0]] == BUTTON_PW8[1]:
            if lspb_goal_theta is None:
                lspb_goal_theta = work_area_thetas[7]
                lspb_goal_pos = WORK_AREA_POINTS[7]
        else:
            clock.tick(FPS)
            continue
        
        if lspb_start_theta is None:
            lspb_start_theta = current_theta

        # どの軌跡を用いるかを決定する
        if (lspb_goal_pos is not None) and (workspace_trajection is None):
            workspace_trajection, two_steps_trajection = calf.decide_trajection_type(
                lspb_start_pos, lspb_goal_pos,
                TRAJECTION_TYPE_SEPARATE_POINT, TEAM_COLOR
            )

        # ２段階軌跡を用いる場合、目的座標を軌跡切り替わり点の座標に変更する
        if two_steps_trajection and (lspb_final_goal_pos is None):
            lspb_final_goal_pos = lspb_goal_pos
            lspb_final_goal_theta = lspb_goal_theta
            lspb_goal_pos = TRAJECTION_CHANGE_POINT
            lspb_goal_theta = TRAJECTION_CHANGE_THETA_HAND_UP if hand_up else TRAJECTION_CHANGE_THETA_HAND_DOWN

        if workspace_trajection:
            try:
                goal_theta, goal_pos = calf.workspace_lspb(
                    lspb_start_pos, lspb_goal_pos, time,
                    ACC_TIME, MOVE_TIME, MAX_VEL, MAX_ACC,
                    L1, L2, WORK_AREA_THETA2_REVERSE,
                    SHERE_AREA_X, ROBOT_AREA_POINT, TEAM_COLOR,
                    THETA1_RANGE, THETA2_RANGE,
                )
            except ValueError:
                clock.tick(FPS)
                continue
            except RuntimeError:
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

        


