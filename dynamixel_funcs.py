import sys
import math
import time

import numpy as np

from dynamixel_sdk import *

import calculate_funcs as calf


class Dynamixel():
    def __init__(
            self,
            id,
            port_handler,
            packet_handler,
            addr_torque_enable=64,
            addr_goal_position=116,
            addr_present_position=132,
            addr_plofile_velocity=112,
            addr_plofile_acceleration=108,
            torque_enable=1,
            torque_disable=0
    ):
        self.id = id
        self.port_handler = port_handler
        self.packet_handler = packet_handler
        self.addr_torque_enable = addr_torque_enable
        self.addr_goal_position = addr_goal_position
        self.addr_present_position = addr_present_position
        self.addr_plofile_velocity = addr_plofile_velocity
        self.addr_plofile_acceleration = addr_plofile_acceleration
        self.torque_enable = torque_enable
        self.torque_disable = torque_disable

    def torque_on(self):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id,
            self.addr_torque_enable, self.torque_enable,
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f'Error! ID{self.id}のトルクをONにできませんでした')
            raise RuntimeError

    def torque_off(self):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.id,
            self.addr_torque_enable, self.torque_disable,
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f'Error! ID{self.id}のトルクをOFFにできませんでした')
            sys.exit()

    def set_profile(self, vel_profile, acc_profile):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id,
            self.addr_plofile_velocity, vel_profile,
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f'Error! ID{self.id}のProfile Velocityの書き込みに失敗しました')
            sys.exit()

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id,
            self.addr_plofile_acceleration, acc_profile,
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f'Error! ID{self.id}のProfile Accelerationの書き込みに失敗しました')
            sys.exit()

    def rotate(self, position):
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.id,
            self.addr_goal_position, position,
        )
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packet_handler.getRxPacketError(dxl_error))

    def read_position(self):
        position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.id, self.addr_present_position,
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f'Error! ID{self.id}のPresent Positionを読み取れませんでした')
            sys.exit()
        
        return position


def dxl_connection_setup(
        device_name,
        baudrate=115200,
        protocol_version=2.0,
):
    port_handler = PortHandler(device_name)
    packet_handler = PacketHandler(protocol_version)

    if not port_handler.openPort():
        print('Error! ポートを開けませんでした')
        sys.exit()
    
    if not port_handler.setBaudRate(baudrate):
        print('Error! ボーレートが間違っています')
        sys.exit()

    print('Dynamixelとの接続が完了しました')

    return port_handler, packet_handler


def dynamixels_setup(
        device_name, baudrate, protocol_version,
        vel_profile, acc_profile,
):
    port_handler, packet_handler = dxl_connection_setup(
        device_name, baudrate, protocol_version
    )

    joint_dxls = [
        Dynamixel(
            id, port_handler, packet_handler
        ) for id in (1, 2)
    ]
    hand_dxl = Dynamixel(3, port_handler, packet_handler)
    dxls = joint_dxls + [hand_dxl]
    #dxls = [hand_dxl]

    for dxl in dxls:
        i = 1
        while True:
            try:
                print(f'try: {i}')
                dxl.torque_on()
            except RuntimeError:
                i += 1
                time.sleep(0.1)
                continue
            else:
                break
        dxl.set_profile(vel_profile, acc_profile)

    return joint_dxls, hand_dxl


def calc_dxl_pos_from_theta(theta, dxl_max_pos):
    dxl_pos = (theta * dxl_max_pos / (2 * np.pi)).astype(int)
    dxl_pos += dxl_max_pos // 2
    return dxl_pos


def calc_theta_from_dxl_pos(dxl_pos, dxl_max_pos):
    dxl_pos -= dxl_max_pos // 2
    theta = dxl_pos * 2 * np.pi / dxl_max_pos
    return theta


def rotate_joint_dxls(joint_dxls, goal_theta, dxl_max_pos):
    goal_pos = calc_dxl_pos_from_theta(goal_theta, dxl_max_pos)
    for dxl, pos in zip(joint_dxls, goal_pos):
        dxl.rotate(pos)


def hand_updown(
        hand_dxl, hand_up,
        dxl_pos_hand_up, dxl_pos_hand_down,
):
    dxl_pos = dxl_pos_hand_up if hand_up else dxl_pos_hand_down
    hand_dxl.rotate(dxl_pos)


def hand_updown_(
        hand_dxl, hand_up,
        theta_hand_up, theta_hand_down, dxl_max_pos,
        current_theta, l1, l2_hand_up, l2_hand_down,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
):
    l2 = l2_hand_up if hand_up else l2_hand_down
    position = calf.solve_fk(
        current_theta, l1, l2,
        shere_area_x, robot_area_point, team_color,
        theta1_range, theta2_range
    )

    theta_hand = theta_hand_up if hand_up else theta_hand_down
    dxl_pos_hand = calc_dxl_pos_from_theta(
        np.array(theta_hand), dxl_max_pos
    )
    hand_dxl.rotate(dxl_pos_hand)

    return position, l2
