import os
import sys

import pygame as pg


os.environ["SDL_VIDEODRIVER"] = "dummy"


def init_ps4_controler():
    pg.init()
    pg.joystick.init()
    if pg.joystick.get_count() == 0:
        print('コントローラが接続されていません')
        sys.exit()

    joystick = pg.joystick.Joystick(0)
    joystick.init()

    return joystick


def wait_until_psbutton_pressed(joystick):
    clock = pg.time.Clock()
    while True:
        pg.event.pump()
        if joystick.get_button(10):
            break
        clock.tick(60)

    return