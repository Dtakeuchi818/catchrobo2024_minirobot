import utils
import pygame

joystick = utils.init_ps4_controler()
print('PS4コントローラの初期化が完了しました')

clock = pygame.time.Clock()
while True:
    pygame.event.pump()

    axis = [
        joystick.get_axis(num) for num in range(5)
    ]
    print(axis)
    clock.tick(10)