import os

os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
from pygame import joystick, event
from pygame import QUIT, JOYAXISMOTION, JOYBALLMOTION, JOYHATMOTION, JOYBUTTONUP, JOYBUTTONDOWN, KEYDOWN, KEYUP, K_UP, K_DOWN, K_LEFT, K_RIGHT
import time


def get_events():

    #time.sleep(1.0 / 20.0)
    for e in pygame.event.get():
        if e.type == JOYAXISMOTION:
            print("axis event: "+"axis: " + str(e.axis) + " value: "+str(e.value))
            
        elif e.type == JOYHATMOTION:
            print("hat event: " + "hat: "+str(e.hat) + " hat X: "+str(e.value[0]) + " hat Y: "+str(e.value[0]))
            
        elif e.type == JOYBUTTONUP:
            print("joy up event: " + "joy button: " + str(e.button))
            
        elif e.type == JOYBUTTONDOWN:
            print("joy down event: " + "joy button: " + str(e.button))

        elif e.type == KEYUP:
            print("key up event: " + "key up: " + str(e.key))
            
        elif e.type == KEYDOWN:
            print("key down event: " + "key down: " + str(e.key))

    
    
def main():

    pygame.init()
    pygame.display.init()
    pygame.display.set_mode((1, 1))
    joystick.init()
    pygame.key.set_repeat(10,10)

    for i in range(joystick.get_count()):
        joy = joystick.Joystick(i)
        joy.init()
        if joy.get_init():
            break

    while True:
        get_events()


if __name__ == "__main__":
    main()
