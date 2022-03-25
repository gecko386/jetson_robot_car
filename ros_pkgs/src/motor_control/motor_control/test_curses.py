#!/usr/bin/env python3

#curses package
import curses
from curses.ascii import ESC

#math packages
import math

class ControlGUI(object):

    #LABELS
    MAIN_WINDOW = 'Robot Control Interface'
    COMMAND_AREA = 'Commands'
    STATUS_AREA = 'Status'
    LOG_AREA = 'Console log'

    #events:
    EXIT = 0
    KEY_UP = 1
    KEY_DOWN = 2
    KEY_LEFT = 3
    KEY_RIGHT = 4


    def __init__(self):
        self._screen = curses.initscr()
        
        self._screen.clear()
        #screen.nodelay(True)
        curses.noecho()
        curses.cbreak()
        curses.curs_set(0)
        self._win = None
        self._commands = None
        self._status = None
        self._log = None

        self.update_window()

    def __del__(self):
        curses.nocbreak()
        self._screen.keypad(False)
        curses.echo()
        curses.endwin()


    def _update_window(self):
        y, x = self._screen.getmaxyx()
        commands_x = 1
        commands_y = 1
        commands_w = int(x*2/3)-1
        commands_h = int(y*2/3)

        status_x = commands_w+1
        status_y = 1
        status_w = int(x*1/3)-1
        status_h = int(y*2/3)

        log_x = 1
        log_y = commands_h+1
        log_w = x-2
        log_h = int(y*1/3)-1


        self._win = curses.newwin(y, x, 0, 0);
        self._win.box()
        self._win.move(0, (x-len(ControlGUI.MAIN_WINDOW))//2)
        self._win.addstr(ControlGUI.MAIN_WINDOW)
        self._win.nodelay(True)
                
        self._commands = self._win.subwin(commands_h, commands_w, commands_y, commands_x);
        self._commands.box()
        self._commands.move(0, 1)
        self._commands.addstr(ControlGUI.COMMAND_AREA)
        
        self._commands.move(1, 1)
        self._commands.addstr(str('hello'))
        self._commands.refresh()

        self._status = self._win.subwin(status_h, status_w, status_y, status_x);
        self._status.box()
        self._status.move(0, 1)
        self._status.addstr(ControlGUI.STATUS_AREA)
        self._status.refresh()

        self._log = self._win.subwin(log_h, log_w, log_y, log_x);
        self._log.box()
        self._log.move(0, 1)
        self._log.addstr(ControlGUI.LOG_AREA)
        self._log.refresh()

        self._win.refresh()


    def get_event(self):
        evt = self._win.getch()
        if evt == ord('q'):
            return ControlGUI.EXIT
        return None


"""
def main_window(x, y, c):

    commands_x = 1
    commands_y = 1
    commands_w = int(x*2/3)-1
    commands_h = int(y*2/3)

    status_x = commands_w+1
    status_y = 1
    status_w = int(x*1/3)-1
    status_h = int(y*2/3)

    log_x = 1
    log_y = commands_h+1
    log_w = x-2
    log_h = int(y*1/3)-1



    win = curses.newwin(y, x, 0, 0);
    win.box()
    win.move(0, (x-len(MAIN_WINDOW))//2)
    win.addstr(MAIN_WINDOW)
    #win.nodelay(True)
    
    
    commands = win.subwin(commands_h, commands_w, commands_y, commands_x);
    commands.box()
    commands.move(0, 1)
    commands.addstr(COMMAND_AREA)
    

    commands.move(1, 1)
    commands.addstr(str(c))
    commands.refresh()

    status = win.subwin(status_h, status_w, status_y, status_x);
    status.box()
    status.move(0, 1)
    status.addstr(STATUS_AREA)
    status.refresh()

    log = win.subwin(log_h, log_w, log_y, log_x);
    log.box()
    log.move(0, 1)
    log.addstr(LOG_AREA)
    log.refresh()

    win.refresh()

    return win

def paint(x, y):
    win = curses.newwin(8, 20, 20, 20);

    win.box()
    win.move(0, 1)
    win.addstr('title')
    win.move(1, 1)
    win.addstr(str(x))
    win.move(2, 1)
    win.addstr(str(y))
    win.refresh()
    return win
"""


def main():
    gui = ControlGUI()

    while True:
        if gui.get_event() == ControlGUI.EXIT:
            break

if __name__ == "__main__":
    main()