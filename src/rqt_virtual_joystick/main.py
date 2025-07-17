#!/usr/bin/env python3

import sys
from rqt_gui.main import Main


def main():
    """Launch VirtualJoystick as a standalone RQt application."""
    main_app = Main()
    sys.exit(main_app.main(sys.argv, standalone='rqt_virtual_joystick.virtual_joystick.VirtualJoystick'))


if __name__ == '__main__':
    main()
