#!/usr/bin/env python

import sys

from niryo_robot_rqt_gui.rqt_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_mypkg'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))