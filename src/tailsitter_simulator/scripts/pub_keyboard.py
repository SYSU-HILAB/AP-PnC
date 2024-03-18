#!/usr/bin/env python
import rclpy
import time
from geometry_msgs.msg import Twist

import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Control Simulation Signal!
---------------------------
Swtich to change mode:
C : Change mode to Step mode from Autonomous mode or vice versa

For Step mode 
---------------------------
S : Forward one step

For Shutdown
---------------------------
D : Shutdown
"""


def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(speed,turn):
	return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):	
	if args is None:
		args = sys.argv



	try:
		while(1):
			key = getKey()
			print(f'key {key}')
	except:
		print()

	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
  main()