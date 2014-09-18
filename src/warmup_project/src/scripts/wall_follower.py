#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def getch():
  """ Return the next character typed on the keyboard """
  import sys, tty, termios
  fd = sys.stdin.fileno()
  old_settings = termios.tcgetattr(fd)
  try:
    tty.setraw(sys.stdin.fileno())
    ch = sys.stdin.read(1)
  finally:
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
  return ch


def getLoc():
  pass


def followWall(pub):
  theta, distance = getLoc()
  des_theta = 0
  des_distance = 1
  pd = .1
  pt = .5
  w = ((distance - des_distance) * pd) * (theta - des_theta) * pt
  v = .1 
  pub.publish(Twist(angular=Vector3(z=w), linear=Vector3(x=v))


def run():
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
  rospy.init_node('control_neato', anonymous=True)
  r = rospy.Rate(10) # 10hz
  start = False
  while not rospy.is_shutdown():
    followWall()
    #if move(pub):
    #  break
 
if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException: pass
