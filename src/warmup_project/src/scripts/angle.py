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

ran = 330
def get_avg_theta(msg):
  angles = [x * 10 + ran for x in range(15)]
  valid_angles = []
  global theta
  for angle in angles: 
    offset = get_theta(msg, angle)
    if offset != -1:
      valid_angles.append(offset)
  if len(valid_angles) > 0:
    theta = sum(valid_angles) / len(valid_angles)
  else: 
    theta = -1


theta = -1
def get_theta(msg, angle_1):
  angle_2 = angle_1 + 90
  d1 = get_avg_distance(msg, angle_1)
  d2 = get_avg_distance(msg, angle_2)
  if d1 != -1 and d2 != -1:
    theta = angle_1 - math.atan(d2/d1)*180/math.pi
    return theta - 360 if theta > 180 else theta
  return -1


def get_avg_distance(msg, angle):
  valid_ranges = []
  for i in range(3):
    new_angle = angle - 1 + i
    reading = msg.ranges[new_angle % 359]
    if reading > 0 and reading < 4:
         valid_ranges.append(reading)
  if len(valid_ranges) > 0:
    return sum(valid_ranges)/1.0/len(valid_ranges) 
  return -1


distance, front_distance = -1, -1
def scan_received(msg, pub):
  get_avg_theta(msg)
  global distance, front_distance
  front_distance = get_avg_distance(msg, 360)
  distance = get_avg_distance(msg, 90 - (int)(theta))

def follow(m, pub):
  s = m * .3
  #if front_distance > 0:
  #  #s = s * (front_distance - distance)
  #  #global ran
  #  #ran = 330 if s < .1 else 30
  des_dist = 1
  if theta != -1.0:
     a = theta/50
     print distance, s, a 
     pub.publish(Twist(linear=Vector3(x=s), angular=Vector3(z=a)))

def move(pub):
  c = getch()
  if c == 'j':
    pub.publish(Twist(angular=Vector3(z=.4)))
  elif c == 'm':
    pub.publish(Twist(linear=Vector3(x=-.3)))
  elif c == 'i':
    pub.publish(Twist(linear=Vector3(x=.3)))
  elif c == 'p':
    print theta
  elif c == 'l':
    pub.publish(Twist(angular=Vector3(z=-.4)))
  elif c == 'f':
    follow(-1, pub)
  elif c == 'd':
    follow(1, pub)
  elif c == 's':
    return True
  elif c == 'w':
    pub.publish(Twist(angular=Vector3(z=0)))
    pub.publish(Twist(linear=Vector3(x=0)))
  return False


def run():
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
  rospy.init_node('control_neato', anonymous=True)
  r = rospy.Rate(10) # 10hz
  while not rospy.is_shutdown():
    if move(pub):
      break
      
        
if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException: pass
