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



def follow_wall():
  pass


def get_distance(msg, angle):
  assert angle < 360
  valid_ranges = []
  for i in range(5):
    if msg.ranges[angle + i] > 0 and msg.ranges[angle + i] < 8:
       valid_ranges.append(msg.ranges[angle + i])
  if len(valid_ranges) > 0:
    mean_distance = sum(valid_ranges)/float(len(valid_ranges))
  else:
    mean_distance = -1.0
  return mean_distance


def find_angles(angle_a, b, c):
  a = (c**2 + b**2 - 2*c*b*math.cos(angle_a/180.0*math.pi))**.5
  angle_b = math.asin(math.sin(angle_a/180.0*math.pi)/a*b)/math.pi*180.0
  angle_c = 180 - angle_b - angle_a
  return angle_b, angle_c


mean_distance = -1
def scan_received(msg, pub):
  """ Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan """
  global mean_distance
  mean_distance = get_distance(msg, 0)
  angle_1, angle_2 = 30, 45
  side_1 = get_distance(msg, angle_1)
  side_2 = get_distance(msg, angle_2)
  if side_1 != -1.0 and side_2 != -1.0:
    if side_1 > side_2:
      print find_angles(15, side_1, side_2)
    else:
      print find_angles(15, side_2, side_1)
    

def move(pub):
  c = getch()
  if c == 'h':
    pub.publish(Twist(angular=Vector3(z=.4)))
  elif c == 'j':
    pub.publish(Twist(linear=Vector3(x=-.4)))
  elif c == 'k':
    pub.publish(Twist(linear=Vector3(x=.4)))
  elif c == 'l':
    pub.publish(Twist(angular=Vector3(z=-.4)))
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
  start = False
  while not rospy.is_shutdown():
    #if start and mean_distance != -1.0:
    #  #vel = Twist(Vector3(0.2*(mean_distance - 1.0), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    #  vel = Twist(Vector3(.1, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
    #  pub.publish(vel)
    if move(pub):
      break
    #r.sleep()
      
        
if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException: pass
