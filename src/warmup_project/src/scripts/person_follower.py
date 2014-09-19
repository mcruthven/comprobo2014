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

class PersonFollower(object):
  """Follows a person around."""
  
  def __init__(self, des_dist=1):
    self.theta = -1
    self.des_dist = des_dist
    self.distance = -1
    self.msg = -1

  def get_location(self, msg):
    """Gets the distance and angle from an object. 
    Sets:
      theta - the angle where the object is
      distance - the average distance from the object
    """
    self.msg = msg
    valid_ranges = []
    r = 50
    angles = []
    angle = 0

    # look from angle -25 to 25 to see if there is an object.
    for i in range(r):
      new_angle = angle - r/2 + i
      reading = self.msg.ranges[new_angle % 359]
      if reading > 0 and reading < 8:
         valid_ranges.append(reading)
         angles.append(new_angle) 

    # get center angle and average distance of object
    if len(valid_ranges) > 0:
      self.theta = sum(angles)/1.0/len(angles)
      self.distance = sum(valid_ranges)/1.0/len(valid_ranges) 
    else:
      self.theta = -1
      self.distance = -1
  
  def follow(self):
    # If an object is sensed in front of the neato, follow it.
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, self.get_location)
    rospy.init_node('control_neato', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      if self.theta != -1.0:
        w = self.theta/50
        v = (self.des_dist - self.distance) * -.3
        self.pub.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=w)))
      r.sleep()
      
        
if __name__ == '__main__':
  try:
    follower = PersonFollower()
    follower.follow()
  except rospy.ROSInterruptException: pass
