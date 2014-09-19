#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


class Follower(object):
  """Follows something around."""

  def __init__(self):
    self.distance = None
    self.object_exists = False
    self.theta = None
    self.v = .3
    self.msg = -1
  
  def object_found(self):
    """Returns True if the follower senses a small object thats not a wall."""
    return self.object_exists

  @staticmethod
  def avg(elements):
    """Returns the average value of elements.
    Returns:
      The average or None if the list is empty
    """
    if len(elements) > 0:
      return sum(elements)/1.0/len(elements)
    return None

  def get_location(self, msg):
    """Subscribed to LaserScan and uses data to change theta or distance."""
  
  def follow(self):
    """Returns an angular and linear velocity based on sensor data."""
  
  def run(self):
    """Follows the wall using sensor data."""
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, self.get_location)
    rospy.init_node('control_neato', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      s, a = self.follow()
      pub.publish(Twist(linear=Vector3(x=s), angular=Vector3(z=a)))
      r.sleep()
