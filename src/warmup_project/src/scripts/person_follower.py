#!/usr/bin/env python

from follower import Follower

import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


class PersonFollower(Follower):
  """Follows a person around."""
  
  def __init__(self, des_dist=1):
    Follower.__init__(self)
    self.des_dist = des_dist

  def get_location(self, msg):
    """Gets the distance and angle from an object. 
    Sets:
      theta - the angle where the object is
      distance - the average distance from the object
    """
    self.msg = msg
    valid_ranges = []
    angles = []
    r = 50

    # look from angle -25 to 25 to see if there is an object.
    for i in range(r):
      new_angle = i - r/2 
      reading = self.msg.ranges[new_angle % 359]
      if reading > 0 and reading < 3:
         valid_ranges.append(reading)
         angles.append(new_angle) 

    # get center angle and average distance of object
    self.theta = self.avg(angles)
    self.distance = self.avg(valid_ranges)

  def follow(self):
    """Follows based on the distance and angle from the center of the object.
    Returns:
       The linear velocity and the angular velocity
    """
    if self.theta:
      w = self.theta/50
      v = (self.distance - self.des_dist) * self.v
      self.object_exists = True
      return v, w
    else:
      self.object_exists = False 
      return None, None
      
        
if __name__ == '__main__':
  try:
    follower = PersonFollower()
    follower.run()
  except rospy.ROSInterruptException: pass
