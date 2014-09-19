#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


class WallFollower(object):
  """Follows a wall if it is next to a wall."""

  def __init__(self):
    self.distance = -1
    self.theta = -1
    self.v = .3
    self.msg = -1
  
  def get_avg_distance(self, angle):
    """Returns the distance from the wall at angle."""
    valid_ranges = []
    for i in range(3):
      reading = self.msg.ranges[(angle - 1 + i) % 359]
      if reading > 0 and reading < 6:
         valid_ranges.append(reading)
    return self.avg(valid_ranges)

  def get_avg_theta(self):
    """Returns the average angle measured over all of angles."""
    start = False
    count = 0
    valid_angles = []
    angles = [x * 10 for x in range(36)]
    for angle in angles: 
      offset = self.get_theta(angle)
      if offset:
        start = True
        valid_angles.append(offset)
        count += 1
      elif start:
          # If it sees a small object remove it
          if count < 3:
            valid_angles = valid_angles[:len(valid_angles) - count]
          count = 0
          start = False
    return self.avg(valid_angles)

  def get_theta(self, angle_1):
    """Get what angle the robot is at in relation to the wall."""
    angle_2 = angle_1 + 90
    d1 = self.get_avg_distance(angle_1)
    d2 = self.get_avg_distance(angle_2)
    if d1 and d2:
      # Both of the measurements are 90 degrees apart
      # Use trig to find theta
      theta = angle_1 - math.atan(d2/d1)*180/math.pi
      return theta - 360 if theta > 180 else theta
    return None
  
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
    self.msg = msg
    self.theta = self.get_avg_theta()
    if self.theta:
      self.distance = self.get_avg_distance(89 - (int)(self.theta))
  
  def run(self, m=1):
    """Follows the wall using sensor data."""
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, self.get_location)
    rospy.init_node('control_neato', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
      if self.theta:
        s = m * self.v
        a = self.theta/50
        #print self.distance, s, a 
        self.pub.publish(Twist(linear=Vector3(x=s), angular=Vector3(z=a)))
      r.sleep()
  
        
if __name__ == '__main__':
  try:
    follower = WallFollower()
    follower.run()
  except rospy.ROSInterruptException: pass
