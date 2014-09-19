#!/usr/bin/env python

from wall_follower import WallFollower
from person_follower import PersonFollower

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan


def run():
  """Follows the wall using sensor data."""
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  rospy.init_node('control_neato', anonymous=True)
  r = rospy.Rate(10) # 10hz

  wall_follower = WallFollower()
  person_follower = PersonFollower()
  follower = wall_follower

  # Subscribe the followers to LaserScan
  sub = rospy.Subscriber('scan', LaserScan, follower.get_location)
  sub2 = rospy.Subscriber('scan', LaserScan, person_follower.get_location)

  while not rospy.is_shutdown():
    v, a = follower.follow()
    if a and v:
      pub.publish(Twist(linear=Vector3(x=v), angular=Vector3(z=a)))
    # If the neato sees an object thats not a wall. Change to person follower
    if follower.object_found():
      follower = person_follower
    else:
      follower = wall_follower
    r.sleep()

if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException: pass
