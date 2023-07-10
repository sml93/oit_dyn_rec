#!/usr/bin/env python3
import time
import rospy

from argparse import Namespace
from oit_dyn_rec.cfg import ParamsConfig
from dynamic_reconfigure.server import Server
from std_msgs.msg import Int16, Float32, Bool
from mavros_msgs.msg import AttitudeTarget

def callback(config, level):
  #pub_angle = rospy.Publisher("jetForce_angle", Int16, queue_size=10)
  pub_mag = rospy.Publisher("jetForce_mag", Float32, queue_size=10)
  pub_mission = rospy.Publisher("mission_status", Bool, queue_size=10)
  pub_orientation = rospy.Publisher("desired_atti", AttitudeTarget, queue_size=10)
  pub_orientation_msg = AttitudeTarget()
  pub_mission_msg = Bool()
  rospy.loginfo("""Reconfigure Request: {e_land}, {jetForceMag}, {orientX}, {orientY}, {orientZ}, {orientThrust}""".format(**config))
  #rospy.loginfo("""Reconfigure Request: {jetForceAng}, {jetForceMag}, {mission_param}, {orientX}, {orientY}, {orientZ}, {orientThrust}""".format(**config))
  
  pub_orientation_msg.orientation.x = config.get('orientX')
  pub_orientation_msg.orientation.y = config.get('orientY')
  pub_orientation_msg.orientation.z = config.get('orientZ')
  pub_orientation_msg.orientation.w = 1.0
  pub_orientation_msg.thrust = config.get('orientThrust')
  pub_mission_msg = config.get('e_land')
  pub_mag.publish(config.get('jetForceMag'))
  #pub_angle.publish(config.get('jetForceAngle'))
  pub_orientation.publish(pub_orientation_msg)
  pub_mission.publish(pub_mission_msg)
  return config
  
  
if __name__ == "__main__":
  rospy.init_node("oit_dyn_rec", anonymous=True)
  srv = Server(ParamsConfig, callback)
  rospy.spin()
