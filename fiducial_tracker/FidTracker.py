#!/usr/bin/python

'''
SBD Metallic Debris Collector Project
Stanley Black and Decker
Georgia Institute of Technology
created by Christian Brice

This code establishes an interrupt of sorts that only listens while the robot is
in the "HOME" state. This will only interrupt the standard odom-based "HOME" code
if a certain fiducial is detected (i.e. ID=108). In that case, this code submits
corrections to more accurately navigate back to the designated home location.

NOTE: this is only meant to be a supplement to the current return code. It was
      determined to be impractical to establish a full-blown fiducial-based
      odometry correction algorithm due to the amount of fiducials needed to
      cover the size of the project demonstration area.
'''

import rospy
from geometry_msgs.msg import TransformStamped, Twist
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import tf2_ros
from math import pi, sqrt, atan2
import traceback
import math
import time
from std_msgs.msg import String
#from state_machine import control


# Easy little rad->deg conversion function:
def degrees(r):
    return (180.0 * r) / math.pi

# [ CONSTRUCTOR ]
class FidTracker:
    def __init__(self):
       # Initialize node:
       rospy.init_node('fid_tracker')

       # Initialize transform listener/broadcaster:
       self.tfBuffer = tf2_ros.Buffer(rospy.Time(30))  #(cache 30s into past)
       self.lr = tf2_ros.TransformListener(self.tfBuffer)
       #self.br = tf2_ros.TransformBroadcaster()  #(only needed for rviz)

       # A publisher for robot motion commands
       self.cmdPub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

       # Coordinate frame of "home base" fiducial:
       self.target_fiducial = rospy.get_param("~target_fiducial", "fid108")

       # Robot's min/max distance from fiducial:
       self.min_dist = rospy.get_param("~min_dist", 0.5)
       self.max_dist = rospy.get_param("~max_dist", 100)

       # Subscribe to incoming transforms:
       rospy.Subscriber("/state_machine/control", String, self.stateTransform)
       rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.newTransform)
       
       # Initialize fiducial tracking vars: 
       self.fid_x = self.min_dist
       self.fid_y = 0
       self.got_fid = False
       self.suppressCmd = False
       self.max_lost_count = rospy.get_param("~max_lost_count", 400)

       # Angular movement vars:
       self.angular_rate = rospy.get_param("~angular_rate", 2.0)
       self.max_angular_rate = rospy.get_param("~max_angular_rate", 1.2)
       self.lost_angular_rate = rospy.get_param("~lost_angular_rate", 0.6)

       # Linear movement vars:
       self.linear_rate = rospy.get_param("~linear_rate", 1.2)
       self.max_linear_rate = rospy.get_param("~max_linear_rate", 1.5)
       self.linear_decay = rospy.get_param("~linear_decay", 0.9)


    # Called when a message is received from Tyler's controller:
    def stateTransform(self, msg):
    	# Initialize vars:
        can_run = False

        # Check message string:
        if msg == "return_home":
            can_run = True
        else:
            return

    # Called when a "FiducialTransformArray" message is received:
    def newTransform(self, msg):
    	# Initialize vars:
        imageTime = msg.header.stamp
        self.linSpeed = 0
        found = False

        # For every fiducial found by the dectector, publish a transform:
        # (shouldn't matter in our case, since we only have one fiducial)
        for m in msg.transforms:
            id = m.fiducial_id
            t = TransformStamped()
            t.child_frame_id = "fid%d" % id
            #t.header.frame_id = msg.header.frame_id
            #t.header.stamp = imageTime

            #trans = m.transform.translation
            #rot = m.transform.rotation
            #t.transform.translation.x = trans.x
            #t.transform.translation.y = trans.y
            #t.transform.translation.z = trans.z
            #t.transform.rotation.x = rot.x
            #t.transform.rotation.y = rot.y
            #t.transform.rotation.z = rot.z
            #t.transform.rotation.w = rot.w

            #self.br.sendTransform(t)  #(only needed for rviz)

            # Found it:
            if t.child_frame_id == self.target_fiducial:
                found = True
                self.tfBuffer.set_transform(t, "fid_tracker")

        # Didn't find it:
        if not found:
            return

        # Get fiducial position relative to "base_link" frame:
        try:
            tf = self.tfBuffer.lookup_transform("base_link", self.target_fiducial, imageTime)
            ct = tf.transform.translation
            cr = tf.transform.rotation
            #print "T_fidBase %lf %lf %lf %lf %lf %lf %lf\n" % \
            #                 (ct.x, ct.y, ct.z, cr.x, cr.y, cr.z, cr.w)

            # Set fiducial tracking vars:
            self.fid_x = ct.x
            self.fid_y = ct.y
            self.got_fid = True
        except:
            traceback.print_exc()
            print "Could not get tf for %s" % self.target_fiducial


    # [ MAIN ]
    def run(self):
        # Initialize vars:
        rate = rospy.Rate(20)  #(loop @ 20hz)
        linSpeed = 0.0
        angSpeed = 0.0
        times_since_last_fid = 0

        while not rospy.is_shutdown() and self.can_run:
            # Calculate dimensional corrections between robot + fiducial:
            fwd_error = self.fid_x - self.min_dist
            lat_error = self.fid_y
            angl_error = math.atan2(self.fid_y, self.fid_x)

            #print "Errors: forward %f lateral %f angular %f" % \
            #               (fwd_error, lat_error, degrees(angl_error))

            # Keep track for "lost" section of code:
            if self.got_fid:
                times_since_last_fid = 0
            else:
                times_since_last_fid += 1

            # If fiducial is too far/not detected, give up:
            if fwd_error > self.max_dist:
                linSpeed = 0
                angSpeed = 0

            # Else if fiducial is detected, go home:
            elif self.got_fid:
                # Set the turning speed based on "angl_error":
                # (add some damping based on previous speed)
                angSpeed = angl_error * self.angular_rate - angSpeed / 2.0

                # Stay within defined angular speed limits:
                if angSpeed < -self.max_angular_rate:
                    angSpeed = -self.max_angular_rate
                if angSpeed > self.max_angular_rate:
                    angSpeed = self.max_angular_rate

                # Set the forward speed based on "fwd_error":
                linSpeed = fwd_error * self.linear_rate

                # Stay within defined linear speed limits:
                if linSpeed < -self.max_linear_rate:
                    linSpeed = -self.max_linear_rate
                if linSpeed > self.max_linear_rate:
                    linSpeed = self.max_linear_rate

            # Else if fiducial is lost, rotate to re-find it:
            elif self.got_fid == False and times_since_last_fid < self.max_lost_count:
                # Stop...
                linSpeed = 0
                # ... then turn...
                if angSpeed < 0:
                    angSpeed = -self.lost_angular_rate
                elif angSpeed > 0:
                    angSpeed = self.lost_angular_rate
                else:
                    angSpeed = 0
                #print "Try keep rotating to refind fiducial: try# %d" % times_since_last_fid
            else:
                angSpeed = 0
                linSpeed = 0

            #print "Speeds: linear %f angular %f" % (linSpeed, angSpeed)

            # Create a Twist message from the velocities and publish it:
            # (avoid sending repeated zero-speed commands)
            zeroSpeed = (angSpeed == 0 and linSpeed == 0)
            if not zeroSpeed:
                self.suppressCmd = False
            if not self.suppressCmd:
                twist = Twist()
                twist.angular.z = angSpeed
                twist.linear.x = linSpeed
                self.cmdPub.publish(twist)
                if zeroSpeed:
                    self.suppressCmd = True

            # We already acted on the current fiducial, so pause this node:
            self.got_fid = False
            rate.sleep()


if __name__ == "__main__":
    # Create an instance of our follow class...
    node = FidTracker()
    # ... then run it!
node.run()