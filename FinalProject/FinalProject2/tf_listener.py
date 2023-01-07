#!/usr/bin/env python3

import roslib
# roslib.load_manifest('learning_tf')
import rospy
import math
import tf
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Float64

def tf_list():
    rospy.init_node('tf_listener', anonymous=False)
    
    pub = rospy.Publisher('tf_listener_pub', PoseWithCovariance, queue_size = 10)
    listener = tf.TransformListener()
    
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
#        try:
            (trans, rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
            
            msg = [trans[1], trans[2], trans[3], rot[1], rot[2], rot[3], rot[4]]
            pub.publish(msg)
            
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            continue
            
        rate.sleep()
        
        
if __name__ == '__main__':
	try:
		tf_list()
	except rospy.ROSInterruptException:
		pass
        
 # insert trans and rot in a covariance message
