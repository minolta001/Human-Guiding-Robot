import rospy
import tf

listener = tf.TransformListener()
rospy.sleep(1.0)  # Wait for the tf tree to populate

while True:
    try:
        (trans, rot) = listener.lookupTransform("odom", "map", rospy.Time(0))
        # 'trans' represents the translation [x, y, z]
        # 'rot' represents the rotation [x, y, z, w]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        # Handle exception if the transform is not available
        pass
    
    rospy.sleep(1.0)