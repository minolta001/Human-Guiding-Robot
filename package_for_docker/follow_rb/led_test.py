import rospy
from std_msgs.msg import Int32

rospy.init_node("led_test")
pub = rospy.Publisher('/color_cmd', Int32, queue_size = 1)

color_cmd_msg = Int32()

color_cmd_msg.data = 0xFF30

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(color_cmd_msg)
    rate.sleep()
