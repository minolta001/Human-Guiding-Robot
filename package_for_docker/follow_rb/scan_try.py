import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    ranges = scan_data.ranges
    min_val = min(ranges)
    min_idx = ranges.index(min_val)

    front10 = ranges[:10]
    back10 = ranges[-10:]
    front_ranges = front10 + back10 # the scan readings from heading 30 degree
    front_min = min(front_ranges)
    front_min_idx = front_ranges.index(front_min)


    print(min_val, front_min, front_min_idx)

rospy.init_node("scan_test")
rospy.Subscriber('/scan', LaserScan, scan_callback)