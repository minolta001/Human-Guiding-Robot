import rospy
from sensor_msgs.msg import LaserScan

min_val = 0

def scan_callback(scan_data):
    global min_val
    ranges = scan_data.ranges
    min_val = min(ranges)
    min_idx = scan_data.ranges.index(min_val)

    front10 = ranges[:10]
    back10 = ranges[-10:]
    front_ranges = front10 + back10 # the scan readings from heading 30 degree
    front_min = min(front_ranges)
    front_min_idx = front_ranges.index(front_min)




if __name__ == "__main__":
    rospy.init_node("scan_test")
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    try:
        while(1):
            print(min_val)
            continue
            if KeyboardInterrupt:
                raise KeyboardInterrupt
                break
    except KeyboardInterrupt:
        print("done")
