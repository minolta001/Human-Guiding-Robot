import rospy
import cv2
import numpy as np
import pyrealsense2 as rs

def main():
    rospy.init_node('d435_camera_node', anonymous=True)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    pipeline.start(config)

    try:
        while not rospy.is_shutdown():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Process and use the color and depth images as needed
            #cv2.imshow("Color Image", color_image)
            #cv2.imshow("Depth Image", depth_image)
            #cv2.waitKey(1)

            # Convert color image to HSV color space
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # Define lower and upper bounds for yellow color
            lower_red = np.array([20, 100, 100])
            upper_red = np.array([40, 255, 255])

            red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
            red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

            # Find contours in the red mask
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
             
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Filter small contours
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    center = (int(x), int(y))
                    radius = int(radius)

                    # Get depth at the center of the circle
                    depth = depth_image[int(y), int(x)]

                    # Convert depth to meters using the depth scale
                    depth_scale = pipeline.get_active_profile().get_device().first_depth_sensor().get_depth_scale()
                    depth = depth * depth_scale 

                    # Display the circle and its distance
                    cv2.circle(color_image, center, radius, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Distance: {depth:.2f} meters", (int(x) - radius, int(y) - radius - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            

            # Display the color image with detected circles and distances
            cv2.imshow("Color Image", color_image)
            cv2.waitKey(1)



    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()