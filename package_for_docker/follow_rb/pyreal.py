import pyrealsense2.pyrealsense2 as rs

if __name__ == '__main__':
    pipe = rs.pipeline()
    profile = pipe.start()
    try:
        for i in range(0, 100):
            frames = pipe.wait_for_frames()
            for f in frames:
                print(f.profile)
    finally:
        pipe.stop()
