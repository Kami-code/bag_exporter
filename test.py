import pyrealsense2 as rs
import numpy as np
import cv2
import os
# Import argparse for command-line options
import argparse
# Import os.path for file path manipulation

from PIL import Image

opencv_visualize = False
data_dir_path = '/home/baochen/Downloads/cabinet_clutter_env1/success/'


if __name__ == '__main__':
    for _, _, a in os.walk(data_dir_path):
        files = a
    scan_names = []
    file_names = []
    for file in files:
        scan_names.append(os.path.join(data_dir_path, file))
        file_names.append(file)
    scan_names.sort()
    file_names.sort()
    print("scan names = ", scan_names)

    for i in range(0, len(scan_names), 1):
        # Create pipeline
        pipeline = rs.pipeline()

        # Create a config object
        config = rs.config()

        # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
        rs.config.enable_device_from_file(config, scan_names[i], repeat_playback=False)

        # Configure the pipeline to stream the depth stream
        # Change this parameters according to the recorded bag file resolution
        config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)

        # Start streaming from file
        pipeline.start(config)

        # Create opencv window to render image in
        cv2.namedWindow("RGB Stream", cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow("Depth Stream", cv2.WINDOW_AUTOSIZE)

        # Create colorizer object
        colorizer = rs.colorizer()

        gifs_color = []
        gifs_depth = []
        # Streaming loop
        try:
            while True:
                # Get frameset of depth
                frames = pipeline.wait_for_frames(timeout_ms=1000)

                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                # Colorize depth frame to jet colormap
                depth_color_frame = colorizer.colorize(depth_frame)

                # Convert depth_frame to numpy array to render image in opencv
                rgb_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_color_frame.get_data())

                if opencv_visualize:
                    # Render image in opencv window
                    cv2.imshow("RGB Stream", rgb_image)
                    cv2.imshow("Depth Stream", depth_image)
                    key = cv2.waitKey(1)

                frame_ = Image.fromarray(rgb_image)
                gifs_color.append(frame_)
                frame_d = Image.fromarray(depth_image)
                gifs_depth.append(frame_d)


        except RuntimeError:  # RuntimeError means the video ends
            pass
        print('开始生成gif...')
        gifs_color[0].save(
            './{}_rgb.gif'.format(file_names[i][:-4]),
            format='GIF', append_images=gifs_color,
            save_all=True, duration=20, loop=0)
        gifs_depth[0].save(
            './{}_depth.gif'.format(file_names[i][:-4]),
            format='GIF', append_images=gifs_depth,
            save_all=True, duration=20, loop=0)
