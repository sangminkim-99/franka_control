# posed_rgbd_logger.py
# save the pose and rgb/depth images to disk
# Topic: /franka/rgb, /franka/depth, /franka/pose

from argparse import ArgumentParser
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import message_filters
import os
from scipy.spatial.transform import Rotation
import numpy as np


class PosedRGBDLogger:
    def __init__(self, logdir):
        # Params
        self.rgb, self.depth, self.pose = None, None, None
        self.br = CvBridge()
        self.image_count = 0
        self.logdir = logdir

        # Subscribers
        sub_rgb = rospy.Subscriber("/franka/rgb", Image)
        sub_depth = rospy.Subscriber("/franka/depth", Image)
        sub_pose = rospy.Subscriber("/franka/pose", Pose)

        ts = message_filters.TimeSynchronizer(
            [sub_rgb, sub_depth, sub_pose], queue_size=1
        )
        ts.registerCallback(self.callback)
        rospy.spin()

    def callback(self, rgb, depth, pose):
        self.rgb = self.br.imgmsg_to_cv2(rgb)
        self.depth = self.br.imgmsg_to_cv2(depth)
        self.pose = pose

        rgb_filename = os.path.join(self.logdir, f"rgb_{self.image_count}.jpg")
        depth_filename = os.path.join(self.logdir, f"depth_{self.image_count}.jpg")
        pose_filename = os.path.join(self.logdir, f"pose_{self.image_count}.txt")

        cv2.imwrite(rgb_filename, self.rgb)
        cv2.imwrite(depth_filename, self.depth)

        position = self.pose.position
        quat = self.pose.orientation

        trans = np.asarray([position.x, position.y, position.z])

        self.image_count += 1


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("--logdir", type=str, default="./data")

    args = parser.parse_args()

    # create directory
    os.makedirs(args.logdir, exist_ok=True)

    rospy.init_node("posed_rgbd_logger", anonymous=True)
    posed_rgbd_logger = PosedRGBDLogger(args.logdir)