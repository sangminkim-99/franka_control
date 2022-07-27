import argparse
import glob
import numpy as np
from franka_env import FrankaEnv
import os
from tqdm import tqdm

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge

class PosedRGBDPublisher(object):
    def __init__(self):
        # Params
        self.rgb, self.depth, self.pose = None, None, None
        self.br = CvBridge()
        self.loop_rate = rospy.Rate(30)        # Node cycle rate (in Hz).

        # Publishers
        self.rgb_pub = rospy.Publisher('/franka/rgb', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/franka/depth', Image, queue_size=1)
        self.pose_pub = rospy.Publisher('/franka/pose', Pose, queue_size=1)

    def publish(self):
        # rospy.loginfo('publishing image')
        if self.rgb is not None:
            self.rgb_pub.publish(self.br.cv2_to_imgmsg(self.rgb))
        if self.depth is not None:
            self.depth_pub.publish(self.br.cv2_to_imgmsg(self.depth))
        if self.pose is not None:
            self.pose_pub.publish(self.pose)
        self.loop_rate.sleep()

parser = argparse.ArgumentParser()
parser.add_argument("file")

def _separate_filename(filename):
    split = filename[:-4].split("_")
    name = "_".join(split[:-1:])
    i = int(split[-1])
    return name, i


def _format_out_dict(list_obs, actions, hz, home):
    out_dict = {k: [] for k in list(list_obs[0].keys())}
    for obs in list_obs:
        for k in out_dict.keys():
            out_dict[k].append(obs[k])
    out_dict = {k: np.array(v) for k, v in out_dict.items()}

    out_dict["actions"] = actions
    out_dict["rate"] = hz
    out_dict["home"] = home
    return out_dict

if __name__ == "__main__":
    args = parser.parse_args()
    rospy.init_node("playback", anonymous=True)
    rgbd_node = PosedRGBDPublisher()

    gain_type = "default"
    camera = True
    data = np.load(args.file)
    home, traj, hz = data["home"], data["traj"], data["hz"]
    env = FrankaEnv(home=home, hz=hz, gain_type=gain_type, camera=camera)
    print(f"Traj: {args.file}, Gain type: {gain_type}, Camera: {camera}")

    user_in = "r"
    while user_in == "r":
        user_in = input(f"Ready. Loaded {args.file} ({hz} hz):")
    
    # Infinite loop trajectory: Cntrl + C to break 
    while True:
        print("Reset!")
        obs = [env.reset()]
        actions = []

        for acs in tqdm(data["traj"]):
            actions.append(acs)
            obs.append(env.step(acs)[0])
            curr_data = env._get_obs()
            if camera: 
                rgbd_node.rgb = curr_data['rgb']
                rgbd_node.depth = curr_data['depth']
            p = Pose()
            p.position.x, p.position.y, p.position.z = curr_data['eep'][0], curr_data['eep'][1], curr_data['eep'][2]
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = curr_data['eep'][3], curr_data['eep'][4], curr_data['eep'][5], curr_data['eep'][6]
            rgbd_node.pose = p
            rgbd_node.publish()
            # print(data)
    env.close()

    out_dict = _format_out_dict(obs, np.array(actions), hz, home)