#!/usr/bin/env python3

import os
import argparse
import rosbag
import math
import numpy as np
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation
import tf

def extract(bagfile, pose_topic, msg_type, out_filename):
    n = 0
    f = open(out_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw\n')
    with rosbag.Bag(bagfile, 'r') as bag:
        for (topic, msg, ts) in bag.read_messages(topics=str(pose_topic)):
            if msg_type == "PoseWithCovarianceStamped":
                # Position conversion
                flu_position = [msg.pose.pose.position.x, -msg.pose.pose.position.y, -msg.pose.pose.position.z]

                # Orientation conversion
                frd_quaternion = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
                frd_rotation = Rotation.from_quat(frd_quaternion)
                frd_to_flu_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
                flu_rotation = frd_to_flu_matrix @ frd_rotation.as_matrix()
                flu_quaternion = Rotation.from_matrix(flu_rotation).as_quat()

                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                        (msg.header.stamp.to_sec(),
                         flu_position[0], 
                         flu_position[1],
                         flu_position[2],
                         flu_quaternion[3],
                         flu_quaternion[0],
                         flu_quaternion[1],
                         flu_quaternion[2]))
            else:
                assert False, "Unknown message type"
            n += 1
    print(('wrote ' + str(n) + ' messages to the file: ' + out_filename))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='''
    Extracts messages from bagfile.
    ''')
    parser.add_argument('bag', help='Bagfile')
    parser.add_argument('topic', help='Topic')
    parser.add_argument('--msg_type', default='PoseStamped',
                        help='message type')
    parser.add_argument('--output', default='stamped_poses.txt',
                        help='output filename')
    args = parser.parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.bag))
    out_fn = os.path.join(out_dir, args.output)

    print(('Extract pose from bag '+args.bag+' in topic ' + args.topic))
    print(('Saving to file '+out_fn))
    extract(args.bag, args.topic, args.msg_type, out_fn)
