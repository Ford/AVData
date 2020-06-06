#!/usr/bin/env python
"""
Broadcast static transform
"""

import sys
import os
import yaml


def main():
    """Main function.
    Reading transform info from a yaml file and publish to tf2
    """
    if len(sys.argv) == 1:
        print("error: no extrinsics yaml file given")
        print("usage: python extrinsics_broadcaster.py extrinsic_example.yaml")
        return

    file_path = open(sys.argv[1])
    print(file_path)
    transform_stamped = yaml.safe_load(file_path)
    command = 'rosrun tf2_ros static_transform_publisher '\
        '%f %f %f %f %f %f %f %s %s' % (transform_stamped['transform']['translation']['x'],
                                        transform_stamped['transform']['translation']['y'],
                                        transform_stamped['transform']['translation']['z'],
                                        transform_stamped['transform']['rotation']['x'],
                                        transform_stamped['transform']['rotation']['y'],
                                        transform_stamped['transform']['rotation']['z'],
                                        transform_stamped['transform']['rotation']['w'],
                                        transform_stamped['header']['frame_id'],
                                        transform_stamped['child_frame_id'])

    print(command)
    ret = os.system(command)
    print(ret)


if __name__ == "__main__":
    main()
