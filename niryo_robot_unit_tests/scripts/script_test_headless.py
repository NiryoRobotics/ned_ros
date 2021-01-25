#!/usr/bin/env python

import sys
from pipeline_test_ros_n_pure_python import test_pipeline

if __name__ == '__main__':
    optional_args = [
        "conveyor:=false",
        "gripper_n_camera:=false",
        "gui:=false",
    ]

    success_bool = test_pipeline(show_errors=True, use_simulation=True,
                                 optional_args=optional_args)
    sys.exit(0 if success_bool else 1)
