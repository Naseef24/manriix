#!/usr/bin/env python3

import sys
import os

install_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
python_path = os.path.join(install_dir, 'local', 'lib', 'python3.10', 'dist-packages')
if python_path not in sys.path:
    sys.path.insert(0, python_path)

from dji_rs3pro_ros_controller.control_gimbal_ros2 import main

if __name__ == '__main__':
    main()
