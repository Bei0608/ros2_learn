import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bei/ros2_learn/chapt2/install/demo_python_pkg'
