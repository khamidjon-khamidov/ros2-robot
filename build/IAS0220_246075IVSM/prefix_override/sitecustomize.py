import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hamidjon/ros2_ws/src/IAS0220_246075IVSM/install/IAS0220_246075IVSM'
