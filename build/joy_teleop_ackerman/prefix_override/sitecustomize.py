import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/aavjetson/Desktop/joy_teleop_ackerman/install/joy_teleop_ackerman'
