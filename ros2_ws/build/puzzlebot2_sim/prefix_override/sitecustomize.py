import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tony/IntegracionRobotica/ros2_ws/install/puzzlebot2_sim'
