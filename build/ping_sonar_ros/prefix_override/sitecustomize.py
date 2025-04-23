import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ahmed/Desktop/2nd_semester/blueRov/bluerov_ws/install/ping_sonar_ros'
