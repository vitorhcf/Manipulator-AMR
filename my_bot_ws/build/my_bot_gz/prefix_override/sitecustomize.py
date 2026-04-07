import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vitor/my_bot_ws/my_bot_ws/install/my_bot_gz'
