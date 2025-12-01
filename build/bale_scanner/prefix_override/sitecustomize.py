import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tatan/fis_ws/src/bale_scanner/install/bale_scanner'
