import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javier-rojas/Documents/Doctorado/3er_Semestre/ur_pick_and_place/install/vision_pick_init'
