#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory,get_package_prefix

import os

def main():
    
    pkg_dir = get_package_share_directory('voice_assistant')
    
    cmd = os.path.join(pkg_dir,"rhasspy3","script/run") +" bin/pipeline_run.py --debug"
    
    os.system(cmd)


main()