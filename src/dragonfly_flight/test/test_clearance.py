import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from ament_index_python.packages import get_package_share_directory
from dragonfly_flight.core import HOME
from dragonfly_flight.clearance import GearClearance

def test_jogs_replay_and_gear_rejection(tmp_path):
    root=ET.fromstring(subprocess.check_output(['xacro',get_package_share_directory('dragonfly_arm')+'/urdf/dragonfly_arm.urdf.xacro']))
    root.remove(root.find("joint[@name='world_to_uav']"))
    path=tmp_path/'robot.urdf';ET.ElementTree(root).write(path)
    guard=GearClearance(path)
    for side in ('left','right'):
        for joint in range(5):
            for sign in (-1,1):
                q=HOME.copy();q[joint]+=sign*np.radians(2)
                assert guard.allows(side,HOME,q)
                assert guard.allows(side,q,HOME)
        replay=HOME.copy();replay[1]+=np.radians(5)
        assert guard.allows(side,HOME,replay) and guard.allows(side,replay,HOME)
        toward_gear=HOME.copy();toward_gear[0]+=np.radians(-10 if side=='left' else 10)
        assert not guard.allows(side,HOME,toward_gear)
