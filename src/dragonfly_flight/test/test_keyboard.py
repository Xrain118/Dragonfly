import math
import time
from types import SimpleNamespace
import numpy as np
from std_msgs.msg import String
from dragonfly_flight.arms import ArmCommands
from dragonfly_flight.core import HOME, ARM_NAMES

class Node:
    time=1.
    def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(self.time*1e9)))
    def create_publisher(self,*args):
        messages=[]
        return SimpleNamespace(publish=messages.append,messages=messages)
    def create_subscription(self,*args):pass

def manager():
    node=Node()
    arm=ArmCommands(node,dict(arm_speed=.1,arm_acceleration=.2,arm_step_degrees=2.,arm_range_degrees=10.))
    arm.q={n:HOME[i] for names in ARM_NAMES.values() for i,n in enumerate(names)}
    arm.state_time=node.time;arm.heartbeat=time.monotonic();arm.ready=True
    return node,arm

def test_all_twenty_key_directions():
    for side,key in [('left','L'),('right','R')]:
        for joint in range(5):
            for direction,sign in [('+',1),('-',-1)]:
                node,arm=manager()
                for k in (key,str(joint+1),direction):arm.key(String(data=k))
                target=HOME.copy();target[joint]+=sign*math.radians(2)
                assert np.allclose(arm.pub[side].messages[-1].points[-1].positions,target)
                assert not arm.pub['right' if side=='left' else 'left'].messages

def test_busy_input_not_queued_and_stop_keeps_flight_separate():
    node,arm=manager();arm.key(String(data='+'));first=arm.target['left'].copy()
    arm.key(String(data='+'));assert np.array_equal(first,arm.target['left'])
    assert len(arm.pub['left'].messages)==1
    arm.key(String(data=' '));assert not arm.sequence and not arm.manual
    assert len(arm.pub['right'].messages)==1

def test_not_ready_stale_state_and_heartbeat_loss():
    node,arm=manager();arm.ready=False;arm.key(String(data='+'));assert not arm.pub['left'].messages
    arm.ready=True;node.time=2.;arm.key(String(data='+'));assert not arm.pub['left'].messages
    arm.state_time=2.;arm.key(String(data='+'));arm.heartbeat-=.6;arm.tick();assert not arm.manual

def test_home_limits_and_replay():
    node,arm=manager();arm.target['left'][0]=math.radians(10)
    arm.key(String(data='+'));assert np.isclose(arm.target['left'][0],math.radians(10))
    arm.busy_until=0.;arm.key(String(data='H'));assert np.allclose(arm.target['left'],HOME)
    arm.busy_until=0.;arm.key(String(data='P'));assert len(arm.sequence)==3
    assert arm.sequence[1][1][1]==math.radians(5) and arm.sequence[1][2]==2
