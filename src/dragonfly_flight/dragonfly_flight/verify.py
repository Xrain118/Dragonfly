"""Evaluate recorded truth, never controller estimates, against declared criteria."""
import argparse
import csv
import json
import math
from pathlib import Path
import numpy as np
from .clearance import GearClearance

def evaluate(directory):
    rows=list(csv.DictReader((directory/'flight.csv').open()))
    if not rows:raise ValueError('No flight samples')
    hover=[r for r in rows if r['phase']=='hover']
    if not hover:raise ValueError('No hover interval')
    errors=np.array([[float(r[k]) for k in ('ex','ey','ez')] for r in hover])
    angles=np.array([float(r['angle']) for r in hover])
    duration=float(hover[-1]['t'])-float(hover[0]['t'])
    result=dict(hover_seconds=duration,position_rms=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
                attitude_rms_deg=float(np.degrees(np.sqrt(np.mean(angles**2)))))
    longest=current=0.
    for prev,row in zip(rows,rows[1:]):
        current=current+float(row['t'])-float(prev['t']) if int(row['saturated']) else 0.;longest=max(longest,current)
    result['longest_saturation_seconds']=longest
    result['hover_pass']=duration>=29.9 and result['position_rms']<=.10 and result['attitude_rms_deg']<=3 and longest<=1
    tracks={}
    for phase in ('track0','track1','track2','track3'):
        samples=[r for r in rows if r['phase']==phase]
        if samples:
            # Reference motion takes 3 s; evaluate 5 s after its completion.
            deadline=float(samples[0]['t'])+8
            end=min(samples,key=lambda r:abs(float(r['t'])-deadline))
            tracks[phase]=float(np.linalg.norm([float(end[k]) for k in ('ex','ey','ez')]))
    result['tracking_errors_at_5s']=tracks
    arm=[r for r in rows if r['phase'].startswith('arms_')]
    if arm:
        result['arm_peak_position']=max(float(np.linalg.norm([float(r[k]) for k in ('ex','ey','ez')])) for r in arm)
        result['arm_peak_angle_deg']=max(math.degrees(float(r['angle'])) for r in arm)
    result['passed']=result['hover_pass'] and all(x<=.10 for x in tracks.values()) and not (directory/'failure.txt').exists()
    if arm:result['passed'] &= result['arm_peak_position']<=.25 and result['arm_peak_angle_deg']<=8
    if (directory/'run.json').exists() and json.loads((directory/'run.json').read_text())['scenario']=='acceptance':
        recovery={};events=json.loads((directory/'events.json').read_text()) if (directory/'events.json').exists() else {}
        for side in ('left','right'):
            samples=[r for r in arm if r['phase']=='arms_'+side]
            if samples:
                end=min(samples,key=lambda r:abs(float(r['t'])-(events[side+'_end']+5))) if side+'_end' in events else samples[-1]
                recovery[side]={'position':float(np.linalg.norm([float(end[k]) for k in ('ex','ey','ez')])),
                                'angle_deg':math.degrees(float(end['angle']))}
                shoulder='q1' if side=='left' else 'q6'
                recovery[side]['shoulder_travel_deg']=math.degrees(np.ptp([float(r[shoulder]) for r in samples]))
        result['arm_recovery']=recovery
        result['passed'] &= len(tracks)==4 and len(recovery)==2 and all(r['position']<=.10 and r['angle_deg']<=3 and r['shoulder_travel_deg']>=4.5 for r in recovery.values())
        guard=GearClearance(directory/'robot.urdf')
        samples=rows[::10]
        result['sampled_gear_clearance_pass']=all(guard.allows(side,q,q) for r in samples
            for side,offset in [('left',0),('right',5)]
            for q in [np.array([float(r[f'q{i}']) for i in range(offset,offset+5)])])
        result['max_arm_effort_nm']=max(abs(float(r[f'effort{i}'])) for r in rows for i in range(10))
        if 'tilt_effort0' in rows[0]:
            result['max_tilt_effort_nm']=max(abs(float(r[f'tilt_effort{i}'])) for r in rows for i in range(6))
        model=json.loads((directory/'model.json').read_text())
        result['rotor_mode']=model['mode']=='px4_actuators'
        result['passed'] &= result['sampled_gear_clearance_pass'] and result['max_arm_effort_nm']<model['config']['arm_effort'] and result['rotor_mode']
    (directory/'metrics.json').write_text(json.dumps(result,indent=2))
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('directory',type=Path);args=p.parse_args()
    result=evaluate(args.directory);print(json.dumps(result,indent=2))
    if not result['passed']:raise SystemExit(1)
