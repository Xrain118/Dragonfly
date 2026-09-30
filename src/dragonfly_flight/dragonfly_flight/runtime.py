"""Control, PX4 adapter and experiment sequencing. Truth never feeds PX4-mode control."""
import argparse
import csv
import json
import math
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, PoseStamped, TransformStamped, WrenchStamped
from std_msgs.msg import Bool, String, Float32MultiArray
from tf2_ros import TransformBroadcaster
from control_msgs.msg import JointTrajectoryControllerState
from px4_msgs.msg import (VehicleLocalPosition, VehicleOdometry, VehicleStatus, VehicleCommand,
                          VehicleCommandAck, OffboardControlMode, ActuatorMotors, ActuatorServos)
from .core import Platform, Allocator, NED_ENU, FRD_FLU, px4_rotation, smoothstep, HOME
from .arms import ArmCommands
from .clearance import GearClearance

class Runtime(Node):
    def __init__(self,out,scenario):
        super().__init__('dragonfly_flight',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        data=json.loads((out/'model.json').read_text());self.mode=data['mode'];self.out=out;self.scenario=scenario
        self.model=Platform(data['dimensions'],data['config'],data['mass'],data['com'],data['inertia']);self.alloc=Allocator(self.model)
        self.truth=None;self.local=None;self.odom=None;self.vehicle=None;self.offset=None
        self.local_time=-1.;self.odom_time=-1.;self.begin=None;self.start_height=None;self.home_xy=None
        self.last_command=-10.;self.last_t=None;self.failures=0;self.done=False;self.error=None;self.last_log=-1.
        self.target=np.array([0.,0.,self.model.cfg['hover_height']]);self.external_target=None
        self.arms=ArmCommands(self,self.model.cfg,GearClearance(out/'robot.urdf'));self.initialized_arms=False;self.replay_index=0;self.replay_started=False
        self.events={};self.previous_arm_busy=False
        self.force_probe=None;self.probe_samples=[]
        self.momentum=[];self.last_momentum=None;self.efforts=np.zeros(10);self.rotor_speeds=np.zeros(6)
        self.arm_reference=np.tile(HOME,2)
        self.tilt_efforts=np.zeros(6)
        for index,side in enumerate(('left','right')):
            self.create_subscription(JointTrajectoryControllerState,f'/{side}_arm_controller/controller_state',
                                     lambda m,i=index:self.on_arm_reference(m,i),qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/dragonfly/status',10)
        self.diagnostics=self.create_publisher(Float32MultiArray,'/dragonfly/diagnostics',10)
        self.create_subscription(Odometry,'/dragonfly/truth',self.on_truth,qos_profile_sensor_data)
        self.create_subscription(WrenchStamped,'/dragonfly/momentum',self.on_momentum,qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/efforts',lambda m:setattr(self,'efforts',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/rotor_speeds',lambda m:setattr(self,'rotor_speeds',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(String,'/dragonfly/tilt_efforts',lambda m:setattr(self,'tilt_efforts',np.fromstring(m.data,sep=',')),qos_profile_sensor_data)
        self.create_subscription(PoseStamped,'/dragonfly/target',self.on_target,10)
        self.tf=TransformBroadcaster(self)
        if self.mode=='px4_actuators':
            for name,kind,callback in [('vehicle_local_position',VehicleLocalPosition,self.on_local),('vehicle_odometry',VehicleOdometry,self.on_odom),
                                       ('vehicle_status',VehicleStatus,lambda m:setattr(self,'vehicle',m)),
                                       ('vehicle_command_ack',VehicleCommandAck,lambda m:self.get_logger().info(f'PX4 ack {m.command}: {m.result}'))]:
                version=getattr(kind,'MESSAGE_VERSION',0)
                topic='/fmu/out/'+name+(f'_v{version}' if version else '')
                self.create_subscription(kind,topic,callback,qos_profile_sensor_data)
            self.offboard=self.create_publisher(OffboardControlMode,'/fmu/in/offboard_control_mode',10)
            self.commands=self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command',10)
            self.motors=self.create_publisher(ActuatorMotors,'/fmu/in/actuator_motors',10)
            self.servos=self.create_publisher(ActuatorServos,'/fmu/in/actuator_servos',10)
        self.ideal=self.create_publisher(Wrench,'/dragonfly/ideal_wrench',10) if self.mode=='ideal_wrench' else None
        self.file=open(out/'flight.csv','w',newline='');self.csv=csv.writer(self.file)
        self.csv.writerow(['t','phase','x','y','z','rx','ry','rz','ex','ey','ez','angle','est_error','force_residual','torque_residual','saturated']+
                          [f'q{i}' for i in range(10)]+[f'effort{i}' for i in range(10)]+[f'f{i}' for i in range(6)]+[f'tilt{i}' for i in range(6)]+['est_angle']+[f'actual_tilt{i}' for i in range(6)]+[f'actual_f{i}' for i in range(6)]+[f'qref{i}' for i in range(10)]+[f'tilt_effort{i}' for i in range(6)])
        self.timer=self.create_timer(.01,self.tick)
        self.wall_start=time.monotonic();self.last_heartbeat=0.;self.get_logger().info(f'Mode {self.mode}; mass={self.model.mass:.3f}, com={self.model.com}')

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def on_arm_reference(self,m,index):
        reference=m.reference if hasattr(m,'reference') else m.desired
        if len(reference.positions)==5:self.arm_reference[index*5:index*5+5]=reference.positions
    def on_truth(self,m):
        self.truth=m
        tf=TransformStamped();tf.header.stamp=m.header.stamp;tf.header.frame_id='world';tf.child_frame_id='uav_base_link'
        tf.transform.translation.x=m.pose.pose.position.x;tf.transform.translation.y=m.pose.pose.position.y;tf.transform.translation.z=m.pose.pose.position.z
        tf.transform.rotation=m.pose.pose.orientation;self.tf.sendTransform(tf)
    def on_local(self,m):self.local=m;self.local_time=self.now()
    def on_odom(self,m):self.odom=m;self.odom_time=self.now()
    def on_momentum(self,m):
        self.last_momentum=np.array([m.wrench.force.x,m.wrench.force.y,m.wrench.force.z])
        self.last_momentum_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if self.mode=='physics':self.momentum.append([self.now(),m.wrench.force.x,m.wrench.force.y,m.wrench.force.z,m.wrench.torque.x,m.wrench.torque.y,m.wrench.torque.z])
    def on_target(self,m):
        values=np.array([m.pose.position.x,m.pose.position.y,m.pose.position.z])
        if m.header.frame_id!='world' or not np.isfinite(values).all():return
        self.external_target=values
    def send_command(self,command,**params):
        m=VehicleCommand(timestamp=int(self.now()*1e6),command=command,target_system=1,target_component=1,source_system=1,source_component=1,from_external=True,**params)
        self.commands.publish(m)
    def abort(self,reason):
        self.error=reason;self.done=True;self.arms.ready=False;self.arms.stop()
        self.get_logger().error(reason);(self.out/'failure.txt').write_text(reason)
    def truth_state(self):
        m=self.truth;v=m.twist.twist;p=m.pose.pose.position;q=m.pose.pose.orientation
        rot=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
        return np.array([p.x,p.y,p.z]),np.array([v.linear.x,v.linear.y,v.linear.z]),rot,rot.T@np.array([v.angular.x,v.angular.y,v.angular.z])

    def tick(self):
        try:self.step()
        except (ValueError,RuntimeError,FloatingPointError) as e:self.abort(str(e))
    def step(self):
        t=self.now()
        if self.done:return
        if self.truth is None or not self.arms.initialized():return
        tp,tv,tr,tw=self.truth_state()
        if not self.initialized_arms:
            for side in ('left','right'):
                self.arms.busy_until=0.;self.arms.move(side,HOME)
            self.initialized_arms=True
        self.arms.tick()
        arm_busy=bool(self.arms.sequence or t<self.arms.busy_until)
        if self.previous_arm_busy and not arm_busy and self.replay_index:
            self.events['left_end' if self.replay_index==1 else 'right_end']=t-self.begin
            (self.out/'events.json').write_text(json.dumps(self.events,indent=2))
        self.previous_arm_busy=arm_busy
        if self.mode=='physics':
            self.arms.ready=True
            if t>3 and not self.replay_started:self.arms.replay('left');self.replay_started=True
            q=[self.arms.q[n] for n in sum(ARM_NAMES_LIST,[])]
            self.csv.writerow([t,'physics',*tp,0,0,1.6,*tv,Rotation.from_matrix(tr).magnitude(),0,0,0,0,*q,*self.efforts,*np.zeros(25),*self.arm_reference,*self.tilt_efforts])
            if t>16:
                np.savetxt(self.out/'momentum.csv',self.momentum,delimiter=',',header='t,px,py,pz,lx,ly,lz');self.done=True
            return
        if self.mode=='px4_actuators':
            self.offboard.publish(OffboardControlMode(timestamp=int(t*1e6),direct_actuator=True))
            if self.local is None or self.odom is None or self.vehicle is None:return
            if not self.local.xy_valid or not self.local.z_valid:
                if self.begin is not None:self.abort('PX4 position estimate invalid')
                return
            p=NED_ENU@np.array([self.local.x,self.local.y,self.local.z])
            if self.offset is None:self.offset=tp-p;self.home_xy=tp[:2].copy()
            p=p+self.offset;v=NED_ENU@np.array([self.local.vx,self.local.vy,self.local.vz])
            rot=px4_rotation(self.odom.q);omega=FRD_FLU@np.array(self.odom.angular_velocity)
            if self.vehicle.nav_state!=14 or self.vehicle.arming_state!=2:
                if self.begin is not None:self.abort('PX4 left armed Offboard mode');return
                # Align the two origins while landed, before the estimator's
                # altitude reference settles. Never use truth for flight feedback.
                self.offset=tp-NED_ENU@np.array([self.local.x,self.local.y,self.local.z])
                if t-self.last_command>2 and t>5:
                    if self.vehicle.nav_state!=14:self.send_command(176,param1=1.,param2=6.)
                    else:self.send_command(400,param1=1.)
                    self.last_command=t
                return
            if t-min(self.local_time,self.odom_time)>.2:self.abort('PX4 state older than 0.2 simulation seconds');return
        else:p,v,rot,omega=tp,tv,tr,tw
        if not np.isfinite(np.r_[p,v,rot.ravel(),omega]).all():self.abort('Non-finite state');return
        if self.begin is None:
            self.begin=t;self.start_height=p[2];self.home_xy=p[:2].copy();self.get_logger().info('Flight started')
        elapsed=t-self.begin
        target=np.r_[self.home_xy,self.start_height+(self.model.cfg['hover_height']-self.start_height)*smoothstep(elapsed/6)]
        phase='takeoff' if elapsed<14 else 'hover'
        if self.scenario=='acceptance' and elapsed>=44:
            targets=[np.array([.2,0,0]),np.array([.2,.2,0]),np.array([.2,.2,.1]),np.zeros(3)]
            k=min(3,int((elapsed-44)/10));s=smoothstep(((elapsed-44)%10)/3)
            start=np.zeros(3) if k==0 else targets[k-1]
            target+=start+(targets[k]-start)*s if elapsed<84 else np.zeros(3)
            phase=f'track{k}' if elapsed<84 else 'arms'
        if self.scenario=='manual' and self.external_target is not None:target=self.external_target
        self.target=target
        error=np.linalg.norm(tp-target);angle=Rotation.from_matrix(tr).magnitude()
        self.arms.ready=elapsed>14 and error<.10 and angle<math.radians(3)
        if self.scenario=='acceptance' and elapsed>87 and self.replay_index<2:
            if self.arms.ready and not self.arms.sequence and t>self.arms.busy_until+5:
                self.arms.replay('left' if self.replay_index==0 else 'right');self.replay_index+=1
        if phase=='arms':phase='arms_'+('left' if self.replay_index<=1 else 'right')
        desired=self.model.control(p,v,rot,omega,target)
        if self.scenario in ('freefall','force_x','force_y','force_z') and elapsed>=14 and self.last_momentum is not None:
            if self.force_probe is None:self.force_probe=(t,self.last_momentum.copy())
            if t-self.force_probe[0]>.1:self.probe_samples.append([self.last_momentum_time,*self.last_momentum])
            force=np.zeros(3)
            if self.scenario!='freefall':
                force[2]=self.model.mass*9.81;force['xyz'.index(self.scenario[-1])]+=1.
            body_force=rot.T@force
            desired=np.r_[body_force,np.cross(self.model.com,body_force)]
            if t-self.force_probe[0]>=.35:
                # Fit after the transport transition, using sensor timestamps.
                samples=np.unique(np.array(self.probe_samples),axis=0)
                measured=np.polyfit(samples[:,0]-samples[0,0],samples[:,1:],1)[0]
                expected=force-np.array([0.,0.,self.model.mass*9.81])
                result=dict(measured_force=measured.tolist(),expected_force=expected.tolist(),
                            passed=bool(np.linalg.norm(measured-expected)<.1))
                (self.out/'diagnostic.json').write_text(json.dumps(result,indent=2));self.done=True
        actual=np.array([self.arms.q.get(f'unit_{u}_{a}_joint',0.) for u in 'abc' for a in ('outer','inner')])
        dt=.01 if self.last_t is None else np.clip(t-self.last_t,.001,.03);self.last_t=t
        if self.ideal:
            msg=Wrench();msg.force.x,msg.force.y,msg.force.z=desired[:3].tolist();msg.torque.x,msg.torque.y,msg.torque.z=desired[3:].tolist()
            self.ideal.publish(msg);z=np.zeros(12);residual=np.zeros(6)
        else:
            try:z,residual=self.alloc.solve(desired,dt);self.failures=0
            except RuntimeError:
                self.failures+=1
                if self.failures>=3:raise
                return
            motors=ActuatorMotors(timestamp=int(t*1e6),timestamp_sample=self.odom.timestamp_sample)
            # THR_MDL_FAC=0, EC_MIN=0: PX4 maps [0,1] linearly to rotor rad/s.
            motors.control=np.r_[np.sqrt(np.clip(z[:6]/self.model.max_thrust,0,1)),np.full(6,np.nan)].astype(float).tolist()
            servos=ActuatorServos(timestamp=int(t*1e6),timestamp_sample=self.odom.timestamp_sample)
            servos.control=np.r_[z[6:]/self.model.cfg['tilt_limit'],np.full(2,np.nan)].astype(float).tolist()
            self.motors.publish(motors);self.servos.publish(servos)
        saturated=bool(np.any(z[:6]>.99*self.model.max_thrust) or np.any(abs(z[6:])>self.model.cfg['tilt_limit']-.01)
                       or np.any(abs(self.efforts)>.99*self.model.cfg['arm_effort']) or np.any(abs(self.tilt_efforts)>.99*self.model.cfg['tilt_effort']))
        q=[self.arms.q[n] for n in sum(ARM_NAMES_LIST,[])]
        actual_f=self.model.kf*self.rotor_speeds**2
        self.csv.writerow([elapsed,phase,*tp,*target,*(tp-target),angle,np.linalg.norm(p-tp),np.linalg.norm(residual[:3]),np.linalg.norm(residual[3:]),int(saturated),*q,*self.efforts,*z,Rotation.from_matrix(rot.T@tr).magnitude(),*actual,*actual_f,*self.arm_reference,*self.tilt_efforts])
        self.diagnostics.publish(Float32MultiArray(data=[*desired.tolist(),*residual.tolist(),*z.tolist()]))
        if t-self.last_log>2:
            self.status.publish(String(data=f'{phase}: position_error={error:.3f} attitude_deg={math.degrees(angle):.2f} ready={self.arms.ready}'))
            self.get_logger().info(f'{elapsed:.1f}s {phase} p={np.round(tp,3)} error={error:.3f} angle={math.degrees(angle):.2f}');self.file.flush();self.last_log=t
        if elapsed>8 and (error>1.5 or angle>math.radians(60)):self.abort('Flight departed test envelope')
        if self.scenario=='hover' and elapsed>=44:self.done=True
        if self.scenario=='acceptance' and self.replay_index==2 and not self.arms.sequence and t>self.arms.busy_until+6:self.done=True
        if self.scenario=='acceptance' and elapsed>150:self.abort('Acceptance sequence did not finish in 150 seconds')

ARM_NAMES_LIST=[[f'{s}_joint_{i}' for i in range(1,6)] for s in ('left','right')]

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--scenario',default='manual')
    args=parser.parse_args();rclpy.init();node=Runtime(args.output,args.scenario)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node,timeout_sec=.1)
            # Keep process liveness separate from paused simulation time.
            wall=time.monotonic()
            if wall-node.last_heartbeat>.1:
                (node.out/'controller.heartbeat').touch();node.last_heartbeat=wall
            if node.begin is None and node.mode!='physics' and time.monotonic()-node.wall_start>150:node.abort('Startup timed out; inspect PX4/controller logs')
    except KeyboardInterrupt:pass
    finally:node.file.close();node.destroy_node();rclpy.try_shutdown()
    if node.error:raise SystemExit(1)

if __name__=='__main__':main()
