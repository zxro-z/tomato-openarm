#!/usr/bin/python3
"""Visualization-only calibration of provisional T_tcp_cube; no MoveIt action client."""
from __future__ import annotations
import math
from pathlib import Path
import rclpy, yaml
from geometry_msgs.msg import Point, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool

def qmul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return (aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)

def rpy_q(r,p,y):
    cr,sr=math.cos(r/2),math.sin(r/2); cp,sp=math.cos(p/2),math.sin(p/2); cy,sy=math.cos(y/2),math.sin(y/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

def rotate(v,q):
    x,y,z,w=q; vx,vy,vz=v; tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)

class Calibration(Node):
    def __init__(self):
        super().__init__('baseline_a_grasp_transform_calibration')
        for name, value in [('config_file',''),('grasp_debug_dx',0.),('grasp_debug_dy',0.),('grasp_debug_dz',0.),('grasp_debug_roll_deg',0.),('grasp_debug_pitch_deg',0.),('grasp_debug_yaw_deg',0.)]: self.declare_parameter(name,value)
        with Path(self.get_parameter('config_file').value).open() as f: cfg=yaml.safe_load(f)
        self.grasp=cfg['baseline_a']['grasp']; self.tf=Buffer(); self.listener=TransformListener(self.tf,self); self.runtime_pose=None
        self.pub=self.create_publisher(MarkerArray,'baseline_a/grasp_transform_calibration',1); self.reported=False; self.create_timer(.5,self.publish)
        self.create_subscription(PoseStamped,'baseline_a/contact_grasp_tcp_pose',self.contact_pose,1)
        self.home_ready=False
        ready_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'baseline_a/grasp_calibration_ready',self.ready,ready_qos)
    def ready(self,message):
        self.home_ready=message.data
        if self.home_ready: self.get_logger().info('Calibration markers enabled after verified HOME, closed gripper, and attached cube')
    def contact_pose(self,message): self.runtime_pose=message.pose
    def values(self):
        if self.runtime_pose is not None:
            p,q=self.runtime_pose.position,self.runtime_pose.orientation
            return [p.x,p.y,p.z],(q.x,q.y,q.z,q.w)
        xyz=list(self.grasp['tcp_to_cube_xyz']); xyz=[xyz[i]+self.get_parameter(n).value for i,n in enumerate(('grasp_debug_dx','grasp_debug_dy','grasp_debug_dz'))]
        qd=rpy_q(*[math.radians(self.get_parameter(n).value) for n in ('grasp_debug_roll_deg','grasp_debug_pitch_deg','grasp_debug_yaw_deg')])
        return xyz,qmul(tuple(self.grasp['tcp_to_cube_quat_xyzw']),qd)
    def cube(self,xyz,q):
        m=Marker();m.header.frame_id='openarm_left_hand_tcp';m.ns='grasp_calibration';m.id=0;m.type=Marker.CUBE;m.action=Marker.ADD
        m.pose.position.x,m.pose.position.y,m.pose.position.z=xyz;m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
        m.scale.x=m.scale.y=m.scale.z=self.grasp['cube_size_m'];m.color.r,m.color.g,m.color.b,m.color.a=1.,.05,.05,.55;return m
    def sphere(self,ident,frame,xyz,color,scale=.014):
        m=Marker();m.header.frame_id=frame;m.ns='grasp_calibration';m.id=ident;m.type=Marker.SPHERE;m.action=Marker.ADD;m.pose.position.x,m.pose.position.y,m.pose.position.z=xyz;m.pose.orientation.w=1.;m.scale.x=m.scale.y=m.scale.z=scale;m.color.r,m.color.g,m.color.b,m.color.a=*color,1.;return m
    def arrow(self,ident,frame,a,b,color):
        m=Marker();m.header.frame_id=frame;m.ns='grasp_calibration';m.id=ident;m.type=Marker.ARROW;m.action=Marker.ADD
        m.points=[Point(x=float(a[0]),y=float(a[1]),z=float(a[2])),Point(x=float(b[0]),y=float(b[1]),z=float(b[2]))]
        m.scale.x,m.scale.y=.005,.012;m.color.r,m.color.g,m.color.b,m.color.a=*color,1.;return m
    def publish(self):
        if not self.home_ready: return
        xyz,q=self.values()
        # This is the collision-mesh-derived basic-finger center stored in
        # config, not the pre-sensor-removal tactile-pad proxy.
        proxy=tuple(self.grasp['tcp_to_cube_xyz']); offset=tuple(xyz[i]-proxy[i] for i in range(3)); norm=math.sqrt(sum(x*x for x in offset))
        output=MarkerArray();output.markers=[self.cube(xyz,q),self.sphere(1,'openarm_left_hand_tcp',proxy,(.1,1.,.1)),self.arrow(2,'openarm_left_hand_tcp',(0,-.025,proxy[2]),(0,.025,proxy[2]),(1.,1.,0.)),self.arrow(3,'openarm_left_hand_tcp',xyz,(xyz[0],xyz[1],xyz[2]+.08),(0.,.4,1.))]
        self.pub.publish(output)
        if not self.reported:
            self.reported=True; warn=[]
            if xyz[2] < -.09: warn.append('cube center is wrist-side of basic-finger collision extent')
            if abs(xyz[1])>.02: warn.append('cube center is laterally offset from finger midpoint')
            if norm>.025: warn.append('cube center differs from finger-midpoint proxy by >25 mm')
            self.get_logger().info(f'A CURRENT BASE T_TCP_CUBE xyz={self.grasp["tcp_to_cube_xyz"]} q={self.grasp["tcp_to_cube_quat_xyzw"]}')
            self.get_logger().info(f'B DEBUG-ADJUSTED T_TCP_CUBE xyz={xyz} q={q}')
            self.get_logger().info(f'C GRASP CENTER PROXY (TCP)={proxy}; D CUBE-vs-PROXY offset={offset} norm={norm:.4f}m')
            self.get_logger().info(f'E HEURISTIC WARNINGS: {warn or "none"}; F SUGGESTED BETTER T_TCP_CUBE xyz={proxy} q={self.grasp["tcp_to_cube_quat_xyzw"]}')
            self.get_logger().info('G RVIZ VERDICT: SENSORLESS_BASIC_FINGER_GRASP_REFERENCE_ACTIVE')
def main():
 rclpy.init();n=Calibration()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
