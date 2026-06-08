#!/usr/bin/env python3
"""Крупный план: объезд препятствия трактором + общий вид (две фигуры)."""
import re, math, time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path

LATCH=QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,reliability=QoSReliabilityPolicy.RELIABLE,
                 history=QoSHistoryPolicy.KEEP_LAST,depth=1)
class Grab(Node):
    def __init__(s):
        super().__init__('obs_plot'); s.plan=None; s.act=None
        s.create_subscription(Path,'/coverage_path',lambda m:setattr(s,'plan',[(p.pose.position.x,p.pose.position.y) for p in m.poses]),LATCH)
        s.create_subscription(Path,'/actual_trajectory',lambda m:setattr(s,'act',[(p.pose.position.x,p.pose.position.y) for p in m.poses]),10)

def obstacles(p):
    t=open(p).read(); o=[]
    for m in re.finditer(r'<model name=[\'"]([^\'"]+)[\'"]>(.*?)</model>',t,re.S):
        nm,b=m.group(1),m.group(2)
        if not re.search(r'obst|pillar|cylinder|box|barrel|cone',nm,re.I): continue
        pm=re.search(r'<pose>\s*([-\d.]+)\s+([-\d.]+)',b)
        if pm: o.append((float(pm.group(1)),float(pm.group(2))))
    return o

rclpy.init(); n=Grab(); t0=time.time()
while rclpy.ok() and (n.plan is None or n.act is None) and time.time()-t0<8:
    rclpy.spin_once(n,timeout_sec=0.2)
plan=n.plan or []; act=n.act or []; obs=obstacles('/home/dan/ros2-agv-tractor/src/agv_gazebo/worlds/agricultural_field.world')
rclpy.shutdown()

# найти препятствие, к которому трактор реально подъезжал ближе всего
def mind(o):
    return min((math.hypot(o[0]-x,o[1]-y) for x,y in act),default=99)
obs_near=sorted(obs,key=mind)
target=obs_near[0] if obs_near else (0,0)
print('ближайшее к траектории препятствие:',target,'мин.расст=',round(mind(target),2))

fig,ax=plt.subplots(figsize=(7,7))
ox,oy=target
R=6
# план
if plan:
    ax.plot(*zip(*plan),'-',color='#1f9e1f',lw=1.5,label='Плановый маршрут',zorder=2)
# факт
if act:
    ax.plot(*zip(*act),'-',color='#1f4fd8',lw=2.2,label='Траектория трактора',zorder=3)
# препятствия в зоне
for i,(x,y) in enumerate(obs):
    if abs(x-ox)<R+2 and abs(y-oy)<R+2:
        ax.add_patch(Circle((x,y),0.6,color='#b00000',zorder=4,label='Препятствие' if i==0 else None))
        ax.add_patch(Circle((x,y),0.6+1.5,color='#b00000',fill=False,ls='--',lw=1,alpha=0.6,zorder=4))
ax.set_xlim(ox-R,ox+R); ax.set_ylim(oy-R,oy+R)
ax.set_aspect('equal'); ax.grid(True,ls=':',alpha=0.5)
ax.set_xlabel('X, м'); ax.set_ylabel('Y, м')
ax.set_title('Объезд препятствия трактором (крупный план)\nпунктир — радиус безопасности (инфляция costmap)',fontsize=10)
ax.legend(loc='upper right',fontsize=9,framealpha=0.95)
plt.tight_layout(); plt.savefig('/tmp/fig_obstacle.png',dpi=160)
print('сохранено: /tmp/fig_obstacle.png')
