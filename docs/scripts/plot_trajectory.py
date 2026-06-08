#!/usr/bin/env python3
"""Строим красивый график: план покрытия + реально пройденная траектория."""
import re, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path

LATCH = QoSProfile(durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                   reliability=QoSReliabilityPolicy.RELIABLE,
                   history=QoSHistoryPolicy.KEEP_LAST, depth=1)

class Grab(Node):
    def __init__(self):
        super().__init__('traj_plot')
        self.plan=None; self.actual=None
        self.create_subscription(Path,'/coverage_path',self._p,LATCH)
        self.create_subscription(Path,'/actual_trajectory',self._a,10)
    def _p(self,m): self.plan=[(p.pose.position.x,p.pose.position.y) for p in m.poses]
    def _a(self,m): self.actual=[(p.pose.position.x,p.pose.position.y) for p in m.poses]

def parse_obstacles(path):
    """Достаём позиции моделей-препятствий из .world (имя + <pose>)."""
    txt=open(path).read()
    obs=[]
    for m in re.finditer(r'<model name=[\'"]([^\'"]+)[\'"]>(.*?)</model>', txt, re.S):
        name,body=m.group(1),m.group(2)
        if not re.search(r'obst|pillar|cylinder|box|столб|barrel|cone', name, re.I):
            continue
        pm=re.search(r'<pose>\s*([-\d.]+)\s+([-\d.]+)', body)
        if pm: obs.append((float(pm.group(1)),float(pm.group(2)),name))
    return obs

def main():
    rclpy.init()
    n=Grab()
    import time
    t0=time.time()
    while rclpy.ok() and (n.plan is None or n.actual is None) and time.time()-t0<8:
        rclpy.spin_once(n,timeout_sec=0.2)
    plan=n.plan or []; actual=n.actual or []
    obs=parse_obstacles('/home/dan/ros2-agv-tractor/src/agv_gazebo/worlds/agricultural_field.world')
    rclpy.shutdown()

    fig,ax=plt.subplots(figsize=(9,9))
    # граница поля 36x36
    ax.add_patch(Rectangle((-18,-18),36,36,fill=False,ec='#caa000',lw=2,ls='-',label='Граница поля 36×36 м'))
    ax.add_patch(Rectangle((-16,-16),32,32,fill=False,ec='#e08000',lw=1,ls='--',label='Зона обработки'))
    # план (змейка)
    if plan:
        px,py=zip(*plan)
        ax.plot(px,py,'-',color='#1f9e1f',lw=1.6,label='План покрытия (бустрофедон)',zorder=2)
        ax.plot(px,py,'.',color='#1f9e1f',ms=4,zorder=2)
    # факт
    if actual:
        ax.plot(*zip(*actual),'-',color='#1f4fd8',lw=1.4,alpha=0.9,label='Пройденная траектория',zorder=3)
        ax.plot(actual[0][0],actual[0][1],'o',color='black',ms=9,label='Старт',zorder=5)
        ax.plot(actual[-1][0],actual[-1][1],'^',color='red',ms=12,label='Трактор (текущее положение)',zorder=5)
    # препятствия
    for i,(ox,oy,nm) in enumerate(obs):
        ax.add_patch(Circle((ox,oy),0.6,color='#b00000',zorder=4,
                            label='Препятствия' if i==0 else None))

    ax.set_xlabel('X, м'); ax.set_ylabel('Y, м')
    ax.set_title('Покрытие поля беспилотным трактором: план и фактическая траектория\n'
                 '(локализация — слияние GPS + IMU + одометрия, EKF)',fontsize=11)
    ax.set_aspect('equal'); ax.grid(True,ls=':',alpha=0.5)
    ax.set_xlim(-20,20); ax.set_ylim(-20,20)
    ax.legend(loc='upper right',fontsize=8,framealpha=0.95)
    plt.tight_layout()
    plt.savefig('/tmp/fig_coverage.png',dpi=160)
    print(f'plan={len(plan)} точек, actual={len(actual)} точек, препятствий={len(obs)}')
    print('сохранено: /tmp/fig_coverage.png')

if __name__=='__main__':
    main()
