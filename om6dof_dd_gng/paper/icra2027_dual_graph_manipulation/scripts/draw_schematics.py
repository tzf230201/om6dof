#!/usr/bin/env python3
"""Vector schematics, not robot observations or experimental trajectories."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/icra2027_dual_graph_mpl')
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle
import numpy as np

OUT = Path(__file__).resolve().parents[1] / 'figures'
OUT.mkdir(exist_ok=True)
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
                     'pdf.fonttype': 42, 'ps.fonttype': 42})
BLUE, GREEN, ORANGE, DARK = '#2463a0', '#18826c', '#c5791b', '#263544'

def box(ax, xy, size, title, detail, color):
    x,y=xy; w,h=size
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.02,rounding_size=0.05',
                               facecolor=color+'12',edgecolor=color,linewidth=1.1))
    ax.text(x+w/2,y+h*.73,title,ha='center',va='center',weight='bold',color=color,fontsize=8)
    ax.text(x+w/2,y+h*.32,detail,ha='center',va='center',fontsize=7.4,color=DARK,linespacing=1.35)

def arrow(ax,a,b,color=DARK,style='-'):
    ax.annotate('',xy=b,xytext=a,arrowprops=dict(arrowstyle='-|>',lw=1.1,color=color,linestyle=style))

fig,ax=plt.subplots(figsize=(7.15,2.55))
ax.set(xlim=(0,10),ylim=(0,3.6)); ax.axis('off')
box(ax,(.1,2.13),(2.2,1.13),'RGB-D + YOLOX','Depth-supported labels\nTimestamp / pose gates',BLUE)
box(ax,(3.05,2.13),(2.7,1.13),'Semantic environment graph',r'$G_E$: nodes, edges, classes'+'\nObserved cluster centre $c_k$',BLUE)
box(ax,(.1,.25),(2.2,1.13),'Recorded IK witnesses','Green rows with joint vectors\nCurrent-model FK / validity',GREEN)
box(ax,(3.05,.25),(2.7,1.13),'Robot configuration graph',r'$G_R$: $(q_i,p_i,R_i)$'+'\nCandidate joint-space edges',GREEN)
box(ax,(6.65,1.15),(3.13,1.15),'Pre-grasp query','Centre + orientation + stand-off\nGraph search / sampled FCL',ORANGE)
arrow(ax,(2.33,2.7),(3.03,2.7),BLUE);arrow(ax,(2.33,.82),(3.03,.82),GREEN)
arrow(ax,(5.77,2.70),(6.63,2.09),BLUE);arrow(ax,(5.77,.82),(6.63,1.39),GREEN)
ax.text(4.4,1.76,'Geometry retained for collision',ha='center',fontsize=7.2,color=DARK)
arrow(ax,(8.20,1.13),(8.20,.66),ORANGE)
ax.text(8.20,.39,'Preview / explicit execution interface',ha='center',fontsize=7.6,color=DARK)
ax.text(.12,3.48,'SCHEMATIC  |  Two representations; one target-conditioned query',fontsize=8,color=DARK)
fig.subplots_adjust(left=0,right=1,bottom=0,top=1)
fig.savefig(OUT/'architecture.pdf');fig.savefig(OUT/'architecture.png',dpi=240);plt.close(fig)

fig,axes=plt.subplots(1,2,figsize=(7.15,2.7))
for ax in axes:
    ax.set_aspect('equal');ax.set(xlim=(-1.12,1.28),ylim=(-.38,1.20));ax.axis('off')
ax=axes[0]
pts=np.array([[-.25,-.21],[-.27,.03],[-.26,.24],[-.27,.48],[-.20,.76],
              [.22,-.19],[.28,.11],[.27,.45],[.18,.88],[.12,.83],[.05,.85],[-.09,.84]])
lo,hi=pts.min(axis=0),pts.max(axis=0)
ax.add_patch(Rectangle(lo,*(hi-lo),fc=BLUE+'12',ec=BLUE,lw=1))
for p in pts:ax.add_patch(Circle(p,.028,color=BLUE))
c=(lo+hi)*.5; ax.plot(*c,'o',color=ORANGE,ms=8)
ax.plot([c[0]-.10,c[0]+.10],[c[1],c[1]],color=ORANGE,lw=1)
ax.plot([c[0],c[0]],[c[1]-.10,c[1]+.10],color=ORANGE,lw=1)
ax.annotate('AABB midpoint $c_k$',xy=c,xytext=(.49,.42),fontsize=8,
            arrowprops=dict(arrowstyle='-',color=DARK),color=DARK)
ax.annotate('Observed\nsurface nodes',xy=pts[3],xytext=(-1.08,.50),fontsize=8,
            arrowprops=dict(arrowstyle='-',color=DARK),color=DARK)
ax.text(-1.08,1.08,'(a) Reference from cluster extent',weight='bold',fontsize=8.4)
ax.text(-1.08,-.36,'Illustration: midpoint is not a measured surface point.',fontsize=7)
ax=axes[1];c=np.array([.18,.24]); side=np.array([-.65,.24]); top=np.array([.18,.90])
ax.add_patch(Rectangle((-.08,-.25),.53,.84,fc=BLUE+'12',ec=BLUE,lw=1))
ax.plot(*c,'o',color=ORANGE,ms=8);ax.text(.31,.19,'$c_k$',fontsize=10)
ax.plot(*side,'s',color=GREEN,ms=6);arrow(ax,side,c,GREEN)
ax.plot(*top,'s',color=ORANGE,ms=6);arrow(ax,top,c,ORANGE)
ax.text(-1.05,.01,r'Side: $R_i a_T$ faces $c_k$',fontsize=7.5,color=GREEN)
ax.text(.37,.82,'Top: also faces $c_k$',fontsize=7.5,color=ORANGE)
ax.text(.37,.63,'Local-axis test\nalone admits both',fontsize=7.5,color=DARK)
ax.text(-1.05,1.08,'(b) Local axis is not a world direction',weight='bold',fontsize=8.4)
ax.text(-1.05,-.36,'A horizontal constraint is an additional requirement.',fontsize=7)
fig.subplots_adjust(left=.015,right=.985,bottom=.035,top=.97,wspace=.16)
fig.savefig(OUT/'target_geometry.pdf');fig.savefig(OUT/'target_geometry.png',dpi=240);plt.close(fig)
print('Wrote architecture and target-geometry schematics.')
