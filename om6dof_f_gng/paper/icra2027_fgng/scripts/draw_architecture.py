#!/usr/bin/env python3
"""Vector architecture schematic. All geometry here is illustrative, not data."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Circle
import numpy as np

OUT = Path(__file__).resolve().parents[1] / 'figures'
OUT.mkdir(parents=True, exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42,'ps.fonttype':42})
fig, ax = plt.subplots(figsize=(7.15, 2.40))
fig.subplots_adjust(0,0,1,1)
ax.set_xlim(0, 10); ax.set_ylim(0, 3.35); ax.axis('off')
ink='#203141'; teal='#087F8C'; orange='#D77644'; muted='#617184'
starts=[.08,2.61,5.14,7.67]
width=2.25
for i,x in enumerate(starts):
    ax.add_patch(FancyBboxPatch((x,.40),width,2.42,boxstyle='round,pad=0.025,rounding_size=0.10',facecolor='#F4F7FA' if i<3 else '#EDF7F5',edgecolor='#D7DFE5',lw=.7))
    ax.text(x+.13,2.61,f'0{i+1}',fontsize=7,color=teal,fontweight='bold')
    if i<3:
        ax.add_patch(FancyArrowPatch((x+width+.035,1.60),(starts[i+1]-.06,1.60),arrowstyle='-|>',mutation_scale=10,lw=1,color=muted))
head=['REGISTER','RETAIN','REPLAY','ALLOCATE']
sub=['Depth + supplied pose','One sample per world voxel','Bounded support samples','Foveated graph, $|V|\\leq B$']
for x,h,s in zip(starts,head,sub):
    ax.text(x+.43,2.61,h,fontsize=8,fontweight='bold',color=ink)
    ax.text(x+.13,2.30,s,fontsize=6.8,color=muted)
# Depth rays and camera/world transformation
x=starts[0]
ax.plot([x+.18,x+.18,x+.58,x+.58,x+.18],[1.43,1.87,1.87,1.43,1.43],color=muted,lw=1)
ax.plot([x+.58,x+.84,x+.84,x+.58],[1.48,1.29,2.02,1.82],color=muted,lw=1)
for yy in np.linspace(1.15,2.04,6):
    ax.plot([x+.84,x+1.78],[1.65,yy],color=teal,lw=.65,alpha=.5)
    ax.scatter(x+1.78,yy,s=8,c=teal,zorder=3)
ax.text(x+1.17,.99,'$T_{wc}(t)$',fontsize=10,color=ink)
ax.text(x+.13,.65,'$p_w=R_{wc}p_c+t_{wc}$',fontsize=8,color=ink)
# Fixed cells/retention
x=starts[1]
for j in range(7):
    ax.plot([x+.26+j*.24]*2,[1.02,2.07],color='#D5DEE5',lw=.65)
for j in range(5):
    ax.plot([x+.26,x+1.70],[1.02+j*.2625]*2,color='#D5DEE5',lw=.65)
pts=np.array([[.38,1.17],[.60,1.42],[.82,1.69],[1.08,1.71],[1.31,1.96],[1.55,1.69],[1.57,1.44],[1.31,1.18]])
ax.scatter(x+pts[:,0],pts[:,1],s=16,color=teal,zorder=3)
ax.text(x+1.78,1.58,'$K$',fontsize=13,color=ink)
ax.text(x+.13,.65,'$K$ lowest-priority voxels',fontsize=7.7,color=ink)
# Replay samples distributed balanced
x=starts[2]
for i in range(15):
    row,col=divmod(i,5)
    ax.scatter(x+.40+col*.33,1.22+row*.30,s=16,color=teal if i%3==0 else '#BFCED7',zorder=3)
ax.add_patch(FancyArrowPatch((x+.53,1.94),(x+1.64,1.94),connectionstyle='arc3,rad=-.35',arrowstyle='-|>',mutation_scale=8,color=muted,lw=.8))
ax.text(x+.13,.65,'One vote per replayed voxel',fontsize=7.2,color=ink)
# Graph illustrative with foveal density
x=starts[3]
gp=np.array([[.25,1.19],[.29,1.81],[.78,1.95],[.90,1.22],[1.35,1.30],[1.65,1.35],[1.73,1.64],[1.57,1.83],[1.33,1.74],[1.47,1.54],[1.99,1.22]])
for a,b in [(0,1),(1,2),(0,3),(2,3),(2,8),(3,4),(4,5),(4,9),(5,6),(5,9),(6,7),(6,9),(7,8),(7,9),(8,9),(3,8),(5,10)]:
    ax.plot(x+gp[[a,b],0],gp[[a,b],1],lw=.65,color=teal,alpha=.8)
ax.add_patch(Circle((x+1.52,1.57),.39,facecolor=orange,alpha=.10,edgecolor=orange,lw=1))
ax.scatter(x+gp[:,0],gp[:,1],s=13,color=teal,zorder=4)
ax.scatter(x+1.52,1.57,marker='+',s=34,c=orange,lw=1.2,zorder=5)
ax.text(x+.13,.65,'Split / merge + hysteresis',fontsize=7.7,color=ink)
# attention feedback above last cards
ax.annotate('',xy=(9.0,2.90),xytext=(6.85,2.90),arrowprops={'arrowstyle':'-|>','color':orange,'lw':1.1})
ax.text(7.93,3.02,'world target $q_t$ → density demand',ha='center',fontsize=7.5,color=orange)
ax.text(.10,.12,'Known-pose, static-scene setting',fontsize=7,color=muted)
ax.text(9.9,.12,'Architecture schematic',ha='right',fontsize=6.5,color=muted)
for ext in ['pdf','svg','png']:
    fig.savefig(OUT/f'architecture.{ext}',dpi=300,facecolor='white')
plt.close(fig)
