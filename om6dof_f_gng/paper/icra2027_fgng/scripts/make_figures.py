#!/usr/bin/env python3
"""Rebuild publication figures from actual, prespecified benchmark outputs."""
from pathlib import Path
import csv, json, os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Circle

PAPER=Path(__file__).resolve().parents[1]
EXPERIMENT=Path(os.environ.get(
    'FGNG_EXPERIMENT_DIR',
    PAPER.parents[3]/'TopoVLA/experiments/fgng_world')).resolve()
if not EXPERIMENT.is_dir():
    raise FileNotFoundError(
        f'FGNG experiment directory not found: {EXPERIMENT}. '
        'Set FGNG_EXPERIMENT_DIR to the fgng_world experiment directory.')
RESULTS=EXPERIMENT/'results'
DATA=EXPERIMENT/'data'
OUT=PAPER/'figures'
OUT.mkdir(exist_ok=True)
COLORS={'camera':'#9B9DA5','world':'#CF754D','memory':'#067F87','memory_uniform':'#44617C','memory_dbl':'#8563A7','memory_fps':'#BA964B','voxel_fps':'#BA964B'}
LABELS={'camera':'Unregistered F-GNG','world':'World-only F-GNG','memory':'WA-FGNG','memory_uniform':'Replay, uniform attention','memory_dbl':'Replay + DBL-GNG','memory_fps':'Replay + FPS','voxel_fps':'Voxel + FPS'}
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.titlesize':8.5,'axes.labelsize':8,'xtick.labelsize':7,'ytick.labelsize':7,'legend.fontsize':7,'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#65717D','axes.labelcolor':'#263542','text.color':'#263542','xtick.color':'#485767','ytick.color':'#485767','axes.linewidth':.65,'grid.linewidth':.5,'grid.alpha':.4})

def read_rows(path):
    rows=[]
    with open(path) as f:
        for raw in csv.DictReader(f):
            row={}
            for k,v in raw.items():
                try: row[k]=float(v)
                except ValueError: row[k]=v
            rows.append(row)
    return rows

def save(fig,name):
    for ext in ['pdf','svg','png']:
        fig.savefig(OUT/f'{name}.{ext}',dpi=250,bbox_inches='tight',pad_inches=.045,facecolor='white')
    plt.close(fig)

def select(rows,**kwargs):
    return [r for r in rows if all(r[k]==v for k,v in kwargs.items())]

def quant_fig(rows):
    fig,axs=plt.subplots(2,2,figsize=(7.10,3.80),sharex=True)
    handles=[]
    methods=['world','memory_uniform','memory','memory_fps']
    methods=[m for m in methods if any(r['method']==m for r in rows)]
    for col,seq in enumerate(['freiburg1_desk','freiburg1_xyz']):
        for row,(metric,label) in enumerate([('history_outside_frustum_mean_m','Off-screen error (cm)'),('seen_roi_mean_m','Focal error (cm)')]):
            ax=axs[row,col]
            for method in methods:
                if row==1 and method=='world':
                    continue
                ss=select(rows,sequence=seq,method=method,budget=256.)
                frames=sorted({r['frame'] for r in ss if r['frame']>=20})
                means=[];stds=[]
                for frame in frames:
                    vals=np.array([r[metric]*100 for r in ss if r['frame']==frame])
                    vals=vals[np.isfinite(vals)]
                    means.append(vals.mean() if len(vals) else np.nan)
                    stds.append(vals.std(ddof=1) if len(vals)>1 else 0.)
                y=np.array(means);sd=np.array(stds)
                line,=ax.plot(frames,y,color=COLORS[method],lw=1.6 if method=='memory' else 1.0,label=LABELS[method],zorder=4 if method=='memory' else 2)
                ax.fill_between(frames,y-sd,y+sd,color=COLORS[method],alpha=.10,lw=0)
                if row==col==0: handles.append(line)
            for t in [40,80]:ax.axvline(t,color='#9AA5AE',ls=(0,(3,3)),lw=.65)
            ax.grid(axis='y');ax.set_ylabel(label);ax.set_xlim(20,119)
            ax.set_xticks([20,40,60,80,100,119]);ax.set_xticklabels(['20','40','60','80','100','119'])
            if row==0: ax.set_title(f'({chr(97+col)}) {seq.replace("freiburg1_", "fr1/")}',loc='left',fontweight='bold')
            else:ax.set_xlabel('Selected frame index')
            if row==0:
                lo,hi=ax.get_ylim()
                for xx,txt in [(29,'A'),(60,'B'),(100,'A')]:ax.text(xx,hi-.03*(hi-lo),txt,ha='center',va='top',fontsize=7,color='#7E6552',bbox={'facecolor':'white','edgecolor':'none','alpha':.75,'pad':1})
    fig.legend(handles,[h.get_label() for h in handles],loc='lower center',ncol=2,frameon=False,bbox_to_anchor=(.5,-.008),columnspacing=2.2)
    fig.subplots_adjust(hspace=.28,wspace=.30,bottom=.16,top=.94)
    save(fig,'temporal_results')

def tradeoff_fig(rows):
    fig,axs=plt.subplots(1,2,figsize=(7.10,2.47))
    methods=[m for m in ['memory','memory_uniform','memory_dbl','memory_fps'] if any(r['method']==m for r in rows)]
    for ax,seq in zip(axs,['freiburg1_desk','freiburg1_xyz']):
        for method in methods:
            ss=select(rows,sequence=seq,method=method)
            xx=[];yy=[]
            for budget in sorted({r['budget'] for r in ss}):
                q=[r for r in ss if r['budget']==budget]
                x=np.mean([r['update_p95_ms'] for r in q]);y=np.mean([r['seen_roi_mean_m']*100 for r in q])
                xx.append(x);yy.append(y)
                ax.scatter(x,y,s=24 if budget==128 else 45,marker='o' if budget==128 else 's',color=COLORS[method],edgecolor='white',lw=.4,zorder=3)
            ax.plot(xx,yy,color=COLORS[method],lw=.7,alpha=.7)
        ax.set_title(seq.replace('freiburg1_','fr1/'),loc='left',fontweight='bold')
        ax.set_xlabel('95th-percentile update time (ms)');ax.set_ylabel('Focal error (cm)');ax.grid(alpha=.22)
    handles=[plt.Line2D([0],[0],marker='s',ls='',color=COLORS[m],label=LABELS[m],markersize=4) for m in methods]
    fig.legend(handles=handles,loc='lower center',ncol=2,frameon=False,bbox_to_anchor=(.5,-.06))
    fig.subplots_adjust(bottom=.26,wspace=.27,top=.89)
    save(fig,'accuracy_cost')

def project(points,origin,basis):return (points-origin)@basis

def qualitative():
    # Fixed sequence/frame/budget/seed, selected before looking at outcomes.
    seq='freiburg1_desk';frame=119;budget=256;seed=1
    methods=['world','memory_uniform','memory','memory_fps']
    paths=[RESULTS/'snapshots'/f'{seq}_{m}_b{budget}_s{seed}_f{frame:03d}.npz' for m in methods]
    methods=[m for m,p in zip(methods,paths) if p.exists()]
    if not methods:return
    sample=np.load(RESULTS/'snapshots'/f'{seq}_{methods[0]}_b{budget}_s{seed}_f{frame:03d}.npz')
    ref=sample['reference'];origin=np.median(ref,axis=0)
    # Use the FIRST camera's image axes for a reproducible common world projection.
    prepared=np.load(DATA/seq/'prepared.npz')
    basis=prepared['poses'][0,:3,:2]
    rr=project(ref,origin,basis);center=project(sample['attention_center'][None],origin,basis)[0]
    low=np.percentile(rr,1,axis=0);high=np.percentile(rr,99,axis=0)
    fig,axs=plt.subplots(2,len(methods),figsize=(7.1,3.75),squeeze=False)
    for col,method in enumerate(methods):
        snap=np.load(RESULTS/'snapshots'/f'{seq}_{method}_b{budget}_s{seed}_f{frame:03d}.npz')
        nodes=snap['nodes'];nn=project(nodes[:,1:4],origin,basis)
        lookup={int(n[0]):nn[i] for i,n in enumerate(nodes)}
        segments=[np.array([lookup[int(a)],lookup[int(b)]]) for a,b in snap['edges'] if int(a) in lookup and int(b) in lookup] if snap['edges'].size else []
        for row in range(2):
            ax=axs[row,col]
            # Rasterize only the background so vector graph/text stays sharp.
            take=np.linspace(0,len(rr)-1,min(len(rr),18000)).astype(int)
            ax.scatter(rr[take,0],rr[take,1],s=.24,c='#AEBBC5',alpha=.35,rasterized=True,lw=0)
            if segments:ax.add_collection(LineCollection(segments,colors=COLORS[method],linewidths=.35,alpha=.58))
            ax.scatter(nn[:,0],nn[:,1],s=3.3 if row==0 else 7,c=COLORS[method],edgecolors='none',zorder=3)
            ax.add_patch(Circle(center,.30,fill=False,color='#D98A42',lw=.9,ls=(0,(3,2))))
            ax.scatter(*center,marker='+',c='#D98A42',s=35,lw=1,zorder=4)
            ax.set_aspect('equal');ax.set_xticks([]);ax.set_yticks([])
            for sp in ax.spines.values():sp.set_visible(False)
            if row==0:
                ax.set_xlim(low[0]-.08,high[0]+.08);ax.set_ylim(high[1]+.08,low[1]-.08)
                ax.set_title(LABELS[method].replace('Replay, uniform attention','Replay, uniform'),fontsize=7.5,fontweight='bold',pad=6)
            else:
                ax.set_xlim(center[0]-.48,center[0]+.48);ax.set_ylim(center[1]+.48,center[1]-.48)
                ax.plot([center[0]-.42,center[0]-.22],[center[1]+.41]*2,color='#3E4B54',lw=1.2)
                ax.text(center[0]-.32,center[1]+.37,'20 cm',ha='center',fontsize=6)
        axs[0,col].text(.02,.03,f'{len(nodes)} nodes',transform=axs[0,col].transAxes,fontsize=6.5,color=COLORS[method],bbox={'facecolor':'white','edgecolor':'none','alpha':.8,'pad':1})
    axs[0,0].text(-.07,.5,'Accumulated surface',transform=axs[0,0].transAxes,rotation=90,va='center',ha='right',fontsize=7)
    axs[1,0].text(-.07,.5,'World-anchor detail',transform=axs[1,0].transAxes,rotation=90,va='center',ha='right',fontsize=7)
    fig.subplots_adjust(wspace=.07,hspace=.13,top=.91,bottom=.02,left=.045,right=.995)
    save(fig,'qualitative_desk')

def observations():
    seq='freiburg1_desk';prepared=np.load(DATA/seq/'prepared.npz')
    from PIL import Image
    fig,axs=plt.subplots(1,3,figsize=(7.1,1.9))
    for ax,frame in zip(axs,[0,59,119]):
        path=DATA/seq/('depth_first.png' if frame==0 else f'depth_{frame:03d}.png')
        depth=np.asarray(Image.open(path),float)/5000
        depth[(depth<.35)|(depth>4.)]=np.nan
        im=ax.imshow(depth,cmap='viridis',vmin=.35,vmax=3.0)
        ax.set_title(f'fr1/desk · frame {frame}',loc='left',fontsize=8)
        ax.set_xticks([]);ax.set_yticks([])
    fig.subplots_adjust(wspace=.05,left=.01,right=.92,bottom=.03,top=.86)
    cb=fig.colorbar(im,cax=fig.add_axes([.945,.13,.014,.61]));cb.set_label('Depth (m)',fontsize=7);cb.ax.tick_params(labelsize=6)
    save(fig,'input_views')

if __name__=='__main__':
    rows=read_rows(RESULTS/'metrics.csv');summary=read_rows(RESULTS/'summary.csv')
    quant_fig(rows);tradeoff_fig(summary);qualitative();observations()
    try:
        result_source=str(RESULTS.relative_to(PAPER.parents[3]))
    except ValueError:
        result_source=str(RESULTS)
    (OUT/'provenance.json').write_text(json.dumps({'source':result_source,'script':'scripts/make_figures.py','selection':'fr1/desk, frame119, seed1, budget256 fixed before inspecting outcomes','uncertainty':'temporal bands = mean ± sample SD across 3 algorithm seeds, not scene-population confidence intervals','qualitative_projection':'first camera image x/y axes, held fixed for all methods; projected anchor disk illustrates focus, not a geometric segmentation'},indent=2)+'\n')
