#!/usr/bin/env python3
"""Generate numerical LaTeX from recorded run summaries without invented values."""
from pathlib import Path
import csv,json,hashlib,os
import numpy as np
P=Path(__file__).resolve().parents[1]
EXP=Path(os.environ.get(
    'FGNG_EXPERIMENT_DIR',
    P.parents[3]/'TopoVLA/experiments/fgng_world')).resolve()
if not EXP.is_dir():
    raise FileNotFoundError(
        f'FGNG experiment directory not found: {EXP}. '
        'Set FGNG_EXPERIMENT_DIR to the fgng_world experiment directory.')
R=EXP/'results'
rows=list(csv.DictReader((R/'summary.csv').open()))
methods=['camera','world','memory','memory_uniform','memory_dbl','memory_fps']
names={'camera':'Unregistered F-GNG','world':'World-only F-GNG','memory':r'\textbf{WA-FGNG}','memory_uniform':'Replay, uniform field','memory_dbl':'Replay + DBL-GNG','memory_fps':'Replay + FPS'}

def q(seq,m,b=256,source=rows):return [r for r in source if r['sequence']==seq and r['method']==m and int(r['budget'])==b]
def values(rs,key):return np.array([float(r[key]) for r in rs])
def mean(rs,key):return float(np.mean(values(rs,key)))
def fmt(rs,key,mult=100):
 x=values(rs,key)*mult
 if not np.all(np.isfinite(x)):return '--'
 return f'{x.mean():.2f} $\\pm$ {x.std(ddof=1):.2f}'
nums={}
lines=[r'\begin{table*}[t]',r'\centering',r'\caption{Primary known-pose results with node cap $B=256$. Errors are in cm; values are mean $\pm$ sample standard deviation of three per-seed means on the fixed evaluation grid. $N_f$ is mean final node count; $t_{95}$ is mean per-run 95th-percentile update time (ms). All memory-based methods use $K=8000$ retained voxels and at most $S=1500$ replay samples. Equal node caps do not imply equal actual node counts or equal total memory.}',r'\label{tab:main}',r'\small',r'\setlength{\tabcolsep}{5pt}',r'\begin{tabular}{llrrrrr}',r'\toprule',r'Sequence & Method & $N_f$ & All observed $\downarrow$ & Off-screen $\downarrow$ & Focal $\downarrow$ & $t_{95}$ \\',r'\midrule']
for seq,prefix in [('freiburg1_desk','Desk'),('freiburg1_xyz','Xyz')]:
 for m in methods:
  rs=q(seq,m)
  if len(rs)!=3:raise ValueError(f'Expected3seeds {seq} {m}')
  lines.append(f'{seq.replace("freiburg1_","fr1/")} & {names[m]} & {mean(rs,"actual_nodes_final"):.0f} & {fmt(rs,"seen_mean_m")} & {fmt(rs,"history_outside_frustum_mean_m")} & {fmt(rs,"seen_roi_mean_m")} & {mean(rs,"update_p95_ms"):.2f} \\\\')
  nums[f'{prefix}_{m}']={key:mean(rs,key) for key in ['seen_mean_m','history_outside_frustum_mean_m','seen_roi_mean_m','seen_periphery_mean_m','current_mean_m','unsupported_edge_fraction_5cm','update_p95_ms','actual_nodes_final']}
 mm=q(seq,'memory');uu=q(seq,'memory_uniform');ww=q(seq,'world')
 nums[f'{prefix}FocusGain']=100*(1-mean(mm,'seen_roi_mean_m')/mean(uu,'seen_roi_mean_m'))
 nums[f'{prefix}GlobalCost']=100*(mean(mm,'seen_mean_m')/mean(uu,'seen_mean_m')-1)
 nums[f'{prefix}HistoryGain']=100*(1-mean(mm,'history_outside_frustum_mean_m')/mean(ww,'history_outside_frustum_mean_m'))
 nums[f'{prefix}SeenGain']=100*(1-mean(mm,'seen_mean_m')/mean(ww,'seen_mean_m'))
 if seq=='freiburg1_desk':lines.append(r'\midrule')
lines.extend([r'\bottomrule',r'\end{tabular}',r'\end{table*}'])
(P/'tables.tex').write_text('\n'.join(lines)+'\n')
macro=[]
method_words={'camera':'Camera','world':'World','memory':'Memory','memory_uniform':'Uniform','memory_dbl':'Dbl','memory_fps':'Fps'}
metric_words={'seen_mean_m':('Seen',100),'history_outside_frustum_mean_m':('History',100),'seen_roi_mean_m':('Focus',100),'seen_periphery_mean_m':('Periphery',100),'current_mean_m':('Current',100),'unsupported_edge_fraction_5cm':('Edge',100),'update_p95_ms':('Time',1),'actual_nodes_final':('Nodes',1)}
for seq,prefix in [('freiburg1_desk','Desk'),('freiburg1_xyz','Xyz')]:
 for method,word in method_words.items():
  rs=q(seq,method)
  for metric,(ending,scale) in metric_words.items():
   value=mean(rs,metric)*scale
   if np.isfinite(value):
    formatted=f'{value:.0f}' if ending=='Nodes' else f'{value:.2f}'
    macro.append('\\newcommand{\\'+prefix+word+ending+'}{'+formatted+'}')
stress=EXP/'results_pose_stress/summary.csv'
if stress.exists():
 stress_rows=list(csv.DictReader(stress.open()))
 for seq,prefix in [('freiburg1_desk','Desk'),('freiburg1_xyz','Xyz')]:
  for sigma,word in [(0.,'Zero'),(.01,'One'),(.03,'Three')]:
   rs=[r for r in stress_rows if r['sequence']==seq and float(r['translation_sigma_m'])==sigma]
   if len(rs)!=3:raise ValueError('Pose-stress incomplete')
   for key,ending in [('seen_mean_m','Seen'),('seen_roi_mean_m','Focus')]:
    macro.append('\\newcommand{\\'+prefix+'Pose'+word+ending+'}{'+f'{mean(rs,key)*100:.2f}'+'}')

for key,value in nums.items():
 if isinstance(value,float):macro.append(f'\\newcommand{{\\{key}}}{{{value:.1f}}}')
(P/'numbers.tex').write_text('\n'.join(macro)+'\n')
(P/'results_numbers.json').write_text(json.dumps({'source_sha256':hashlib.sha256((R/'summary.csv').read_bytes()).hexdigest(),'aggregation':'primary summary.csv, three seeds,20uniform selected frames20:5:115','values':nums},indent=2)+'\n')
protocol=json.loads((R/'protocol.json').read_text())
(P/'protocol_details.tex').write_text(r'''We use node limits $B\in\{128,256\}$, seeds $\{1,2,3\}$, one learning epoch per selected frame, a voxel width of 3\,cm, memory cap $K=8000$, and replay limit $S=1500$. All methods receive the same current samples and learn from at most 1500 samples per update. FPS selects its output from that same replay batch. The graph uses $\mu=1$, $\kappa=1$, split/merge ratios 1.35/0.45, hysteresis 3, reallocation patience 8, and attention floor 0.15. Winner/neighbor learning rates are 0.5/0.01, error and insertion decays are both 0.5, and distributed initialization uses 10 regions. Thus the main evaluation comprises $2\times2\times3\times6=72$ runs, executed sequentially in native C++ on an NVIDIA Jetson AGX Orin CPU. CPU clocks and power modes were not controlled, so runtime comparisons are descriptive.

Metrics are sampled every five selected frames. Headline accuracy summaries average the 20 frames at indices $20,25,\ldots,115$, separately per seed, then report the mean and sample standard deviation across three seeds. Prespecified snapshots at frames 59 and 119 are retained for visualization but excluded from those averages. Timing percentiles use all 120 updates, including initialization. The selected frames span approximately 19.8\,s for \texttt{desk} and 26.6\,s for \texttt{xyz}; these are decimated recordings, not a live full-rate deployment.
''')
# Additional rotation sequence uses the same settings but is recorded separately.
supp=EXP/'results_rotation/summary.csv'
if supp.exists():
 sr=list(csv.DictReader(supp.open()))
 sl=[r'\begin{table}[t]',r'\centering\small',r'\caption{Additional rotation sequence, fr1/rpy, at $B=256$. Mean errors in cm across three seeds; same fixed protocol. This sequence was added after the two-sequence primary protocol was fixed.}',r'\label{tab:rotation}',r'\begin{tabular}{lrr}',r'\toprule Method & All observed & Focal \\',r'\midrule']
 for m in methods:
  z=q('freiburg1_rpy',m,source=sr)
  if len(z)!=3:raise ValueError('Supplemental run incomplete')
  sl.append(f'{names[m]} & {mean(z,"seen_mean_m")*100:.2f} & {mean(z,"seen_roi_mean_m")*100:.2f} \\\\')
 sl.extend([r'\bottomrule\end{tabular}',r'\end{table}'])
 (P/'rotation_table.tex').write_text('\n'.join(sl)+'\n')
if supp.exists():
 for method,word in method_words.items():
  rs=q('freiburg1_rpy',method,source=sr)
  for metric,(ending,scale) in metric_words.items():
   value=mean(rs,metric)*scale
   if np.isfinite(value):macro.append('\\newcommand{\\Rpy'+word+ending+'}{'+f'{value:.2f}'+'}')
 rr=q('freiburg1_rpy','memory',source=sr);uu=q('freiburg1_rpy','memory_uniform',source=sr)
 for ending,val in [('FocusGain',100*(1-mean(rr,'seen_roi_mean_m')/mean(uu,'seen_roi_mean_m'))),('GlobalCost',100*(mean(rr,'seen_mean_m')/mean(uu,'seen_mean_m')-1))]:
  macro.append('\\newcommand{\\Rpy'+ending+'}{'+f'{val:.1f}'+'}')
(P/'numbers.tex').write_text('\n'.join(macro)+'\n')
print('Generated numerical tables and protocol from completed primary runs.')
