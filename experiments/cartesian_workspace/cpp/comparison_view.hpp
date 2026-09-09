#pragma once
// Static, self-contained comparison; all IK computations remain in C++.
inline constexpr const char kComparisonBeforeData[] = R"COMPARE(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OM6DOF · v1 / v2 workspace comparison</title>
<style>
:root{color-scheme:light;--ink:#203247;--muted:#52667b;--line:#d9e2ea;--green:#15976b;--blue:#397ac6;--yellow:#e5a21a;--red:#d15b59}
*{box-sizing:border-box}body{margin:0;background:#f3f6f9;color:var(--ink);font:16px/1.5 system-ui,sans-serif}
header{background:white;padding:24px max(20px,4vw);border-bottom:1px solid var(--line)}h1{margin:2px 0 8px;font-size:clamp(24px,3vw,34px);letter-spacing:-.7px}h2{font-size:18px;margin:0}p{margin:6px 0}
.eyebrow{text-transform:uppercase;letter-spacing:.1em;font-size:13px;color:#176758;font-weight:700}a{color:#2165a0}nav{display:flex;flex-wrap:wrap;gap:20px;margin-top:12px;font-size:14px}
main{max-width:1800px;padding:22px;margin:auto}.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:16px;margin-bottom:18px}.card,.surface{background:white;border:1px solid var(--line);border-radius:10px}.card{padding:17px 20px}.card h2{font-size:14px;color:var(--muted);font-weight:500}.big{font-size:27px;font-weight:700;font-variant-numeric:tabular-nums}.small{font-size:14px;color:var(--muted)}
.controls{padding:16px;display:flex;align-items:center;gap:16px;flex-wrap:wrap;border-bottom:1px solid var(--line)}label{font-size:14px;display:inline-flex;gap:8px;align-items:center}button,select{font:inherit;color:inherit;background:white;border:1px solid #bccbd6;border-radius:5px;padding:6px 10px}button{cursor:pointer}button:hover{background:#e9f2f5}input[type=range]{accent-color:#236983;width:160px}input[type=checkbox]{accent-color:#236983}button:focus-visible,select:focus-visible,input:focus-visible,canvas:focus-visible{outline:3px solid #2681aa;outline-offset:2px}
.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr))}.plot{min-width:0;border-right:1px solid var(--line)}.plot:last-child{border:0}.plot-head{padding:16px;border-bottom:1px solid var(--line)}.plot-head p{font-size:14px;color:var(--muted)}canvas{width:100%;height:440px;display:block;background:#f9fbfc;touch-action:none;cursor:grab}.legend{padding:12px 16px;display:flex;gap:12px;flex-wrap:wrap;font-size:14px}.key{white-space:nowrap}.hint{padding:10px 16px;border-top:1px solid var(--line);font-size:14px;color:var(--muted)}
section.readout{padding:18px;margin-top:18px}.table-wrap{overflow-x:auto}table{border-collapse:collapse;width:100%;font-size:14px;margin-top:12px}th,td{padding:9px 12px;border-bottom:1px solid var(--line);text-align:right;font-variant-numeric:tabular-nums}th:first-child,td:first-child{text-align:left}caption{text-align:left;font-size:16px;font-weight:600;margin:8px 0}.notice{background:#fff8e8;border:1px solid #e4cf9e;border-radius:8px;padding:15px 18px;margin-top:18px;font-size:14px}details{padding:18px;margin-top:18px}summary{cursor:pointer;font-weight:600}pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}#error{padding:18px;color:#912b29}output{font-variant-numeric:tabular-nums}
@media(max-width:1000px){.panels{grid-template-columns:1fr}.plot{border-right:0;border-bottom:1px solid var(--line)}canvas{height:360px}}@media(max-width:640px){.cards{grid-template-columns:1fr}main{padding:12px}.controls{gap:12px}input[type=range]{width:120px}}
</style></head><body>
<header><div class="eyebrow">Offline model experiment · C++ / KDL</div><h1>Workspace comparison <span class="small">om6dof → om6dof_v2</span></h1>
<p id="subtitle">Matched samples from two independently rendered URDF models.</p><nav>
<a href="v1/viewer3d.html">v1 full 3D explorer ↗</a><a href="v2/viewer3d.html">v2 full 3D explorer ↗</a>
<a href="v1/index.html">v1 slice report</a><a href="v2/index.html">v2 slice report</a><a href="comparison.md">Analysis</a><a href="comparison.json">Statistics JSON</a><a href="comparison_slices.csv">Per-plane CSV</a></nav></header>
<div id="error" role="alert" hidden></div><main>
<div class="cards"><div class="card"><h2>Position found · all samples</h2><div id="position-rate" class="big">—</div><p id="position-delta" class="small"></p></div>
<div class="card"><h2>Pose found · all samples</h2><div id="pose-rate" class="big">—</div><p id="pose-delta" class="small"></p></div>
<div class="card"><h2>Matched XYZ samples</h2><div id="sample-count" class="big">—</div><p class="small">Same radius, target orientations and search budget.</p></div></div>
<section class="surface"><div class="controls">
<label>Compare <select id="metric"><option value="pose">Pose (position + orientation)</option><option value="position">Position only</option></select></label>
<label>View <select id="view"><option value="iso">3D</option><option value="xy">XY</option><option value="xz">XZ</option><option value="yz">YZ</option></select></label>
<label>Slice <select id="axis"><option value="all">All planes</option><option value="0">Constant X</option><option value="1">Constant Y</option><option value="2">Constant Z</option></select></label>
<input id="level" type="range" min="0" max="0" value="0" aria-label="Slice coordinate" disabled><output id="level-label">All planes</output>
<label><input type="checkbox" id="found-only" checked> Found results only (v1/v2)</label>
<label><input type="checkbox" id="changes-only" checked> Differences only (right panel)</label><button id="reset">Reset</button>
</div><div class="panels">
<div class="plot"><div class="plot-head"><h2>om6dof · v1</h2><p id="v1-count">—</p></div><canvas id="canvas-v1" tabindex="0" aria-label="v1 workspace, drag to rotate, scroll to zoom, arrows rotate"></canvas></div>
<div class="plot"><div class="plot-head"><h2>om6dof_v2 · v2</h2><p id="v2-count">—</p></div><canvas id="canvas-v2" tabindex="0" aria-label="v2 workspace, synchronized view"></canvas></div>
<div class="plot"><div class="plot-head"><h2>Found-result overlap</h2><p id="diff-count">—</p></div><canvas id="canvas-diff" tabindex="0" aria-label="Matched workspace overlap, synchronized view"></canvas></div>
</div>
<div class="legend" aria-label="v1 and v2 categories"><strong>v1 / v2:</strong><span class="key" style="color:var(--green)">● Pose found</span><span class="key" style="color:var(--blue)">▲ Near-singular pose</span><span class="key" style="color:var(--yellow)">■ Orientation unresolved</span><span class="key" style="color:#9658ad">◆ Colliding candidates only</span><span class="key" style="color:var(--red)">✕ Position unresolved</span></div>
<div class="legend" aria-label="Overlap categories"><strong>Overlap:</strong><span class="key" style="color:var(--green)">● v2 only</span><span class="key" style="color:var(--yellow)">■ v1 only</span><span class="key" style="color:var(--blue)">▲ Both found</span><span class="key" style="color:var(--red)">✕ Neither found</span></div>
<div class="hint">Views are synchronized. Drag to rotate · Scroll or +/− to zoom · Arrow keys to rotate · Click a point to inspect both results. Counts use every sample on the selected plane, before display filters.</div>
</section>
<section class="surface readout"><h2>Same-point inspection</h2><p id="selection" class="small" aria-live="polite">Select a point in any panel.</p><div id="point-details" class="table-wrap"></div></section>
<section class="surface readout"><h2>Coverage overlap · all sampled points</h2><div class="table-wrap"><table><thead><tr><th>Metric</th><th>Both found</th><th>v1 only</th><th>v2 only</th><th>Neither found</th></tr></thead><tbody id="overlap-rows"></tbody></table></div>
<div class="table-wrap"><table><caption>Category transitions · rows v1 → columns v2</caption><thead id="transition-head"></thead><tbody id="transition-rows"></tbody></table></div></section>
<div class="notice"><strong>Unresolved is not proven unreachable.</strong> This is a finite grid and multi-start IK search, with one orientation per point. Collision uses simplified chain capsules, not triangle meshes: the D435/bracket geometry and new link3 mesh are not assessed. Mass, CoM, dynamics, payload capacity, a table/floor and safe trajectories are not tested. Near-singularity describes the selected joint solution, not every solution at that point.</div>
<details class="surface"><summary>Matched settings and model joint limits</summary><p class="small">Both models are rerun on a common grid; these are not percentages taken from the older public demo. The numerical seed banks can differ with the model even with the same RNG seed and budget.</p><h2>v1</h2><pre id="meta-v1"></pre><h2>v2</h2><pre id="meta-v2"></pre></details>
</main><script id="comparison-data" type="application/json">)COMPARE";

inline constexpr const char kComparisonAfterData[] = R"COMPARE(</script><script>
(() => {'use strict';
const $=id=>document.getElementById(id);
try {
 const data=JSON.parse($('comparison-data').textContent), points=data.points, stats=data.statistics;
 if(!Array.isArray(points)||!points.length||points.some(p=>p.length!==11||!p.slice(0,3).every(Number.isFinite)||!p.slice(3,5).every(s=>Number.isInteger(s)&&s>=0&&s<5)))throw Error('Invalid comparison samples.');
 const metas=[JSON.parse(stats.v1SummaryText),JSON.parse(stats.v2SummaryText)];
 const labels=['Pose found','Pose near singular','Orientation unresolved','Colliding candidates only','Position unresolved'];
 const colors=['#15976b','#397ac6','#e5a21a','#9658ad','#d15b59'];
 const found=(s,metric)=>s<=(metric==='position'?2:1);
 const fmt=n=>n.toLocaleString(), pct=n=>(100*n/points.length).toFixed(2)+'%';
 $('subtitle').textContent=`${metas[0].arguments.spacing_mm} mm grid · ${metas[0].radius_mm} mm radius · ${metas[0].arguments.orientation} orientation · frame ${metas[0].arguments.base_link}`;
 $('sample-count').textContent=fmt(points.length);
 for(const metric of ['position','pose']) {
  const a=metas[0][metric+'_found'],b=metas[1][metric+'_found'];
  $(metric+'-rate').textContent=`${pct(a)} → ${pct(b)}`;
  $(metric+'-delta').textContent=`v1 ${fmt(a)} · v2 ${fmt(b)} · ${b>=a?'+':''}${(100*(b-a)/points.length).toFixed(2)} percentage points`;
 }
 $('meta-v1').textContent=JSON.stringify(metas[0],null,2);$('meta-v2').textContent=JSON.stringify(metas[1],null,2);
 function row(parent,values,heading=false){const tr=document.createElement('tr');values.forEach((v,i)=>{const cell=document.createElement(heading||i===0?'th':'td');cell.textContent=String(v);tr.appendChild(cell);});parent.appendChild(tr);}
 for(const metric of ['positions','poses']) {const v=stats[metric];row($('overlap-rows'),[metric,...['both','v1_only','v2_only','neither'].map(k=>fmt(v[k]))]);}
 row($('transition-head'),['v1 → v2',...labels],true);stats.transitions.forEach((r,i)=>row($('transition-rows'),[labels[i],...r.map(fmt)]));
 const canvases=['v1','v2','diff'].map(s=>$('canvas-'+s)), contexts=canvases.map(c=>c.getContext('2d'));
 if(contexts.some(c=>!c))throw Error('Canvas is unavailable; use the linked slice reports.');
 const levels=[0,1,2].map(a=>[...new Set(points.map(p=>p[a]))].sort((a,b)=>a-b));
 const extent=Math.max(metas[0].radius_mm,1)*1.12;
 const state={az:-Math.PI/3,el:Math.PI/6,zoom:1,selected:null};
 let visible=[],hits=[[],[],[]],pending=false,drag=null;
 function basis(){
  if($('view').value==='xy')return [[1,0,0],[0,1,0],[0,0,1]];
  if($('view').value==='xz')return [[1,0,0],[0,0,1],[0,-1,0]];
  if($('view').value==='yz')return [[0,1,0],[0,0,1],[1,0,0]];
  const ca=Math.cos(state.az),sa=Math.sin(state.az),ce=Math.cos(state.el),se=Math.sin(state.el);
  return [[-sa,ca,0],[-se*ca,-se*sa,ce],[ce*ca,ce*sa,se]];
 }
 const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
 function project(p,w,h){const [right,up,front]=basis(),scale=Math.min(w,h)*.42/extent*state.zoom;return {x:w/2+dot(right,p)*scale,y:h/2-dot(up,p)*scale,z:dot(front,p)};}
 function marker(ctx,s,x,y,r){ctx.beginPath();ctx.fillStyle=ctx.strokeStyle=colors[s];
  if(s===0)ctx.arc(x,y,r,0,2*Math.PI);
  else if(s===1){ctx.moveTo(x,y-r*1.3);ctx.lineTo(x+r,y+r);ctx.lineTo(x-r,y+r);ctx.closePath();}
  else if(s===2)ctx.rect(x-r,y-r,r*2,r*2);
  else if(s===3){ctx.moveTo(x,y-r);ctx.lineTo(x+r,y);ctx.lineTo(x,y+r);ctx.lineTo(x-r,y);ctx.closePath();}
  else{ctx.moveTo(x-r,y-r);ctx.lineTo(x+r,y+r);ctx.moveTo(x-r,y+r);ctx.lineTo(x+r,y-r);ctx.lineWidth=1.3;ctx.stroke();return;}
  ctx.fill();
 }
 function overlap(p){const a=found(p[3],$('metric').value),b=found(p[4],$('metric').value);return a&&b?1:a?2:b?0:4;}
 function selectedPlane(){return $('axis').value==='all'?null:[Number($('axis').value),levels[Number($('axis').value)][Number($('level').value)]];}
 function refresh(){
  const plane=selectedPlane();visible=points.map((p,i)=>({p,i})).filter(({p})=>!plane||p[plane[0]]===plane[1]);
  $('level-label').textContent=plane?`${'XYZ'[plane[0]]} = ${plane[1]} mm`:'All planes';
  for(let panel=0;panel<2;panel++){
   const count=visible.filter(({p})=>found(p[3+panel],$('metric').value)).length;
   $(`v${panel+1}-count`).textContent=`${fmt(count)} / ${fmt(visible.length)} ${$('metric').value}s found`;
  }
  const only1=visible.filter(({p})=>overlap(p)===2).length,only2=visible.filter(({p})=>overlap(p)===0).length;
  $('diff-count').textContent=`v1 only ${fmt(only1)} · v2 only ${fmt(only2)}`;
  schedule();
 }
 function schedule(){if(pending)return;pending=true;requestAnimationFrame(()=>{pending=false;draw();});}
 function draw(){canvases.forEach((canvas,panel)=>{
  const ctx=contexts[panel],w=canvas.clientWidth,h=canvas.clientHeight,dpr=Math.min(window.devicePixelRatio||1,2);
  canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,w,h);
  function line(a,b,color){const u=project(a,w,h),v=project(b,w,h);ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=1;ctx.moveTo(u.x,u.y);ctx.lineTo(v.x,v.y);ctx.stroke();}
  const end=Math.ceil(metas[0].radius_mm/100)*100;
  for(let t=-end;t<=end;t+=100){line([-end,t,0],[end,t,0],'#e0e7ee');line([t,-end,0],[t,end,0],'#e0e7ee');}
  ['#bc4545','#288365','#397ac6'].forEach((color,a)=>{const l=[0,0,0],u=[0,0,0];l[a]=-end;u[a]=end;line(l,u,color);const label=project(u,w,h);ctx.fillStyle=color;ctx.font='bold 14px system-ui';ctx.fillText('XYZ'[a]+' mm',label.x+3,label.y-6);});
  hits[panel]=visible.filter(({p})=>panel<2?(!$('found-only').checked||found(p[3+panel],$('metric').value)):(!$('changes-only').checked||[0,2].includes(overlap(p))))
   .map(({p,i})=>({...project(p,w,h),p,i})).sort((a,b)=>a.z-b.z);
  ctx.globalAlpha=.78;
  for(const h of hits[panel])marker(ctx,panel<2?h.p[3+panel]:overlap(h.p),h.x,h.y,selectedPlane()?3.2:2.3);
  ctx.globalAlpha=1;
  if(!hits[panel].length){ctx.fillStyle='#52667b';ctx.font='16px system-ui';ctx.fillText('No samples match the display filters.',18,32);}
  if(state.selected!==null&&visible.some(v=>v.i===state.selected)){const p=project(points[state.selected],w,h);ctx.beginPath();ctx.arc(p.x,p.y,8,0,2*Math.PI);ctx.strokeStyle='#172e47';ctx.lineWidth=2;ctx.stroke();}
 });}
 function inspect(i){state.selected=i;const p=points[i];$('selection').textContent=`X ${p[0]} mm · Y ${p[1]} mm · Z ${p[2]} mm`;
  const dest=$('point-details');dest.replaceChildren();const table=document.createElement('table');row(table,['Recorded result','v1','v2'],true);
  row(table,['Category',labels[p[3]],labels[p[4]]]);
  const val=n=>n===null?'n/a':n.toFixed(4);
  row(table,['Jacobian reciprocal condition',val(p[5]),val(p[6])]);row(table,['Position error (mm)',val(p[7]),val(p[8])]);row(table,['Orientation error (deg)',val(p[9]),val(p[10])]);dest.appendChild(table);schedule();
 }
 $('axis').addEventListener('change',()=>{const a=$('axis').value;$('level').disabled=a==='all';if(a!=='all'){const ls=levels[Number(a)];$('level').max=ls.length-1;$('level').value=Math.max(0,ls.indexOf(0));$('view').value=['yz','xz','xy'][Number(a)];}refresh();});
 for(const id of ['level','metric','view','found-only','changes-only'])$(id).addEventListener(id==='level'?'input':'change',refresh);
 $('reset').addEventListener('click',()=>{$('axis').value='all';$('view').value='iso';$('metric').value='pose';$('level').disabled=true;$('found-only').checked=true;$('changes-only').checked=true;state.az=-Math.PI/3;state.el=Math.PI/6;state.zoom=1;state.selected=null;$('selection').textContent='Select a point in any panel.';$('point-details').replaceChildren();refresh();});
 canvases.forEach((c,panel)=>{
  c.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag={id:e.pointerId,x:e.clientX,y:e.clientY,moved:false};c.setPointerCapture(e.pointerId);});
  c.addEventListener('pointermove',e=>{if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(Math.abs(dx)+Math.abs(dy)>2)drag.moved=true;if(drag.moved){$('view').value='iso';state.az+=dx*.008;state.el=Math.max(-1.5,Math.min(1.5,state.el+dy*.008));drag.x=e.clientX;drag.y=e.clientY;schedule();}});
  c.addEventListener('pointerup',e=>{if(!drag)return;const moved=drag.moved;drag=null;if(moved)return;const box=c.getBoundingClientRect(),x=e.clientX-box.left,y=e.clientY-box.top;let best=null,d=100;for(const h of hits[panel]){const s=(h.x-x)**2+(h.y-y)**2;if(s<d){d=s;best=h.i;}}if(best!==null)inspect(best);});
  c.addEventListener('pointercancel',()=>{drag=null;});
  c.addEventListener('wheel',e=>{e.preventDefault();state.zoom=Math.max(.4,Math.min(5,state.zoom*Math.exp(-e.deltaY*.001)));schedule();},{passive:false});
  c.addEventListener('keydown',e=>{if(!['ArrowLeft','ArrowRight','ArrowUp','ArrowDown','+','=','-'].includes(e.key))return;e.preventDefault();if(['+','='].includes(e.key))state.zoom=Math.min(5,state.zoom*1.1);else if(e.key==='-')state.zoom=Math.max(.4,state.zoom/1.1);else{$('view').value='iso';state.az+=(e.key==='ArrowRight'?.1:e.key==='ArrowLeft'?-.1:0);state.el=Math.max(-1.5,Math.min(1.5,state.el+(e.key==='ArrowUp'?.1:e.key==='ArrowDown'?-.1:0)));}schedule();});
 });
 window.addEventListener('resize',schedule);refresh();
}catch(error){$('error').hidden=false;$('error').textContent='Comparison could not load: '+error.message;}
})();
</script></body></html>)COMPARE";
