#pragma once

// Local-only HTML/Canvas viewer. C++ embeds the CSV data between these strings.
// No network fetches, WebGL, external assets, ROS, or hardware commands.
inline constexpr const char kViewer3dBeforeData[] = R"WORKSPACE(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>OM6DOF · 3D workspace explorer</title>
<style>
:root{color-scheme:light;--ink:#1d2d3f;--muted:#596a7b;--line:#dce3ea;--accent:#176758}
*{box-sizing:border-box}body{margin:0;background:#f2f5f7;color:var(--ink);font:14px/1.5 system-ui,-apple-system,sans-serif}
header{padding:22px 28px 18px;background:white;border-bottom:1px solid var(--line)}
.eyebrow{font-size:11px;letter-spacing:.14em;text-transform:uppercase;font-weight:700;color:var(--accent)}
h1{font-size:27px;line-height:1.2;letter-spacing:-.6px;margin:5px 0 8px}h2{font-size:14px;margin:0 0 12px}
p{margin:5px 0}a{color:#1760a5}nav{display:flex;gap:18px;flex-wrap:wrap;font-size:13px;margin-top:10px}
.layout{display:grid;grid-template-columns:288px minmax(0,1fr);gap:16px;padding:18px;max-width:1900px;margin:auto}
aside,.panel{background:white;border:1px solid var(--line);border-radius:12px}aside{align-self:start;padding:18px}
.section{padding:0 0 18px;margin:0 0 18px;border-bottom:1px solid var(--line)}.section:last-child{padding:0;margin:0;border:0}
.legend-row{display:flex;gap:8px;align-items:flex-start;margin:9px 0;font-size:12px}.marker{width:14px;height:14px;flex-shrink:0;margin-top:3px}
input[type=checkbox]{accent-color:var(--accent);margin-top:4px}input[type=range]{width:100%;accent-color:var(--accent)}
.row{display:flex;justify-content:space-between;align-items:center;gap:8px}.slice{margin:13px 0}.slice output{font-variant-numeric:tabular-nums;font-size:12px}
button,select{font:inherit;padding:7px 11px;border:1px solid #cbd6de;background:white;border-radius:6px;color:var(--ink);cursor:pointer}
button:hover{background:#eaf3f0}button[aria-pressed=true]{background:#dcedea;border-color:#559689}button:focus-visible,select:focus-visible,input:focus-visible,canvas:focus-visible{outline:3px solid #2b8199;outline-offset:2px}
.buttons{display:flex;gap:6px;flex-wrap:wrap}.small{font-size:12px;color:var(--muted)}.muted{color:var(--muted)}
.toolbar{display:flex;justify-content:space-between;gap:12px;flex-wrap:wrap;padding:12px 16px;border-bottom:1px solid var(--line)}
.stats{display:flex;gap:24px;flex-wrap:wrap;padding:14px 18px;border-bottom:1px solid var(--line)}.stat strong{display:block;font-size:22px;line-height:1.3;font-variant-numeric:tabular-nums}.stat span{font-size:11px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted)}
.canvas-wrap{position:relative;background:#f8fafb;border-radius:0 0 12px 12px;overflow:hidden}
canvas{display:block;width:100%;height:clamp(450px,65vh,800px);touch-action:none;cursor:grab}canvas:active{cursor:grabbing}
.hint{padding:8px 16px;background:#edf2f5;font-size:12px;color:var(--muted)}
.readout{padding:16px 18px;border-top:1px solid var(--line)}.readout table{border-collapse:collapse;width:100%;font-size:13px}.readout th,.readout td{padding:5px 10px 5px 0;text-align:left;border-bottom:1px solid #edf1f5}.readout th{font-weight:500;color:var(--muted);width:44%}.readout td{font-variant-numeric:tabular-nums}
.notice{background:#fff7e5;border:1px solid #ecdbad;padding:12px 16px;border-radius:9px;margin-top:14px;font-size:12px;color:#634e22}
details{padding:12px 16px;margin-top:14px}summary{cursor:pointer;font-weight:600}pre{font-size:12px;white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f6f8;padding:12px;border-radius:6px}
#empty-state{position:absolute;top:42%;left:10%;right:10%;text-align:center;pointer-events:none;background:#ffffffed;border:1px solid var(--line);border-radius:10px;padding:20px}
[hidden]{display:none!important}.error{padding:20px;background:#ffe4e4;color:#861e1e}
@media(max-width:850px){.layout{grid-template-columns:1fr;padding:10px}aside{display:grid;grid-template-columns:1fr 1fr;gap:18px}.section{margin:0}header{padding:18px}canvas{height:500px}.stats{gap:18px}}
@media(max-width:520px){aside{display:block}.section{margin-bottom:16px}canvas{height:440px}h1{font-size:24px}.toolbar{padding:10px}.stat strong{font-size:19px}}
</style></head><body>
<header><div class="eyebrow">Offline experiment · model-based reachability</div><h1>Cartesian workspace <span class="muted">/ 3D explorer</span></h1>
<p id="experiment-label" class="muted">Loading the embedded experiment snapshot…</p>
<nav><a href="index.html">All 2D slices</a><a href="analysis.md">Analysis</a><a href="points.csv">Point &amp; joint data</a><a href="summary.json">Experiment settings</a></nav></header>
<noscript><p class="error">JavaScript is required for the interactive 3D view. The <a href="index.html">2D slice maps</a> remain available without JavaScript.</p></noscript>
<div id="load-error" class="error" hidden role="alert"></div>
<div class="layout"><aside>
<section class="section"><h2>Result categories</h2>
<label class="legend-row"><input id="status-0" type="checkbox" checked><svg class="marker" viewBox="-8 -8 16 16" aria-hidden="true" data-shape="circle"><circle r="6" fill="#15976b"/></svg><span>Pose found · circle</span></label>
<label class="legend-row"><input id="status-1" type="checkbox" checked><svg class="marker" viewBox="-8 -8 16 16" aria-hidden="true" data-shape="triangle"><path d="M 0 -7 L 7 6 L -7 6 Z" fill="#397ac6"/></svg><span>Pose found · near singular · triangle</span></label>
<label class="legend-row"><input id="status-2" type="checkbox" checked><svg class="marker" viewBox="-8 -8 16 16" aria-hidden="true" data-shape="square"><rect x="-6" y="-6" width="12" height="12" fill="#e5a21a"/></svg><span>Position found · orientation unresolved · square</span></label>
<label class="legend-row"><input id="status-3" type="checkbox" checked><svg class="marker" viewBox="-8 -8 16 16" aria-hidden="true" data-shape="diamond"><path d="M 0 -7 L 7 0 L 0 7 L -7 0 Z" fill="#9658ad"/></svg><span>Only colliding candidates found · diamond</span></label>
<label class="legend-row"><input id="status-4" type="checkbox" checked><svg class="marker" viewBox="-8 -8 16 16" aria-hidden="true" data-shape="cross"><path d="M -5 -5 L 5 5 M -5 5 L 5 -5" fill="none" stroke="#d15b59" stroke-width="2.5" stroke-linecap="round"/></svg><span>Position unresolved · X</span></label>
<div class="buttons"><button id="show-all">All results</button><button id="show-poses">Found poses only</button></div>
</section>
<section class="section"><h2>Constant-coordinate slices</h2><p class="small">Enable an axis to show one sampled plane. Combine axes to inspect a line or point.</p>
<div class="slice"><div class="row"><label><input id="slice-enable-x" type="checkbox"> Constant X</label><output id="slice-label-x">All X</output></div><input id="slice-value-x" aria-label="Constant X coordinate" type="range" min="0" max="0" value="0" disabled></div>
<div class="slice"><div class="row"><label><input id="slice-enable-y" type="checkbox"> Constant Y</label><output id="slice-label-y">All Y</output></div><input id="slice-value-y" aria-label="Constant Y coordinate" type="range" min="0" max="0" value="0" disabled></div>
<div class="slice"><div class="row"><label><input id="slice-enable-z" type="checkbox"> Constant Z</label><output id="slice-label-z">All Z</output></div><input id="slice-value-z" aria-label="Constant Z coordinate" type="range" min="0" max="0" value="0" disabled></div>
<button id="clear-slices">Clear slices</button></section>
<section class="section"><h2>Display</h2><label class="row" for="point-size">Point size <output id="size-label">3 px</output></label><input id="point-size" type="range" min="1" max="8" step=".5" value="3">
<label class="row" for="opacity">Opacity <output id="opacity-label">75%</output></label><input id="opacity" type="range" min="10" max="100" value="75">
<label class="legend-row"><input id="show-grid" type="checkbox" checked> Base grid (Z = 0)</label>
<label class="legend-row"><input id="show-bound" type="checkbox"> Conservative URDF outer bound</label>
<p class="small">The bound is not a guarantee of reachability. Uncolored space has not been tested.</p></section>
</aside><main>
<div class="panel"><div class="toolbar"><div class="buttons" role="group" aria-label="Camera view">
<button id="view-iso" aria-pressed="true">3D</button><button id="view-xy" aria-pressed="false">XY</button><button id="view-xz" aria-pressed="false">XZ</button><button id="view-yz" aria-pressed="false">YZ</button><button id="reset">Reset all</button></div>
<label>Drag to <select id="drag-mode"><option value="rotate">rotate</option><option value="pan">pan</option></select></label></div>
<div class="stats" aria-live="polite"><div class="stat"><strong id="visible-count">—</strong><span>Visible / sampled points</span></div><div class="stat"><strong id="position-count">—</strong><span>Positions found · visible</span></div><div class="stat"><strong id="pose-count">—</strong><span>Poses found · visible</span></div></div>
<div class="canvas-wrap"><canvas id="workspace-canvas" tabindex="0" aria-label="Interactive 3D workspace. Drag to rotate, shift-drag to pan, scroll to zoom; arrow keys rotate and plus/minus zoom."></canvas>
<div id="empty-state" hidden>No samples match these filters.<br>Enable a result category or clear the slice filters.</div></div>
<div class="hint">Drag: rotate · Shift/right-drag: pan · Scroll: zoom · Click a point: inspect · Arrow keys: rotate · +/−: zoom</div>
<section class="readout"><div class="row"><h2>Selected sample</h2><button id="clear-selection">Clear selection</button></div>
<p id="selection-empty" class="small">Click a visible point to inspect its coordinates, errors and saved joint solution.</p><div id="selection-details" hidden></div></section></div>
<div class="notice"><strong>Unresolved is not proven unreachable.</strong> These are discrete model samples, not a physical robot test or a collision-free trajectory. Singularity indicators describe the best tested joint solution, not every possible solution at that XYZ.</div>
<details class="panel"><summary>Model, orientation and tolerances</summary><p class="small">Each sample tests one orientation. Radial mode changes yaw with the sample azimuth; fixed mode uses the same target angles throughout. Positions and angles come from the saved scan, not live feedback.</p><pre id="metadata"></pre></details>
<p class="small">The scan and report generator run in C++. This local browser view only displays saved data using JavaScript and a software-projected 3D canvas. No WebGL, external libraries or robot connection.</p>
</main></div>
<script id="workspace-data" type="application/json">)WORKSPACE";

inline constexpr const char kViewer3dAfterData[] = R"WORKSPACE(</script>
<script>
(() => {
'use strict';
const $ = id => document.getElementById(id);
try {
  const dataset = JSON.parse($('workspace-data').textContent);
  const points = dataset.points;
  if (!Array.isArray(points) || !points.length || points.some(p => !Array.isArray(p) || p.length !== 17 || !p.slice(0,3).every(Number.isFinite) || !Number.isInteger(p[3]) || p[3] < 0 || p[3] > 4)) throw new Error('The embedded point data is invalid or empty.');
  let meta = {};
  try { meta = JSON.parse(dataset.summaryText || '{}'); } catch (_) { /* Still display valid CSV data. */ }
  const args = meta.arguments || {};
  $('metadata').textContent = dataset.summaryText || 'No summary metadata available.';
  $('experiment-label').textContent = `${points.length.toLocaleString()} sampled points · ${args.spacing_mm ?? 'unknown'} mm grid · ${args.orientation || 'unspecified'} orientation · frame ${args.base_link || 'unspecified'}`;
  const labels = ['Pose found','Pose found · near singular','Position found · orientation unresolved','Only colliding candidates found','Position unresolved'];
  const colors = ['#15976b','#397ac6','#e5a21a','#9658ad','#d15b59'];
  const canvas = $('workspace-canvas'), ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('A 2D canvas is unavailable. Please use the 2D slice maps.');
  const levels = [0,1,2].map(a => [...new Set(points.map(p => Math.round(p[a]*1e6)/1e6))].sort((a,b)=>a-b));
  const maximum = points.reduce((m,p)=>Math.max(m,...p.slice(0,3).map(Math.abs)),1);
  const bound = Number.isFinite(meta.radius_mm) && meta.radius_mm > 0 ? meta.radius_mm : maximum;
  const extent = Math.max(maximum,bound)*1.10;
  const axes = ['x','y','z'];
  const state = {azimuth:-Math.PI/3,elevation:Math.PI/6,zoom:1,panX:0,panY:0,preset:'iso',selectedIndex:null,
    categories:[true,true,true,true,true],slices:[null,null,null]};
  let visible = [], projected = [], width = 1, height = 1, framePending = false;
  let drag = null;
  const dot = (a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  function basis() {
    if (state.preset==='xy') return [[1,0,0],[0,1,0],[0,0,1]];
    if (state.preset==='xz') return [[1,0,0],[0,0,1],[0,-1,0]];
    if (state.preset==='yz') return [[0,1,0],[0,0,1],[1,0,0]];
    const ca=Math.cos(state.azimuth),sa=Math.sin(state.azimuth),ce=Math.cos(state.elevation),se=Math.sin(state.elevation);
    return [[-sa,ca,0],[-se*ca,-se*sa,ce],[ce*ca,ce*sa,se]];
  }
  function projectPoint(p) {
    const [right,up,forward]=basis(), scale=Math.min(width,height)*.40/extent*state.zoom;
    return {x:width/2+state.panX+dot(right,p)*scale,y:height/2+state.panY-dot(up,p)*scale,depth:dot(forward,p)};
  }
  function requestDraw() {
    if (framePending) return;
    framePending=true;
    requestAnimationFrame(()=>{framePending=false;render();});
  }
  function line(vertices,color,lineWidth=1,dashed=false) {
    ctx.beginPath();
    vertices.forEach((v,i)=>{const p=projectPoint(v);if(i)ctx.lineTo(p.x,p.y);else ctx.moveTo(p.x,p.y);});
    ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dashed?[4,5]:[]);ctx.stroke();ctx.setLineDash([]);
  }
  function drawWorld() {
    const end=Math.ceil(maximum/100)*100, tick=Math.max(50,Math.ceil(end/500)*100);
    if ($('show-grid').checked) for(let t=-end;t<=end+.0001;t+=tick) {
      line([[-end,t,0],[end,t,0]],'#e1e7ec');line([[t,-end,0],[t,end,0]],'#e1e7ec');
    }
    if ($('show-bound').checked) for(let axis=0;axis<3;axis++) {
      const ring=[];
      for(let i=0;i<=80;i++){const p=[0,0,0];p[(axis+1)%3]=bound*Math.cos(i*Math.PI/40);p[(axis+2)%3]=bound*Math.sin(i*Math.PI/40);ring.push(p);}
      line(ring,'#a8b4c3',1,true);
    }
    state.slices.forEach((value,a)=>{
      if(value===null)return;
      const other=[0,1,2].filter(i=>i!==a);
      const vertices=[[-end,-end],[end,-end],[end,end],[-end,end],[-end,-end]].map(v=>{const p=[0,0,0];p[a]=value;p[other[0]]=v[0];p[other[1]]=v[1];return p;});
      line(vertices,'#78899a',1,true);
    });
    ['#ab4341','#318064','#426aaf'].forEach((color,a)=>{
      const low=[0,0,0],high=[0,0,0];low[a]=-end;high[a]=end;
      line([low,[0,0,0]],color,1,true);line([[0,0,0],high],color,1.8);
      const label=projectPoint(high);ctx.fillStyle=color;ctx.font='bold 12px system-ui';ctx.fillText(`${axes[a].toUpperCase()} (mm)`,label.x+7,label.y-7);
      if(state.preset!=='iso' && Math.hypot(label.x-width/2-state.panX,label.y-height/2-state.panY)<5)return;
      for(let t=-end;t<=end+.0001;t+=tick)if(t){const p=[0,0,0];p[a]=t;const at=projectPoint(p);ctx.font='10px system-ui';ctx.fillText(String(t),at.x+4,at.y+13);}
    });
    const origin=projectPoint([0,0,0]);ctx.fillStyle='#1c2d40';ctx.beginPath();ctx.arc(origin.x,origin.y,3,0,2*Math.PI);ctx.fill();
  }
  function drawMarker(status,x,y,radius) {
    ctx.beginPath();ctx.fillStyle=colors[status];ctx.strokeStyle=colors[status];
    if(status===0)ctx.arc(x,y,radius,0,Math.PI*2);
    else if(status===2)ctx.rect(x-radius,y-radius,2*radius,2*radius);
    else if(status===1){
      ctx.moveTo(x,y-radius*1.25);ctx.lineTo(x+radius*1.1,y+radius*.85);ctx.lineTo(x-radius*1.1,y+radius*.85);ctx.closePath();
    } else if(status===3){
      ctx.moveTo(x,y-radius*1.25);ctx.lineTo(x+radius*1.25,y);ctx.lineTo(x,y+radius*1.25);ctx.lineTo(x-radius*1.25,y);ctx.closePath();
    } else {
      ctx.moveTo(x-radius,y-radius);ctx.lineTo(x+radius,y+radius);ctx.moveTo(x-radius,y+radius);ctx.lineTo(x+radius,y-radius);
      ctx.lineWidth=Math.max(1.2,radius*.5);ctx.lineCap='round';ctx.stroke();ctx.lineCap='butt';return;
    }
    ctx.fill();
  }
  function render() {
    const rect=canvas.getBoundingClientRect();width=Math.max(1,rect.width);height=Math.max(1,rect.height);
    const ratio=Math.min(window.devicePixelRatio||1,2), w=Math.round(width*ratio),h=Math.round(height*ratio);
    if(canvas.width!==w || canvas.height!==h){canvas.width=w;canvas.height=h;}
    ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);ctx.fillStyle='#f8fafb';ctx.fillRect(0,0,width,height);
    drawWorld();
    projected=visible.map(index=>({index,...projectPoint(points[index])})).sort((a,b)=>a.depth-b.depth);
    const radius=Number($('point-size').value)||3;
    ctx.globalAlpha=Number($('opacity').value)/100;
    for(const p of projected) {
      const padding=radius*1.5;
      if(p.x < -padding || p.x>width+padding || p.y < -padding || p.y>height+padding)continue;
      drawMarker(points[p.index][3],p.x,p.y,radius);
    }
    ctx.globalAlpha=1;
    if(state.selectedIndex!==null) {
      const p=projectPoint(points[state.selectedIndex]);ctx.beginPath();ctx.arc(p.x,p.y,radius+5,0,Math.PI*2);
      ctx.strokeStyle='#182d43';ctx.lineWidth=2;ctx.stroke();
    }
  }
  function updateVisible() {
    visible=[];
    points.forEach((p,i)=>{if(state.categories[p[3]] && state.slices.every((s,a)=>s===null || Math.abs(p[a]-s)<1e-6))visible.push(i);});
    if(state.selectedIndex!==null && !visible.includes(state.selectedIndex))selectPoint(null);
    $('visible-count').textContent=`${visible.length.toLocaleString()} / ${points.length.toLocaleString()}`;
    $('position-count').textContent=visible.filter(i=>points[i][3]<=2).length.toLocaleString();
    $('pose-count').textContent=visible.filter(i=>points[i][3]<=1).length.toLocaleString();
    $('empty-state').hidden=visible.length!==0;requestDraw();
  }
  function setCategory(index,enabled) {
    if(!Number.isInteger(index)||index<0||index>=5)throw new Error('Invalid category');
    state.categories[index]=Boolean(enabled);$(`status-${index}`).checked=Boolean(enabled);updateVisible();
  }
  function setSlice(axis,value) {
    const a=axes.indexOf(axis);if(a<0)throw new Error('Invalid slice axis');
    if(value!==null && (!Number.isFinite(value)||!levels[a].some(v=>Math.abs(v-value)<1e-6)))throw new Error('Slice must use a sampled coordinate');
    state.slices[a]=value;$(`slice-enable-${axis}`).checked=value!==null;$(`slice-value-${axis}`).disabled=value===null;
    if(value!==null)$(`slice-value-${axis}`).value=String(levels[a].findIndex(v=>Math.abs(v-value)<1e-6));
    $(`slice-label-${axis}`).textContent=value===null?`All ${axis.toUpperCase()}`:`${value.toFixed(0)} mm`;updateVisible();
  }
  function setView(name) {
    if(!['iso','xy','xz','yz'].includes(name))throw new Error('Invalid camera view');
    state.preset=name;
    if(name==='iso'){state.azimuth=-Math.PI/3;state.elevation=Math.PI/6;}
    if(name==='xy'){state.azimuth=-Math.PI/2;state.elevation=Math.PI/2;}
    if(name==='xz'){state.azimuth=-Math.PI/2;state.elevation=0;}
    if(name==='yz'){state.azimuth=0;state.elevation=0;}
    ['iso','xy','xz','yz'].forEach(v=>$(`view-${v}`).setAttribute('aria-pressed',String(v===name)));requestDraw();
  }
  const fmt=(v,d=3)=>v===null||!Number.isFinite(v)?'n/a':v.toFixed(d);
  function selectPoint(index) {
    if(index!==null && (!Number.isInteger(index)||!visible.includes(index)))throw new Error('Select a visible sample');
    state.selectedIndex=index;$('selection-empty').hidden=index!==null;$('selection-details').hidden=index===null;
    if(index!==null) {
      const p=points[index], q=p.slice(8,14), rpy=p.slice(14,17);
      const rows=[['Coordinates X / Y / Z (mm)',p.slice(0,3).map(v=>fmt(v,1)).join(' / ')],['Result',labels[p[3]]],
        ['Position error (mm)',fmt(p[6])],['Orientation error (deg)',fmt(p[7])],['Scaled Jacobian rcond',fmt(p[4],6)],
        ['Smallest joint-limit margin (deg)',fmt(p[5])],['Tested roll / pitch / yaw (deg)',rpy.map(v=>fmt(v,2)).join(' / ')],
        ['Saved joint solution q1…q6 (deg)',q.map(v=>fmt(v===null?null:v*180/Math.PI,2)).join(', ')]];
      $('selection-details').innerHTML='<table>'+rows.map(r=>`<tr><th>${r[0]}</th><td>${r[1]}</td></tr>`).join('')+'</table><p class="small">n/a means no value was saved. A saved joint solution is a model witness, not a motion command.</p>';
    } else $('selection-details').textContent='';
    requestDraw();
  }
  function reset() {
    state.zoom=1;state.panX=state.panY=0;state.categories.fill(true);state.slices.fill(null);
    for(let i=0;i<5;i++)$(`status-${i}`).checked=true;
    axes.forEach((a,i)=>{const nearest=levels[i].reduce((best,v,k)=>Math.abs(v)<Math.abs(levels[i][best])?k:best,0);$(`slice-value-${a}`).value=String(nearest);setSlice(a,null);});
    $('point-size').value='3';$('size-label').textContent='3 px';$('opacity').value='75';$('opacity-label').textContent='75%';
    $('show-grid').checked=true;$('show-bound').checked=false;$('drag-mode').value='rotate';selectPoint(null);setView('iso');updateVisible();
  }
  for(let i=0;i<5;i++)$(`status-${i}`).addEventListener('change',e=>setCategory(i,e.target.checked));
  axes.forEach((a,i)=>{
    const slider=$(`slice-value-${a}`);slider.max=String(levels[i].length-1);
    slider.addEventListener('input',()=>setSlice(a,levels[i][Number(slider.value)]));
    $(`slice-enable-${a}`).addEventListener('change',e=>setSlice(a,e.target.checked?levels[i][Number(slider.value)]:null));
  });
  $('show-all').addEventListener('click',()=>{for(let i=0;i<5;i++)setCategory(i,true);});
  $('show-poses').addEventListener('click',()=>{for(let i=0;i<5;i++)setCategory(i,i<=1);});
  $('clear-slices').addEventListener('click',()=>axes.forEach(a=>setSlice(a,null)));
  ['iso','xy','xz','yz'].forEach(v=>$(`view-${v}`).addEventListener('click',()=>setView(v)));
  $('reset').addEventListener('click',reset);$('clear-selection').addEventListener('click',()=>selectPoint(null));
  $('point-size').addEventListener('input',()=>{$('size-label').textContent=$('point-size').value+' px';requestDraw();});
  $('opacity').addEventListener('input',()=>{$('opacity-label').textContent=$('opacity').value+'%';requestDraw();});
  ['show-grid','show-bound'].forEach(id=>$(id).addEventListener('change',requestDraw));
  function orbit(dx,dy) {
    state.preset='iso';state.azimuth-=dx*.008;state.elevation=Math.max(-1.56,Math.min(1.56,state.elevation+dy*.008));
    ['iso','xy','xz','yz'].forEach(v=>$(`view-${v}`).setAttribute('aria-pressed',String(v==='iso')));requestDraw();
  }
  canvas.addEventListener('pointerdown',e=>{
    if(drag)return;
    canvas.focus();canvas.setPointerCapture(e.pointerId);
    drag={id:e.pointerId,x:e.clientX,y:e.clientY,distance:0,pan:e.shiftKey||e.button===2||$('drag-mode').value==='pan'};
  });
  canvas.addEventListener('pointermove',e=>{
    if(!drag||drag.id!==e.pointerId)return;
    const dx=e.clientX-drag.x,dy=e.clientY-drag.y;drag.distance+=Math.hypot(dx,dy);drag.x=e.clientX;drag.y=e.clientY;
    if(drag.pan){state.panX+=dx;state.panY+=dy;requestDraw();}else orbit(dx,dy);
  });
  canvas.addEventListener('pointerup',e=>{
    if(!drag||drag.id!==e.pointerId)return;
    if(drag.distance<4 && !drag.pan){
      // Update cached projections before picking; RAF may not have run yet.
      render();const rect=canvas.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top;
      let best=null,dist=Math.max(9,(Number($('point-size').value)||3)*1.5+2);
      for(let i=projected.length-1;i>=0;i--){const p=projected[i],d=Math.hypot(p.x-x,p.y-y);if(d<dist){best=p.index;dist=d;}}
      selectPoint(best);
    }
    drag=null;
  });
  canvas.addEventListener('pointercancel',()=>{drag=null;});canvas.addEventListener('lostpointercapture',()=>{drag=null;});
  canvas.addEventListener('contextmenu',e=>e.preventDefault());
  canvas.addEventListener('wheel',e=>{e.preventDefault();state.zoom=Math.max(.25,Math.min(8,state.zoom*Math.exp(-e.deltaY*.001)));requestDraw();},{passive:false});
  canvas.addEventListener('keydown',e=>{
    if(e.key==='ArrowLeft')orbit(-8,0);else if(e.key==='ArrowRight')orbit(8,0);else if(e.key==='ArrowUp')orbit(0,8);else if(e.key==='ArrowDown')orbit(0,-8);
    else if(e.key==='+'||e.key==='='){state.zoom=Math.min(8,state.zoom*1.15);requestDraw();}
    else if(e.key==='-'){state.zoom=Math.max(.25,state.zoom/1.15);requestDraw();}
    else if(e.key==='Escape')selectPoint(null);else return;e.preventDefault();
  });
  window.addEventListener('resize',requestDraw);
  window.__workspaceViewer={getState:()=>({visibleIndices:[...visible],selectedIndex:state.selectedIndex,azimuth:state.azimuth,elevation:state.elevation,zoom:state.zoom,preset:state.preset}),
    projectPoint,setView,setSlice,setCategory,selectPoint,reset,render};
  reset();render();
} catch(error) {
  $('load-error').hidden=false;$('load-error').textContent=`Unable to display the 3D view: ${error.message} Use the 2D slice maps or regenerate this report.`;
  console.error(error);
}
})();
</script></body></html>)WORKSPACE";
