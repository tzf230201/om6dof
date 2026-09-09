#!/usr/bin/env node
'use strict';
// Runs the actual embedded application with a minimal DOM/canvas test double.
// No browser, network, ROS, hardware, or third-party packages.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const {test}=require('node:test');
const filename=process.argv[2];
if(!filename)throw Error('Usage: node test_comparison_view.js comparison.html');
const html=fs.readFileSync(filename,'utf8');
const blocks=[...html.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script>/g)];
const raw=blocks.find(m=>m[1].includes('id="comparison-data"'))[2];
const data=JSON.parse(raw),source=blocks.filter(m=>!m[1].includes('application/json')).map(m=>m[2]).join('\n');
new vm.Script(source);
function boot(payload=raw){
 const elements=new Map(),frames=[];
 class Element{
  constructor(attrs={}){this.id=attrs.id;this.value=attrs.value||'';this.checked='checked' in attrs;this.disabled='disabled' in attrs;this.hidden='hidden' in attrs;this.textContent='';this.children=[];this.events=new Map();this.clientWidth=500;this.clientHeight=440;this.calls=[];
   this.context=new Proxy({}, {get:(o,k)=>k in o?o[k]:(...args)=>this.calls.push([k,...args])});}
  addEventListener(name,fn){this.events.set(name,fn);}
  appendChild(el){this.children.push(el);return el;}
  replaceChildren(...els){this.children=els;}
  getContext(){return this.context;}
  getBoundingClientRect(){return {left:0,top:0};}
  setPointerCapture(){}
  emit(name,event={}){this.events.get(name)?.({preventDefault(){},...event});}
 }
 for(const m of html.matchAll(/<([a-z][\w-]*)\b([^>]*)>/gi)){
  const attrs=Object.fromEntries([...m[2].matchAll(/([\w-]+)(?:="([^"]*)")?/g)].map(x=>[x[1],x[2]||'']));
  if(attrs.id)elements.set(attrs.id,new Element(attrs));
 }
 const get=id=>elements.get(id);
 Object.entries({metric:'pose',view:'iso',axis:'all',level:'0'}).forEach(([k,v])=>get(k).value=v);
 get('comparison-data').textContent=payload;
 const doc={getElementById:get,createElement:()=>new Element()};
 const win={devicePixelRatio:1,addEventListener(){}};
 vm.runInNewContext(source,{document:doc,window:win,requestAnimationFrame:fn=>frames.push(fn),console});
 function flush(){let n=0;while(frames.length){frames.shift()();assert.ok(++n<10,'render must settle');}}
 flush();return {get,flush};
}
test('embedded data and all-sample statistics agree',()=>{
 const ui=boot();assert.equal(ui.get('error').hidden,true);
 assert.equal(data.points.length,data.statistics.points);
 assert.equal(ui.get('sample-count').textContent,data.points.length.toLocaleString());
 for(const metric of ['poses','positions']){
  const limit=metric==='poses'?1:2,counts={both:0,v1_only:0,v2_only:0,neither:0};
  data.points.forEach(p=>{const a=p[3]<=limit,b=p[4]<=limit;counts[a&&b?'both':a?'v1_only':b?'v2_only':'neither']++;});
  assert.deepEqual(counts,data.statistics[metric]);
 }
 assert.equal(ui.get('transition-rows').children.length,5);
 assert.ok(ui.get('canvas-v1').calls.some(c=>c[0]==='arc'));
});
test('metric and constant-coordinate filters use matched sample denominators',()=>{
 const ui=boot();ui.get('metric').value='position';ui.get('metric').emit('change');
 ui.get('axis').value='0';ui.get('axis').emit('change');ui.flush();
 const plane=data.points.filter(p=>p[0]===0);
 assert.equal(ui.get('level-label').textContent,'X = 0 mm');assert.equal(ui.get('view').value,'yz');
 assert.equal(ui.get('v1-count').textContent,`${plane.filter(p=>p[3]<=2).length.toLocaleString()} / ${plane.length.toLocaleString()} positions found`);
 assert.equal(ui.get('v2-count').textContent,`${plane.filter(p=>p[4]<=2).length.toLocaleString()} / ${plane.length.toLocaleString()} positions found`);
 ui.get('level').value='0';ui.get('level').emit('input');ui.flush();
 const min=Math.min(...data.points.map(p=>p[0]));assert.equal(ui.get('level-label').textContent,`X = ${min} mm`);
});
test('display filters do not change slice denominators; reset restores defaults',()=>{
 const ui=boot();const before=ui.get('v1-count').textContent;
 ui.get('found-only').checked=false;ui.get('found-only').emit('change');
 ui.get('changes-only').checked=false;ui.get('changes-only').emit('change');ui.flush();
 assert.equal(ui.get('v1-count').textContent,before);
 ui.get('axis').value='2';ui.get('axis').emit('change');ui.get('reset').emit('click');ui.flush();
 assert.equal(ui.get('axis').value,'all');assert.equal(ui.get('view').value,'iso');
 assert.equal(ui.get('found-only').checked,true);assert.equal(ui.get('level').disabled,true);
});
test('keyboard and pointer interactions redraw all synchronized canvases',()=>{
 const ui=boot();const canvases=['v1','v2','diff'].map(s=>ui.get('canvas-'+s));
 canvases.forEach(c=>c.calls=[]);canvases[0].emit('keydown',{key:'ArrowRight'});ui.flush();
 canvases.forEach(c=>assert.ok(c.calls.some(c=>c[0]==='clearRect')));
 canvases[1].emit('pointerdown',{pointerId:1,button:0,clientX:20,clientY:20});
 canvases[1].emit('pointermove',{pointerId:1,clientX:60,clientY:40});
 canvases[1].emit('pointerup',{pointerId:1,clientX:60,clientY:40});ui.flush();
 assert.equal(ui.get('view').value,'iso');
 canvases[2].emit('wheel',{deltaY:-20});ui.flush();assert.equal(ui.get('error').hidden,true);
});
test('point selection displays paired results at the selected XYZ',()=>{
 const fixture=JSON.parse(raw);fixture.points=[[0,0,0,0,2,.1,null,.01,.02,.01,5]];
 fixture.statistics.points=1;
 const ui=boot(JSON.stringify(fixture)),canvas=ui.get('canvas-v1');
 canvas.emit('pointerdown',{pointerId:2,button:0,clientX:250,clientY:220});
 canvas.emit('pointerup',{pointerId:2,clientX:250,clientY:220});ui.flush();
 assert.equal(ui.get('selection').textContent,'X 0 mm · Y 0 mm · Z 0 mm');
 const rows=ui.get('point-details').children[0].children;
 assert.equal(rows[1].children[1].textContent,'Pose found');
 assert.equal(rows[1].children[2].textContent,'Orientation unresolved');
});
test('malformed samples show an error instead of a blank page',()=>{
 const bad=JSON.parse(raw);bad.points=[[1,2,3,99,0]];
 const ui=boot(JSON.stringify(bad));assert.equal(ui.get('error').hidden,false);
 assert.match(ui.get('error').textContent,/Invalid comparison samples/);
});
