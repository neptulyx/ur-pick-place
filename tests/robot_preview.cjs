'use strict';
const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const vm=require('node:vm');
const web=path.join(__dirname,'../src/ur_pick_place/web');
const {UR10E_MODEL:model,UR_MODEL_SHA256:hash,DEFAULT_COLLISION_SCENE:example}=require(web+'/ur10e-model.js');
const {robotFrames,rigid,multiply,point,validateScene,readJointPlan}=require(web+'/robot-preview.js');
const frame=multiply(robotFrames([0,0,0,0,0,0],model).at(-1),rigid(model.tool0));
point(frame,[0,0,0]).forEach((v,i)=>assert.ok(Math.abs(v-[-1.18425,-.2907,.06085][i])<1e-8));
assert.deepEqual(validateScene(example,hash),example);
for(const change of [d=>d.extra=1,d=>d.confirmed='yes',d=>d.model_sha256='wrong',d=>d.attached[0].dimensions_m[0]=0,d=>d.attached[1].name='tool',d=>d.attached[2].pose.rpy_rad[0]=Infinity]){const d=structuredClone(example);change(d);assert.throws(()=>validateScene(d,hash));}
const q=[0,0,0,0,0,0],p=readJointPlan({start_joints:q,plan:{targets:[{label:'tilt',joints:[.1,0,0,0,0,0],via_start:true}]}},hash);
assert.equal(p.segments.length,3);assert.equal(p.segments.at(-1).label,'return_start');
assert.throws(()=>readJointPlan({start_joints:q,plan:{targets:[{label:'bad',joints:[null,0,0,0,0,0]}]}},hash));

// Execute the real scripts with a small DOM double. This does not test browser layout.
const els={},downloads=[];let domReady;
class Element {
  get id(){return this._id;}set id(value){this._id=value;if(value)els[value]=this;}
  constructor(id=''){this.id=id;this._value='';this.children=[];this.events={};this.textContent='';this.checked=false;this.disabled=false;this._html='';}
  get value(){return this._value;}set value(v){this._value=String(v);}
  get options(){return this.children;}
  get innerHTML(){return this._html;}
  set innerHTML(s){this._html=s;for(const match of s.matchAll(/id="([^"]+)"/g))if(!els[match[1]])els[match[1]]=new Element(match[1]);}
  addEventListener(type,fn){(this.events[type]??=[]).push(fn);}
  dispatchEvent(event){return Promise.all((this.events[event.type]||[]).map(fn=>fn({...event,target:this})));}
  append(child){this.children.push(child);if(!this.value&&child.value)this.value=child.value;}
  replaceChildren(){this.children=[];this.value='';}
  insertBefore(child){this.append(child);}
  checkValidity(){return this.value.trim()!==''&&Number.isFinite(Number(this.value));}
  click(){if(this.download)downloads.push(this.download);return this.dispatchEvent({type:'click'});}
  getContext(){return new Proxy({},{get:()=>()=>{}});}
}
for(const id of ['length','width','height','x','y','z','roll','pitch','yaw','margin','coverage','message','iso','top','front','side','reset','preset','export','import'])els[id]=new Element(id);
[390,80,100,0,0,50,0,0,0,0].forEach((v,i)=>els[['length','width','height','x','y','z','roll','pitch','yaw','margin'][i]].value=v);els.coverage.value='tool-only';
const main=new Element(),events={};
const context={console,structuredClone,Blob,URL,Event:class{constructor(type){this.type=type;}},CustomEvent:class{constructor(type,{detail}){this.type=type;this.detail=detail;}},setTimeout:fn=>fn(),document:{getElementById:id=>els[id],createElement:()=>new Element(),querySelector:s=>s==='main'?main:new Element(),addEventListener:(type,fn)=>{if(type==='DOMContentLoaded')domReady=fn;else(events[type]??=[]).push(fn);},dispatchEvent:e=>(events[e.type]||[]).forEach(fn=>fn(e))}};
vm.createContext(context);for(const name of ['ur10e-model.js','robot-preview.js','tool-envelope.js'])vm.runInContext(fs.readFileSync(web+'/'+name,'utf8'),context,{filename:name});
domReady();
(async()=>{
  assert.equal(els['scene-state'].textContent,'AUTOMATIC MOTION BLOCKED');
  assert.equal(els.yaw.value,'90');
  assert.match(els['scene-frame'].textContent,/tool0 \(flange\)/);
  els['scene-box'].value='mount';await els['scene-box'].dispatchEvent({type:'change'});assert.match(els['scene-frame'].textContent,/tool0 \(flange\)/);
  await els['add-obstacle'].click();assert.equal(els['scene-box'].value,'obstacle_1');
  assert.match(els['scene-frame'].textContent,/base coordinates/);
  els['scene-box'].value='camera';await els['scene-box'].dispatchEvent({type:'change'});assert.match(els['scene-frame'].textContent,/tool0 \(flange\)/);
  els['scene-box'].value='obstacle_1';await els['scene-box'].dispatchEvent({type:'change'});
  await els['scene-export'].click();assert.equal(downloads.at(-1),'collision-scene.json');
  const count=downloads.length;els['body-0'].value='';await els['body-0'].dispatchEvent({type:'input'});await els['scene-export'].click();assert.equal(downloads.length,count);
  els['body-0'].value='200';await els['body-0'].dispatchEvent({type:'input'});
  els['scene-confirmed'].checked=true;await els['scene-confirmed'].dispatchEvent({type:'change'});
  els['scene-complete'].checked=true;await els['scene-complete'].dispatchEvent({type:'change'});
  assert.match(els['scene-state'].textContent,/PATH CHECK REQUIRED/);
  els.width.value='90';await els.width.dispatchEvent({type:'input'});assert.equal(els['scene-state'].textContent,'AUTOMATIC MOTION BLOCKED');
  els['path-import'].files=[{size:100,text:async()=>JSON.stringify({start_joints:q,plan:{targets:[{label:'tilt',joints:[.1,0,0,0,0,0]}]}})}];await els['path-import'].dispatchEvent({type:'change'});assert.equal(els['path-segment'].options.length,2);assert.match(els['path-status'].textContent,/no matching path check/);
  els['scene-import'].files=[{size:2,text:async()=>'{'}];await els['scene-import'].dispatchEvent({type:'change'});assert.match(els['scene-message'].textContent,/not valid JSON/);
  console.log('Robot geometry, scene validation, original script events, export, invalid input and path review: PASS');
})().catch(e=>{console.error(e);process.exitCode=1;});
