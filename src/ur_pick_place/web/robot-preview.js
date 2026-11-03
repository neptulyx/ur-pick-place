'use strict';
function multiply(a,b) {return a.map((row,i)=>row.map((_,j)=>row.reduce((s,__,k)=>s+a[i][k]*b[k][j],0)));}
function rigid(p) {
  const [r,y,w]=p.rpy_rad,[cr,sr,cp,sp,cy,sy]=[Math.cos(r),Math.sin(r),Math.cos(y),Math.sin(y),Math.cos(w),Math.sin(w)],v=p.xyz_m;
  return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,v[0]],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,v[1]],[-sp,cp*sr,cp*cr,v[2]],[0,0,0,1]];
}
function point(t,v) {return t.slice(0,3).map(row=>row.slice(0,3).reduce((s,n,i)=>s+n*v[i],row[3]));}
function identity() {return [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]];}
function finiteVector(v,n=3,bound=10) {return Array.isArray(v)&&v.length===n&&v.every(x=>typeof x==='number'&&Number.isFinite(x)&&Math.abs(x)<=bound);}
function robotFrames(q,model) {
  if(!finiteVector(q,6,4*Math.PI)) throw Error('Enter six valid joint angles.');
  const out=[identity()];
  model.joints.forEach((joint,i)=>out.push(multiply(multiply(out.at(-1),rigid(joint)),rigid({xyz_m:[0,0,0],rpy_rad:[0,0,q[i]]}))));
  return out;
}
function boxPoints(b) {
  const t=rigid(b.pose),pts=[];
  for(const x of [-1,1])for(const y of [-1,1])for(const z of [-1,1])pts.push(point(t,[x*b.dimensions_m[0]/2,y*b.dimensions_m[1]/2,z*b.dimensions_m[2]/2]));
  return pts;
}
function exactKeys(o,keys) {return o&&typeof o==='object'&&!Array.isArray(o)&&Object.keys(o).sort().join('|')===keys.slice().sort().join('|');}
function validateScene(d,hash) {
  if(!exactKeys(d,['version','type','frame','model_sha256','confirmed','environment_complete','clearance_m','model_uncertainty_m','attached','obstacles','notes'])||d.version!==1||d.type!=='ur10e_collision_scene'||d.frame!=='base'||d.model_sha256!==hash||typeof d.confirmed!=='boolean'||typeof d.environment_complete!=='boolean'||typeof d.notes!=='string'||d.notes.length>2000) throw Error('This file does not match the robot scene format or model.');
  if(!['clearance_m','model_uncertainty_m'].every(k=>typeof d[k]==='number'&&Number.isFinite(d[k])&&d[k]>=.0001&&d[k]<=.1)) throw Error('Set clearance and model uncertainty from 0.1 to 100 mm.');
  if(!Array.isArray(d.attached)||d.attached.length!==3||!Array.isArray(d.obstacles)||d.obstacles.length>40) throw Error('Include the tool, camera and mount. Use at most 40 obstacles.');
  const names=new Set(),arm=['base_link_inertia','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link'];
  for(const b of [...d.attached,...d.obstacles]) {
    if(!exactKeys(b,['name','dimensions_m','pose'])||typeof b.name!=='string'||!/^[A-Za-z0-9_]{1,40}$/.test(b.name)||names.has(b.name)||!finiteVector(b.dimensions_m)||b.dimensions_m.some(x=>x<=0)||!exactKeys(b.pose,['xyz_m','rpy_rad'])||!finiteVector(b.pose.xyz_m)||!finiteVector(b.pose.rpy_rad,3,2*Math.PI)) throw Error('A box has an invalid name, dimension or pose.');
    if(d.obstacles.includes(b)&&[...arm,'tool','camera','mount'].includes(b.name)) throw Error('An obstacle name conflicts with a robot part.');
    names.add(b.name);
  }
  if(d.attached.map(b=>b.name).sort().join('|')!=='camera|mount|tool') throw Error('Name the attached boxes tool, camera and mount.');
  return JSON.parse(JSON.stringify(d));
}
function readJointPlan(d,hash) {
  let segments;
  if(d.type==='ur10e_checked_joint_plan') {
    if(d.version!==1||d.model_sha256!==hash) throw Error('This path file uses a different robot model.');
    validateScene(d.scene,hash);segments=d.segments;
  } else {
    if(!finiteVector(d.start_joints,6,4*Math.PI)||!Array.isArray(d.plan?.targets)) throw Error('This file has no calibration joint plan.');
    segments=[];let q=d.start_joints;
    for(const t of d.plan.targets) {
      if(t.via_start){segments.push({label:'via_start',start_joints:q,target_joints:d.start_joints});q=d.start_joints;}
      segments.push({label:t.label,start_joints:q,target_joints:t.joints});q=t.joints;
    }
    segments.push({label:'return_start',start_joints:q,target_joints:d.start_joints});
  }
  if(!Array.isArray(segments)||segments.length<1||segments.length>200||segments.some(s=>typeof s.label!=='string'||s.label.length>80||!finiteVector(s.start_joints,6,4*Math.PI)||!finiteVector(s.target_joints,6,4*Math.PI)||s.status&&!['clear_in_model','collision','unresolved','unavailable'].includes(s.status))) throw Error('The joint path file is not valid.');
  return {segments,scene:d.scene||null};
}
function canonical(v) {if(Array.isArray(v))return '['+v.map(canonical).join(',')+']';if(v&&typeof v==='object')return '{'+Object.keys(v).sort().map(k=>JSON.stringify(k)+':'+canonical(v[k])).join(',')+'}';return JSON.stringify(v);}
if(typeof module!=='undefined')module.exports={multiply,rigid,point,robotFrames,boxPoints,validateScene,readJointPlan,canonical};

if(typeof document!=='undefined')document.addEventListener('DOMContentLoaded',()=>{
  const section=document.createElement('section');section.className='panel robot-panel';section.id='robot-panel';
  section.innerHTML=`<h2>UR10e and tool preview</h2><p class="badge">NOMINAL MODEL · NO ROBOT CONNECTION</p>
  <p>Drag the view to rotate it. Use the mouse wheel to change the scale.</p>
  <div class="robot-layout"><div><canvas id="robot-canvas" width="1000" height="620" tabindex="0" aria-label="UR10e, tool, camera, mount and cell preview"></canvas>
  <p class="hint">Keyboard: use arrow keys to rotate the view. Use plus or minus to change the scale.</p>
  <p class="legend">Grey: UR10e. Orange: tool. Blue: camera. Purple: mount. Green: cell.</p>
  <p>Box dimensions and positions use millimetres. Box angles and joint angles use degrees.</p>
  <p>The preview shows collision hulls. The nominal model uses generic UR10e data. It does not include this robot's factory calibration.</p>
  <p id="path-status" role="status">Example joint pose. No path check is loaded.</p>
  <label>Path file <input type="file" id="path-import" accept=".json,application/json"></label>
  <label>Path segment <select id="path-segment" disabled></select></label>
  <label>Position along segment <input id="path-position" type="range" min="0" max="1000" value="0" disabled></label>
  <div id="joint-inputs" class="grid"></div></div>
  <div><h3>Camera, mount and cell</h3><p>The tool box above excludes the camera and mount. These parts use separate boxes.</p><p>Mounting offsets use anobot as a reference. The offsets are unverified starting values.</p>
  <label>Box <select id="scene-box"></select></label><p id="scene-frame" class="hint" aria-live="polite"></p><div id="scene-fields" class="grid"></div>
  <button id="add-obstacle" type="button">Add cell box</button><button id="remove-obstacle" class="secondary" type="button">Remove cell box</button>
  <label>Clearance (mm) <input id="scene-clearance" type="number" min="0.1" max="100" step="0.1" value="5"></label>
  <label>Model uncertainty (mm) <input id="scene-uncertainty" type="number" min="0.1" max="100" step="0.1" value="3"></label>
  <p class="hint">Model uncertainty is the maximum position error for each body. Measure this error before motion.</p>
  <label class="check"><input type="checkbox" id="scene-confirmed"> Physical model checked</label>
  <p class="hint">Check all shapes and offsets against the physical robot. Check robot kinematics, cable coverage and model uncertainty.</p>
  <label class="check"><input type="checkbox" id="scene-complete"> Cell coverage checked</label>
  <p class="hint">Include the table, fixed board, dummy and nearby objects. Add at least one measured cell box.</p>
  <button id="scene-export" type="button">Export collision scene</button><label class="file secondary">Import collision scene<input type="file" id="scene-import" accept=".json,application/json"></label>
  <p id="scene-message" role="alert"></p><p id="scene-state" class="badge">AUTOMATIC MOTION BLOCKED</p>
  <p>This page cannot move the robot. The calibration application checks the scene and path before each joint move.</p>
  <p>The teacher uses this page for viewing only. Teacher motion does not use these checks.</p></div></div>
  <p class="hint">© 2023 Universal Robots A/S. <a href="UR-MODEL-NOTICE.txt">Model terms and notices</a>.</p>`;
  document.querySelector('main').insertBefore(section,document.querySelector('footer'));
  const el=id=>document.getElementById(id),copy=v=>JSON.parse(JSON.stringify(v));
  let scene=copy(DEFAULT_COLLISION_SCENE),q=[0,-1.9,-1.6,-1.2,1.5708,0],plan=null,az=.8,elevation=.35,zoom=1,bodyValid=true;
  const state=el('scene-state'),message=el('scene-message'),canvas=el('robot-canvas'),ctx=canvas.getContext('2d');
  const jointInputs=[];
  for(let i=0;i<6;i++){const label=document.createElement('label');label.textContent=['Base','Shoulder','Elbow','Wrist 1','Wrist 2','Wrist 3'][i]+' (deg)';const input=document.createElement('input');input.type='number';input.min=-720;input.max=720;input.step=.1;input.value=(q[i]*180/Math.PI).toFixed(2);input.addEventListener('input',()=>{const v=jointInputs.map(n=>n.value.trim()===''?NaN:Number(n.value)*Math.PI/180);if(finiteVector(v,6,4*Math.PI)){q=v;plan=null;el('path-status').textContent='Manual joint pose. No path check applies.';draw();}});label.append(input);jointInputs.push(input);el('joint-inputs').append(label);}
  const fieldKeys=['Length','Width','Height','Centre X','Centre Y','Centre Z','Roll','Pitch','Yaw'],bodyInputs=[];
  for(let i=0;i<9;i++){const label=document.createElement('label');label.textContent=fieldKeys[i]+(i<6?' (mm)':' (deg)');const input=document.createElement('input');input.type='number';input.step='.1';input.id='body-'+i;input.addEventListener('input',editBody);label.append(input);bodyInputs.push(input);el('scene-fields').append(label);}
  function selected(){return [...scene.attached,...scene.obstacles].find(b=>b.name===el('scene-box').value);}
  function options(){const previous=el('scene-box').value;el('scene-box').replaceChildren();for(const b of [...scene.attached,...scene.obstacles].filter(b=>b.name!=='tool')){const o=document.createElement('option');o.value=b.name;o.textContent=b.name;el('scene-box').append(o);}if([...el('scene-box').options].some(o=>o.value===previous))el('scene-box').value=previous;showBody();}
  function showBody(){const b=selected();if(!b)return;el('scene-frame').textContent=scene.obstacles.includes(b)?'Cell box pose uses base coordinates.':'Camera and mount poses use tool0 (flange) coordinates.';bodyValid=true;const v=[...b.dimensions_m.map(x=>x*1000),...b.pose.xyz_m.map(x=>x*1000),...b.pose.rpy_rad.map(x=>x*180/Math.PI)];bodyInputs.forEach((n,i)=>n.value=Number(v[i].toFixed(6)));el('remove-obstacle').disabled=!scene.obstacles.includes(b);}
  function changed(){scene.confirmed=false;scene.environment_complete=false;message.textContent='The scene changed. Check the physical model and cell again.';sync();draw();}
  function editBody(){const v=bodyInputs.map(n=>n.value.trim()===''?NaN:Number(n.value)),next=copy(scene),b=[...next.attached,...next.obstacles].find(x=>x.name===selected()?.name);if(!b)return;b.dimensions_m=v.slice(0,3).map(x=>x/1000);b.pose={xyz_m:v.slice(3,6).map(x=>x/1000),rpy_rad:v.slice(6).map(x=>x*Math.PI/180)};try{validateScene(next,UR_MODEL_SHA256);bodyValid=true;scene=next;changed();}catch{bodyValid=false;message.textContent='Enter valid dimensions and offsets. The view keeps the last valid scene.';}}
  function sync(){el('scene-confirmed').checked=scene.confirmed;el('scene-complete').checked=scene.environment_complete;el('scene-clearance').value=scene.clearance_m*1000;el('scene-uncertainty').value=scene.model_uncertainty_m*1000;state.textContent=scene.confirmed&&scene.environment_complete&&scene.obstacles.length?'SCENE RECORDED · PATH CHECK REQUIRED':'AUTOMATIC MOTION BLOCKED';showPathStatus();}
  function showPathStatus(){if(!plan)return;const segment=plan.segments[Number(el('path-segment').value)||0];const same=plan.scene&&canonical(plan.scene)===canonical(scene);el('path-status').textContent=same&&segment.status?'Recorded path check: '+segment.status.replaceAll('_',' ')+'. This is not a live check.':'This scene has no matching path check. Run the offline path check.';}
  el('scene-box').addEventListener('change',showBody);
  el('add-obstacle').addEventListener('click',()=>{if(scene.obstacles.length>=40){message.textContent='Use at most 40 cell boxes.';return;}let i=1;while(scene.obstacles.some(b=>b.name==='obstacle_'+i))i++;scene.obstacles.push({name:'obstacle_'+i,dimensions_m:[.2,.2,.2],pose:{xyz_m:[.6,0,.1],rpy_rad:[0,0,0]}});options();el('scene-box').value='obstacle_'+i;showBody();changed();});
  el('remove-obstacle').addEventListener('click',()=>{scene.obstacles=scene.obstacles.filter(b=>b!==selected());options();changed();});
  for(const [id,key] of [['scene-clearance','clearance_m'],['scene-uncertainty','model_uncertainty_m']])el(id).addEventListener('change',()=>{const old=scene[key];scene[key]=Number(el(id).value)/1000;try{validateScene(scene,UR_MODEL_SHA256);changed();}catch{scene[key]=old;message.textContent='Set clearance and model uncertainty from 0.1 to 100 mm.';}});
  for(const [id,key] of [['scene-confirmed','confirmed'],['scene-complete','environment_complete']])el(id).addEventListener('change',()=>{scene[key]=el(id).checked;sync();});
  function download(data,name){const url=URL.createObjectURL(new Blob([JSON.stringify(data,null,2)+'\n'],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  el('scene-export').addEventListener('click',()=>{try{if(!bodyValid||[...bodyInputs,...['length','width','height','x','y','z','roll','pitch','yaw','scene-clearance','scene-uncertainty'].map(el)].some(n=>String(n.value).trim()===''||!Number.isFinite(Number(n.value))||!n.checkValidity()))throw Error('invalid');for(const [id,key] of [['scene-clearance','clearance_m'],['scene-uncertainty','model_uncertainty_m']])if(Number(el(id).value)/1000!==scene[key])throw Error('uncommitted');validateScene(scene,UR_MODEL_SHA256);download(scene,'collision-scene.json');message.textContent='';}catch{message.textContent='The editor cannot export this scene. Check the values.';}});
  async function fileData(event){const file=event.target.files[0];if(!file)return null;if(file.size>1000000)throw Error('Use a JSON file smaller than 1 MB.');try{return JSON.parse(await file.text());}catch{throw Error('The file is not valid JSON.');}}
  el('scene-import').addEventListener('change',async event=>{try{const data=await fileData(event);if(!data)return;const next=validateScene(data,UR_MODEL_SHA256),tool=next.attached.find(b=>b.name==='tool');const vals=[...tool.dimensions_m.map(x=>x*1000),...tool.pose.xyz_m.map(x=>x*1000),...tool.pose.rpy_rad.map(x=>x*180/Math.PI)];if(vals.slice(0,3).some(x=>x>5000)||vals.slice(3,6).some(x=>Math.abs(x)>5000))throw Error('The tool exceeds the tool box editor limits.');['length','width','height','x','y','z','roll','pitch','yaw'].forEach((id,i)=>el(id).value=vals[i]);el('coverage').value='tool-only';el('length').dispatchEvent(new Event('input'));scene=next;options();sync();draw();message.textContent='';}catch(e){message.textContent='The editor did not import the scene. '+e.message;}finally{event.target.value='';}});
  document.addEventListener('tool-box-change',event=>{const p=event.detail;scene.attached.find(b=>b.name==='tool').dimensions_m=p.dimensions_m;scene.attached.find(b=>b.name==='tool').pose=copy(p.pose_in_flange);changed();});
  el('path-import').addEventListener('change',async event=>{try{const d=await fileData(event);if(!d)return;plan=readJointPlan(d,UR_MODEL_SHA256);el('path-segment').replaceChildren();plan.segments.forEach((s,i)=>{const o=document.createElement('option');o.value=i;o.textContent=(i+1)+': '+s.label;el('path-segment').append(o);});el('path-segment').disabled=el('path-position').disabled=false;el('path-position').value=0;pathPose();message.textContent='';}catch(e){message.textContent='The editor did not import the path. '+e.message;}finally{event.target.value='';}});
  function pathPose(){if(!plan)return;const s=plan.segments[Number(el('path-segment').value)||0],t=Number(el('path-position').value)/1000;q=s.start_joints.map((v,i)=>v*(1-t)+s.target_joints[i]*t);jointInputs.forEach((n,i)=>n.value=(q[i]*180/Math.PI).toFixed(2));showPathStatus();draw();}
  el('path-position').addEventListener('input',pathPose);el('path-segment').addEventListener('change',()=>{el('path-position').value=0;pathPose();});
  const faces=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]];
  function draw(){
    const fs=robotFrames(q,UR10E_MODEL),tool=multiply(fs.at(-1),rigid(UR10E_MODEL.tool0)),objects=UR10E_MODEL.bodies.map((b,i)=>({name:b.name,color:i%2?'#9ebccd':'#d6dfe3',points:b.vertices.map(p=>point(fs[b.depth],p)),faces:b.faces}));
    for(const b of [...scene.attached,...scene.obstacles])objects.push({name:b.name,color:{tool:'#e8a558',camera:'#45a7cb',mount:'#aa8ac5'}[b.name]||'#9abb8f',points:boxPoints(b).map(p=>scene.attached.includes(b)?point(tool,p):p),faces});
    const project=p=>{const u=Math.cos(az)*p[0]-Math.sin(az)*p[1],v=Math.sin(az)*p[0]+Math.cos(az)*p[1];return [u,Math.sin(elevation)*v-Math.cos(elevation)*p[2],Math.cos(elevation)*v+Math.sin(elevation)*p[2]];};
    const all=objects.flatMap(o=>o.points).map(project),xs=all.map(p=>p[0]),ys=all.map(p=>p[1]),lowx=Math.min(...xs),highx=Math.max(...xs),lowy=Math.min(...ys),highy=Math.max(...ys),scale=Math.min(850/(highx-lowx||1),500/(highy-lowy||1))*zoom;
    const screen=p=>{const v=project(p);return [500+(v[0]-(lowx+highx)/2)*scale,300+(v[1]-(lowy+highy)/2)*scale,v[2]];};
    ctx.fillStyle='#f3f7f9';ctx.fillRect(0,0,1000,620);const triangles=[];
    for(const o of objects){const points=o.points.map(screen);for(const f of o.faces)triangles.push({p:f.map(i=>points[i]),color:o.color,z:f.reduce((s,i)=>s+points[i][2],0)/3});}
    triangles.sort((a,b)=>a.z-b.z);for(const t of triangles){ctx.beginPath();t.p.forEach((p,i)=>i?ctx.lineTo(p[0],p[1]):ctx.moveTo(p[0],p[1]));ctx.closePath();ctx.fillStyle=t.color;ctx.fill();ctx.strokeStyle='#55707b55';ctx.lineWidth=.45;ctx.stroke();}
    ctx.font='14px sans-serif';for(const o of objects.filter(o=>!UR10E_MODEL.bodies.some(b=>b.name===o.name))){const centre=o.points.reduce((a,p)=>a.map((n,i)=>n+p[i]/o.points.length),[0,0,0]),v=screen(centre);ctx.fillStyle='#153e50';ctx.fillText(o.name,v[0]+8,v[1]-8);}
    for(const [frame,label] of [[identity(),'base'],[tool,'tool0']]){const origin=screen(point(frame,[0,0,0]));ctx.fillStyle='#173d4d';ctx.fillText(label,origin[0]+5,origin[1]+20);for(let i=0;i<3;i++){const end=[0,0,0];end[i]=.1;const v=screen(point(frame,end));ctx.beginPath();ctx.moveTo(origin[0],origin[1]);ctx.lineTo(v[0],v[1]);ctx.strokeStyle=['#b33e45','#238254','#2469ba'][i];ctx.lineWidth=3;ctx.stroke();ctx.fillStyle=ctx.strokeStyle;ctx.fillText('XYZ'[i],v[0]+4,v[1]);}}
    ctx.fillStyle='#395b69';ctx.fillText('Preview only · UR10e nominal collision hulls',18,600);
  }
  let dragging=null;canvas.addEventListener('pointerdown',e=>{dragging=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});canvas.addEventListener('pointermove',e=>{if(!dragging)return;az+=(e.clientX-dragging[0])*.008;elevation=Math.max(-1.4,Math.min(1.4,elevation+(e.clientY-dragging[1])*.008));dragging=[e.clientX,e.clientY];draw();});canvas.addEventListener('pointerup',()=>dragging=null);
  canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.3,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));draw();},{passive:false});
  canvas.addEventListener('keydown',e=>{if(e.key==='ArrowLeft')az-=.1;else if(e.key==='ArrowRight')az+=.1;else if(e.key==='ArrowUp')elevation=Math.min(1.4,elevation+.1);else if(e.key==='ArrowDown')elevation=Math.max(-1.4,elevation-.1);else if(e.key==='+')zoom=Math.min(3,zoom*1.1);else if(e.key==='-')zoom=Math.max(.3,zoom/1.1);else return;e.preventDefault();draw();});
  options();sync();draw();
  const tool=scene.attached.find(b=>b.name==='tool');['length','width','height','x','y','z','roll','pitch','yaw'].forEach((id,i)=>{const values=[...tool.dimensions_m.map(x=>x*1000),...tool.pose.xyz_m.map(x=>x*1000),...tool.pose.rpy_rad.map(x=>x*180/Math.PI)];el(id).value=values[i];});el('coverage').value='tool-only';el('length').dispatchEvent(new Event('input'));
});
