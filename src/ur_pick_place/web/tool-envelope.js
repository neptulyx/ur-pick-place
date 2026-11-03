'use strict';
const keys=['length','width','height','x','y','z','roll','pitch','yaw','margin'];
const defaults=[390,80,100,0,0,50,0,0,0,0];
const coverages=['unconfirmed','tool-only','includes-camera-mount'];
function validateValues(v,coverage){
  if(v.length!==10 || v.some(n=>typeof n!=='number'||!Number.isFinite(n))) throw Error('Enter a valid number in each field.');
  if(v.slice(0,3).some(n=>n<=0||n>5000)||v.slice(3,6).some(n=>Math.abs(n)>5000)||v.slice(6,9).some(n=>Math.abs(n)>360)||v[9]<0||v[9]>1000||!coverages.includes(coverage)) throw Error('Check the limits shown on this page. Select a valid coverage option.');
}
function profile(v,coverage){validateValues(v,coverage);return {version:1,type:'tool_bounding_box',frame:'tool0',dimensions_m:v.slice(0,3).map(n=>n/1000),pose_in_flange:{xyz_m:v.slice(3,6).map(n=>n/1000),rpy_rad:v.slice(6,9).map(n=>n*Math.PI/180)},clearance_m:v[9]/1000,assembly_coverage:coverage,status:'draft',purpose:'visualization_only'};}
function exact(o,k){return o!==null&&typeof o==='object'&&!Array.isArray(o)&&Object.keys(o).sort().join('|')===k.slice().sort().join('|');}
function parseProfile(p){
  if(!exact(p,['version','type','frame','dimensions_m','pose_in_flange','clearance_m','assembly_coverage','status','purpose'])||p.version!==1||p.type!=='tool_bounding_box'||p.frame!=='tool0'||p.status!=='draft'||p.purpose!=='visualization_only'||!exact(p.pose_in_flange,['xyz_m','rpy_rad'])) throw Error('This file does not match version 1 of the tool box format.');
  const arrays=[p.dimensions_m,p.pose_in_flange.xyz_m,p.pose_in_flange.rpy_rad];
  if(arrays.some(a=>!Array.isArray(a)||a.length!==3||a.some(n=>typeof n!=='number'||!Number.isFinite(n)))||typeof p.clearance_m!=='number'||!Number.isFinite(p.clearance_m)) throw Error('Each dimension, position, angle and margin in the file must be a valid number.');
  const v=[...arrays[0].map(n=>n*1000),...arrays[1].map(n=>n*1000),...arrays[2].map(n=>n*180/Math.PI),p.clearance_m*1000];validateValues(v,p.assembly_coverage);return {values:v,coverage:p.assembly_coverage};
}
function rotate(p,rpy){const [r,pit,y]=rpy.map(n=>n*Math.PI/180),[cr,sr,cp,sp,cy,sy]=[Math.cos(r),Math.sin(r),Math.cos(pit),Math.sin(pit),Math.cos(y),Math.sin(y)];return [(cy*cp)*p[0]+(cy*sp*sr-sy*cr)*p[1]+(cy*sp*cr+sy*sr)*p[2],(sy*cp)*p[0]+(sy*sp*sr+cy*cr)*p[1]+(sy*sp*cr-cy*sr)*p[2],-sp*p[0]+cp*sr*p[1]+cp*cr*p[2]];}
function corners(v,margin=0){let out=[];for(let x of [-1,1])for(let y of [-1,1])for(let z of [-1,1]){const q=rotate([x*(v[0]/2+margin),y*(v[1]/2+margin),z*(v[2]/2+margin)],v.slice(6,9));out.push(q.map((n,i)=>n+v[i+3]));}return out;}
function render(v,view){
 const project={iso:([x,y,z])=>[(x-y)*.866,(x+y)*.5+z],top:([x,y])=>[x,-y],front:([x,y,z])=>[x,z],side:([x,y,z])=>[y,z]}[view];
 const inner=corners(v),outer=corners(v,v[9]), axes=[[0,0,0],[100,0,0],[0,100,0],[0,0,100]],all=[...inner,...outer,...axes].map(project),xs=all.map(p=>p[0]),ys=all.map(p=>p[1]),minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys),scale=Math.min(340/(maxx-minx||1),190/(maxy-miny||1));
 const xy=p=>{let q=project(p);return [200+(q[0]-(minx+maxx)/2)*scale,130+(q[1]-(miny+maxy)/2)*scale]};
 const line=(a,b,color,width,dash='')=>{let [x1,y1]=xy(a),[x2,y2]=xy(b);return `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${color}" stroke-width="${width}" ${dash}/>`};
 const edges=pts=>{let s='';for(let i=0;i<8;i++)for(let bit of [1,2,4])if(!(i&bit))s+=line(pts[i],pts[i|bit],pts===inner?'#146780':'#d0881c',pts===inner?2:1.5,pts===inner?'':'stroke-dasharray="5 4"');return s};
 let s=(v[9]?edges(outer):'')+edges(inner);for(let i=1;i<4;i++){s+=line(axes[0],axes[i],['','#c23b3b','#32905a','#3568bd'][i],2);let [x,y]=xy(axes[i]);s+=`<text x="${x+4}" y="${y-4}" font-size="12" fill="${['','#c23b3b','#32905a','#3568bd'][i]}">${['','X','Y','Z'][i]}</text>`;}let [x,y]=xy(axes[0]);s+=`<circle cx="${x}" cy="${y}" r="3" fill="#172d3b"/><text x="${x+5}" y="${y+15}" font-size="11" fill="#172d3b">flange origin</text><text x="12" y="266" font-size="11" fill="#405c67">${v[0]} × ${v[1]} × ${v[2]} mm · centre ${v[3]}, ${v[4]}, ${v[5]} mm</text>`;return s;
}
if(typeof module!=='undefined') module.exports={profile,parseProfile,rotate,corners,render};
if(typeof document!=='undefined'){
 const el=id=>document.getElementById(id),message=el('message');let last=defaults.slice();
 function read(){return keys.map(id=>{let s=el(id).value.trim();return s===''?NaN:Number(s)});}
 function update(){try{let v=read();validateValues(v,el('coverage').value);last=v;for(let view of ['iso','top','front','side'])el(view).innerHTML=render(v,view);message.textContent='';return true}catch(e){message.textContent=e.message+' The view keeps the last valid tool box.';return false}}
 function set(v,c){keys.forEach((k,i)=>el(k).value=v[i]);el('coverage').value=c;update()}
 keys.forEach(k=>el(k).addEventListener('input',update));el('coverage').addEventListener('change',update);
 el('reset').addEventListener('click',()=>set(defaults,'unconfirmed'));
 el('preset').addEventListener('click',()=>{let v=read();try{validateValues(v,el('coverage').value);let centre=rotate([0,0,v[2]/2],v.slice(6,9));v.splice(3,3,...centre);set(v,el('coverage').value)}catch(e){message.textContent=e.message}});
 el('export').addEventListener('click',()=>{if(!update())return;let blob=new Blob([JSON.stringify(profile(last,el('coverage').value),null,2)+'\n'],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='tool-box-draft.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)});
 el('import').addEventListener('change',async e=>{try{let file=e.target.files[0];if(!file)return;if(file.size>100000)throw Error('The file is too large.');let raw;try{raw=await file.text()}catch{throw Error('The editor could not read the file.')}let data;try{data=JSON.parse(raw)}catch{throw Error('The file is not valid JSON.')}let p=parseProfile(data);set(p.values,p.coverage)}catch(err){message.textContent='The editor did not import the file. '+err.message+' The view did not change.'}finally{e.target.value=''}});update();
}
