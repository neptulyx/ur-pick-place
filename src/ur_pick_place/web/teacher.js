'use strict';
const $ = id => document.getElementById(id);
const token = document.querySelector('meta[name="teach-token"]').content;
let defaults, state, dirty = false, posting = false, previousResult = '', imageVersion = 0;
let connectionOK = true, validationImageId = '';
const steps = ['configure', 'capture', 'teach', 'locate', 'validate'];
let activeStep = 0;
const stepNames = ['Set up', 'Capture marker', 'Teach pick', 'Read & export', 'Validation'];
const guidance = [
  ['Prepare your setup', 'Use the calibration for this camera and its unchanged wrist mounting. Confirm the gripper TCP against the pendant.'],
  ['Frame the whole marker', 'Keep the camera and board still during capture. Review the detected axes and quality before continuing.'],
  ['Keep the marker fixed', 'Only move the robot. Teach the final pick position and orientation, then stop before saving.'],
  ['Read from a clear view', 'Show the whole marker and stop. The observation pose may differ from teaching. Re-read for every pick.'],
  ['Test marker tracking', 'Capture a repositioned marker and review all six target coordinates before requesting a slow move. Full 3D projects follow XYZ and orientation; planar projects hold capture Z.']
];
function availableStep() {
  const f=state?.project?.flow||{};
  // Validation makes its own fresh capture. Reopening clears export freshness,
  // but the saved marker-to-pick relationship is sufficient to capture again.
  return dirty||!state?.project ? 0 : f.taught ? 4 : f.captured ? 2 : 1;
}
function showStep(index) {
  if(state?.busy||posting||index<0||index>availableStep()) return;
  activeStep=index; render();
  $('title-'+steps[index]).focus({preventScroll:true});
}
function renderWizard(ready,busy) {
  if(activeStep>availableStep()) activeStep=availableStep();
  steps.forEach((step,index)=>{
    $('step-'+step).hidden=index!==activeStep;
    const nav=$('nav-'+step), light=$('light-'+step);
    nav.disabled=busy||index>availableStep();
    nav.setAttribute('aria-current',index===activeStep?'step':'false');
    nav.classList.toggle('done',!dirty&&ready[step]);
    nav.classList.toggle('failed',failedStep()===step&&!!state.error||step==='validate'&&state.project?.validation?.visual_result==='misaligned');
    $('nav-state-'+step).textContent=light.textContent.replace('● ','');
  });
  $('step-counter').textContent=`Step ${activeStep+1} of ${steps.length}`;
  $('back').disabled=busy||activeStep===0;
  $('next').hidden=activeStep===steps.length-1;
  $('next').disabled=busy||activeStep>=availableStep();
  $('next').textContent='Next: '+(stepNames[activeStep+1]||'')+' →';
  $('guide-title').textContent=guidance[activeStep][0];
  $('guide-text').textContent=guidance[activeStep][1];
  $('setup-details').hidden=!state.project||dirty;
  $('camera-placeholder').hidden=!$('capture-image').hidden;
}
function message(text, error=false) { $('message').textContent=text; $('message').classList.toggle('error',error); }
function targetFields() {
  for (const kind of ['charuco','aruco']) document.querySelectorAll('.'+kind).forEach(el=>el.hidden=$('kind').value!==kind);
  const index=(defaults?.picking_boards||[]).findIndex(b=>b.target.board_manifest===$('manifest').value&&b.target.board===$('board').value);
  $('picking-board').value=index<0?'':String(index);
}
function tcpText(tcp) {
  return tcp.map((v,i)=>Number((i<3?v*1000:v).toPrecision(12))).join(', ');
}
function showCalibrationTCP() {
  const calibration=defaults.calibrations.find(c=>c.path===$('calibration').value);
  $('calibration-tcp').textContent=calibration
    ? 'Calibration TCP (XYZ mm, rotation-vector rad): '+tcpText(calibration.tcp)
    : 'This TCP must match the TCP recorded in your selected calibration.';
}
function fill(s) {
  for (const id of ['project','calibration','robot_ip','motion_model','approach_mm']) $(id).value=s[id];
  $('max_shift_mm').value=s.limits?.max_shift_mm??100; $('max_rotation_deg').value=s.limits?.max_rotation_deg??180;
  $('camera_provider').value=s.camera_provider||'realsense';
  $('refine_before_pick').checked=s.refine_before_pick??false;
  $('tcp').value=tcpText(s.tcp); $('kind').value=s.target.kind;
  if(s.target.kind==='charuco') { $('manifest').value=s.target.board_manifest; $('board').value=s.target.board; }
  else { for(const id of ['dictionary','marker_id','marker_length_mm']) $(id).value=s.target[id]; }
  targetFields(); showCalibrationTCP();
}
function settings() {
  const kind=$('kind').value;
  const tcp=$('tcp').value.trim().split(/[\s,]+/).map((v,i)=>i<3?Number(v)/1000:Number(v));
  if(tcp.length!==6||!tcp.every(Number.isFinite))
    throw new Error('Active gripper TCP needs six numbers: X, Y, Z in mm, then RX, RY, RZ in rotation-vector radians.');
  return {project:$('project').value,calibration:$('calibration').value,robot_ip:$('robot_ip').value,
    camera_provider:$('camera_provider').value,refine_before_pick:$('refine_before_pick').checked,
    motion_model:$('motion_model').value,approach_mm:Number($('approach_mm').value),
    tcp,tcp_confirmed:$('tcp_confirmed').checked,
    limits:{max_shift_mm:Number($('max_shift_mm').value),max_rotation_deg:Number($('max_rotation_deg').value),max_tilt_change_deg:10,max_height_change_mm:10,max_tool_tilt_deg:20},
    target:kind==='charuco'?{kind,board_manifest:$('manifest').value,board:$('board').value}:
      {kind,dictionary:$('dictionary').value,marker_id:Number($('marker_id').value),marker_length_mm:Number($('marker_length_mm').value)}};
}
async function request(path,body) {
  const response=await fetch(path,body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Teach-Token':token},body:JSON.stringify(body)});
  const value=await response.json(); if(!response.ok) throw new Error(value.error||'Request failed'); return value;
}
function restoreCapture() {
  if(state?.project?.flow.captured) {
    $('capture-image').src='/api/image?v='+(++imageVersion);
    $('capture-image').hidden=false;
  }
}
function poseText(pose) { return pose ? '['+pose.map(v=>v.toFixed(6)).join(', ')+']' : ''; }
function failedStep() {
  if(state?.failed==='return_observation') return activeStep===2?'teach':'validate';
  return ['capture_validation','validate_move'].includes(state?.failed)?'validate':state?.failed;
}
function validationPose(pose) { return pose ? pose.map((v,i)=>`${['X','Y','Z','RX','RY','RZ'][i]}: ${(i<3?v*1000:v).toFixed(i<3?2:5)} ${i<3?'mm':'rad'}`).join('\n') : ''; }
function renderValidation(p,busy) {
  const v=dirty?null:p?.validation;
  const full=p?.settings.motion_model==='full';
  const mode=full?'full_pose':'fixed_z';
  const fresh=v?.validation_mode===mode&&v?.status==='captured'&&Date.now()/1000-v.timestamp>=0&&Date.now()/1000-v.timestamp<=60;
  $('validation-instructions').textContent=full
    ? 'Reposition the marker and object together, or park the AGV at a different position and height. Keep the AGV stopped. From any clear camera view, stop the arm and capture. The test follows the calculated pick in X, Y, Z and orientation.'
    : 'Reposition the marker and object together. From a clear camera view, stop the arm and capture. This planar project follows pick XY and orientation while holding the TCP Z recorded at capture.';
  $('validation-target-title').textContent=full?'Validation target · full 3D pick pose':'Validation target · Z held at capture height';
  $('validation-move').textContent=full?'Move to full 3D pick':'Move at fixed Z';
  $('validation-confirmation-text').textContent=full
    ? 'The AGV and marker/object have stayed fixed since capture. I reviewed all six target coordinates, including Z, the workspace and the clear linear path to the pick endpoint.'
    : 'The AGV and marker/object have stayed fixed since capture. I reviewed the target, workspace and clear linear path at the captured TCP height.';
  $('validation-motion-help').textContent=full
    ? 'One slow linear move to the calculated pick endpoint, including Z and tool orientation. It can lower the tool to the taught endpoint; it does not search for contact or operate the gripper. The approach-height setting does not apply. Keep this tab active. Each attempt requires a fresh capture within 60 seconds.'
    : 'One slow linear move following pick XY and orientation, holding capture Z. The approach-height setting does not apply. Keep this tab active. Each attempt requires a fresh capture within 60 seconds.';
  $('validation-review-help').textContent=full
    ? 'TCP agreement checks arrival at the commanded pose. Independently check the gripper position and orientation against the object to assess camera-derived accuracy.'
    : 'TCP agreement checks motion completion. Visually check alignment at capture height; this test does not validate pick Z.';
  const canReturn=!busy&&!dirty&&connectionOK&&!!p?.observation;
  $('observation-return').disabled=!canReturn;
  $('teach-observation-return').disabled=!canReturn||!p?.flow.taught;
  $('observation-return-speed').disabled=busy;
  const returned=state.return_result;
  const returnText=state.busy==='return_observation'?'Returning to the saved observation joints…':returned
    ? returned.status+(returned.message?' · '+returned.message:'')+(returned.joint_error_deg!==undefined?` · maximum joint error ${returned.joint_error_deg.toFixed(3)}°`:'')
    : '';
  for(const id of ['observation-return-result','teach-observation-return-result']) $(id).textContent=returnText;
  $('validation-capture').disabled=busy||dirty||!connectionOK||!p?.flow.taught;
  $('validation-move').disabled=busy||!connectionOK||!fresh||!$('validation-confirmed').checked;
  $('validation-confirmed').disabled=busy||!fresh;
  for(const id of ['validation-speed','validation-workspace-min','validation-workspace-max']) $(id).disabled=busy;
  $('validation-preview').hidden=!v;
  $('validation-result').hidden=!v||v.status==='captured'||v.status==='moving';
  $('validation-aligned').disabled=$('validation-misaligned').disabled=busy||v?.status!=='completed';
  $('validation-observe-joints').textContent=p?.observation?'Observation joints (degrees): '+p.observe_joints.map(q=>(q*180/Math.PI).toFixed(2)).join(', '):'';
  if(v) {
    $('validation-marker-pose').textContent=validationPose(v.marker_tcp_pose_m_rad);
    $('validation-pick-pose').textContent=validationPose(v.pick_tcp_pose_m_rad);
    $('validation-target-pose').textContent=validationPose(v.validation_tcp_pose_m_rad);
    const r=v.validation;
    $('validation-change').textContent=`Pick shift: ${r.shift_from_taught_mm.toFixed(2)} mm\nPick rotation: ${r.rotation_from_taught_deg.toFixed(2)}°\nMarker yaw: ${r.marker_yaw_change_deg.toFixed(2)}°`;
    if(r.marker_height_change_mm!==undefined) $('validation-change').textContent+=`\nMarker Z change: ${r.marker_height_change_mm.toFixed(2)} mm\nMarker tilt change: ${r.marker_tilt_change_deg.toFixed(2)}°`;
    $('validation-quality').textContent=`${v.quality.frames_accepted} accepted frames · fit ${v.quality.reprojection_rmse_px.toFixed(3)} px`;
    $('validation-freshness').textContent=v.status==='captured'?(fresh?`Ready for one attempt · ${Math.max(0,Math.floor(60-(Date.now()/1000-v.timestamp)))} seconds remaining`:'Capture expired. Capture again before motion.'):'Capture used · capture again for another motion test.';
    $('validation-motion-result').textContent=v.status+(v.message?' · '+v.message:'')+(v.error?`\nTCP error: ${v.error.position_mm.toFixed(2)} mm / ${v.error.rotation_deg.toFixed(3)}°`:'')+(v.visual_result?'\nVisual alignment: '+v.visual_result:'');
    $('validation-actual-pose').textContent=v.actual_tcp_pose_m_rad?'Measured TCP:\n'+validationPose(v.actual_tcp_pose_m_rad):'';
    if(validationImageId!==v.id) {
      validationImageId=v.id; $('validation-confirmed').checked=false; $('validation-measured-error').value='';
      $('validation-move').disabled=true;
      $('validation-image').src='/api/validation/image?v='+encodeURIComponent(v.id);
      $('validation-image').hidden=false;
    }
  } else {
    $('validation-image').hidden=true; $('validation-confirmed').checked=false; validationImageId='';
  }
  $('validation-placeholder').hidden=!$('validation-image').hidden;
  $('validation-history').textContent=(!dirty&&p?.validation_history?.length)?p.validation_history.slice().reverse().map((t,i)=>`${p.validation_history.length-i}. ${new Date(t.timestamp*1000).toLocaleString()} · ${t.status}${t.error?` · ${t.error.position_mm.toFixed(2)} mm / ${t.error.rotation_deg.toFixed(3)}°`:''} · visual: ${t.visual_result||'unreviewed'}${t.measured_position_error_mm!==undefined?` · independently measured: ${t.measured_position_error_mm.toFixed(2)} mm (${t.within_2mm?'within':'outside'} 2 mm)`:''}${t.message?'\n'+t.message:''}`).join('\n'):'No motion tests yet.';
}
function render() {
  if(!state) return;
  $('version').textContent='v'+state.version;
  const p=state.project, f=p?.flow||{}, busy=!!state.busy||posting;
  const ready={configure:!!p,capture:!!f.captured,teach:!!f.taught,locate:!!f.exported,validate:p?.validation?.status==='completed'&&p.validation.visual_result==='aligned'};
  for(const step of steps) {
    const light=$('light-'+step); let status=dirty?'pending':ready[step]?'ready':'pending';
    if(state.busy===step||step==='validate'&&['capture_validation','validate_move'].includes(state.busy)
        ||state.busy==='return_observation'&&step===(activeStep===2?'teach':'validate')) status='running';
    if(state.error&&failedStep()===step) status='failed';
    if(step==='validate'&&p?.validation?.visual_result==='misaligned') status='failed';
    light.className='light '+status; light.textContent='● '+({pending:'Pending',ready:'Ready',running:'Running',failed:'Failed'}[status]);
  }
  $('configure').disabled=busy; $('open').disabled=busy;
  for(const input of $('setup').querySelectorAll('input,select')) input.disabled=busy;
  $('capture').disabled=busy||dirty||!p;
  $('teach').disabled=busy||dirty||!f.captured||!$('marker_unchanged').checked;
  $('locate').disabled=busy||dirty||!f.taught;
  $('marker_unchanged').disabled=busy||!f.captured;
  $('export-result').hidden=dirty||!f.exported; $('saved-links').hidden=dirty||!f.taught;
  renderValidation(p,busy);
  renderWizard(ready,busy);
  if(p&&!dirty) {
    $('movement-limits').textContent=`Configured limits: ${p.settings.limits.max_shift_mm} mm target shift and ${p.settings.limits.max_rotation_deg}° rotation.`;
    $('setup-result').textContent='Calibration '+p.calibration.verdict.toUpperCase()+' · '+p.target+'\nCamera serial: '+(p.calibration.camera_serial||'not recorded')+'\n'+p.project;
    const o=p.observation;
    $('capture-result').textContent=o?`${o.quality.frames_accepted} accepted frames · ${o.quality.corner_count} corners · fit ${o.quality.reprojection_rmse_px.toFixed(3)} px` : '';
    $('observe-joints').textContent=o?'Saved observation joints (degrees): '+p.observe_joints.map(q=>(q*180/Math.PI).toFixed(2)).join(', '):'Saved observation joints appear after capture.';
    $('teach-result').textContent=f.taught?'Saved gripper TCP endpoint (base m/rad):\n'+poseText(p.taught.taught_tcp_in_base)+'\nMarker-relative relationship saved in grasp_pick.json.':'';
    $('storage').textContent='Project files: '+p.project;
    if(p.exported) { $('pick-pose').textContent=poseText(p.exported.pick_tcp_pose_m_rad)+(p.exported.refinement_required?'\nPreview only · use live vision to refine before picking':''); $('approach-pose').textContent=poseText(p.exported.approach_tcp_pose_m_rad); }
  }
}
async function post(action,body) {
  posting=true;
  try {
    previousResult='';
    render();
    if(['validation/move','validation/return'].includes(action)) await request('/api/validation/heartbeat',{});
    await request('/api/'+action,body); dirty=false;
    if(action==='capture') $('marker_unchanged').checked=false;
    message(action==='configure'?'Setup ready. Continue to step 2.':['validation/move','validation/return'].includes(action)?'Motion requested. Keep this tab active; Stop motion is available above.':'Working — keep the robot stationary.');
    await poll();
    if(action==='configure'&&!state.error) { activeStep=availableStep(); restoreCapture(); }
  } catch(e) { message(e.message,true); }
  finally { posting=false; render(); }
}
async function poll() {
  try {
    state=await request('/api/state');
    connectionOK=true;
    const resultKey=JSON.stringify([state.completed,state.failed,state.error]);
    if(!state.busy && resultKey!==previousResult && !dirty) {
      if(state.error) message(state.error,true);
      else if(state.completed) message({configure:'Setup ready. Continue to step 2.',capture:'Marker captured. Keep it fixed and teach the pick endpoint in step 3.',teach:'Picking position saved. Use Return to observation pose before step 4.',locate:'Fresh poses exported. Continue to step 5 to validate tracking.',capture_validation:'Validation capture ready. Review the target and clear path before motion.',validate_move:state.project?.validation?.validation_mode==='full_pose'?'Full 3D motion completed. Check alignment in XYZ and orientation and record the result.':'Fixed-height motion completed. Check visual alignment and record the result.',return_observation:activeStep===2?'Returned to the saved observation joints. Continue to step 4 to read and export.':'Returned to the saved observation joints. Choose Capture for validation.'}[state.completed]);
      if(['capture','locate'].includes(state.completed)||['capture','locate'].includes(state.failed)) {
        $('capture-image').src='/api/image?v='+(++imageVersion); $('capture-image').hidden=false;
      }
    }
    previousResult=resultKey; render();
  } catch(e) { connectionOK=false; render(); message('Teacher connection lost: '+e.message,true); $('capture').disabled=$('teach').disabled=$('locate').disabled=true; }
}
steps.forEach((step,index)=>$('nav-'+step).addEventListener('click',()=>showStep(index)));
$('back').addEventListener('click',()=>showStep(activeStep-1));
$('next').addEventListener('click',()=>showStep(activeStep+1));
$('setup').addEventListener('submit',async e=>{
  e.preventDefault();
  try {
    // Handle validation here so a field inside closed advanced settings cannot
    // silently prevent the submit event and leave the user without feedback.
    const invalid=$('setup').querySelector('input:invalid, select:invalid');
    if(invalid) {
      const details=invalid.closest('details'); if(details) details.open=true;
      const label=invalid.labels?.[0]?.firstChild?.textContent.trim()
        ||invalid.labels?.[0]?.textContent.trim()||invalid.id;
      activeStep=0; render();
      message('Check '+label+': '+invalid.validationMessage,true);
      invalid.focus(); invalid.reportValidity(); return;
    }
    await post('configure',settings());
  } catch(e) { message(e.message,true); }
});
$('open').addEventListener('click',async()=>{
  try {
    const saved=await request('/api/open',{project:$('project').value});
    fill(saved); dirty=true; activeStep=0; $('tcp_confirmed').checked=false;
    message('Saved project loaded. Check its active TCP on the pendant, then complete setup to continue.'); render();
  } catch(e) { message(e.message,true); }
});
$('setup').addEventListener('input',()=>{dirty=true;activeStep=0;targetFields();$('tcp_confirmed').checked=false;message('Setup changed. Check the TCP and complete step 1 again.');render();});
// Checking the declaration itself must not immediately uncheck it.
$('tcp_confirmed').addEventListener('input',e=>{e.stopPropagation();render();});
$('calibration').addEventListener('change',()=>{const item=defaults.calibrations.find(c=>c.path===$('calibration').value);if(item){$('tcp').value=tcpText(item.tcp);$('robot_ip').value=item.robot_ip;}showCalibrationTCP();});
$('kind').addEventListener('change',targetFields);
$('picking-board').addEventListener('input',()=>{
  const value=$('picking-board').value;
  const board=value===''?null:defaults.picking_boards[Number(value)];
  if(board) { $('manifest').value=board.target.board_manifest; $('board').value=board.target.board; }
  targetFields();
});
$('manifest').addEventListener('change',targetFields);
$('board').addEventListener('change',targetFields);
$('marker_unchanged').addEventListener('change',render);
$('capture').addEventListener('click',()=>post('capture',{}));
$('teach').addEventListener('click',()=>post('teach',{marker_unchanged:$('marker_unchanged').checked}));
$('locate').addEventListener('click',()=>post('locate',{}));
$('validation-capture').addEventListener('click',()=>{ $('validation-confirmed').checked=false; post('validation/capture',{}); });
$('validation-confirmed').addEventListener('change',render);
function workspace(id) {
  const text=$(id).value.trim(), values=text.split(/[\s,]+/).map(Number);
  if(!text||values.length!==3||!values.every(Number.isFinite)) throw new Error('Workspace needs three XYZ numbers in millimetres.');
  return values.map(v=>v/1000);
}
for(const id of ['validation-speed','validation-workspace-min','validation-workspace-max','observation-return-speed']) $(id).addEventListener('input',()=>{$('validation-confirmed').checked=false;render();});
async function returnToObservation(){
  try {
    const body={path_confirmed:true,
      joint_speed_deg_s:Number($('observation-return-speed').value),
      workspace_min_m:workspace('validation-workspace-min'),workspace_max_m:workspace('validation-workspace-max')};
    $('validation-confirmed').checked=false;
    state.return_result=null;
    await post('validation/return',body);
  } catch(e) { message(e.message,true); }
}
for(const id of ['observation-return','teach-observation-return']) $(id).addEventListener('click',returnToObservation);
$('validation-move').addEventListener('click',async()=>{
  try {
    const body={capture_id:state.project.validation.id,path_confirmed:$('validation-confirmed').checked,
      speed_mm_s:Number($('validation-speed').value),workspace_min_m:workspace('validation-workspace-min'),workspace_max_m:workspace('validation-workspace-max')};
    $('validation-confirmed').checked=false;
    await post('validation/move',body);
  } catch(e) { message(e.message,true); }
});
async function stopValidation() {
  $('validation-confirmed').checked=false;
  try { await request('/api/validation/stop',{}); await poll(); message('Stop requested. Capture again before another motion attempt.'); }
  catch(e) { message('Stop request could not reach the teacher. Use the pendant stop: '+e.message,true); }
}
$('validation-stop').addEventListener('click',stopValidation);
for(const [id,result] of [['validation-aligned','aligned'],['validation-misaligned','misaligned']]) $(id).addEventListener('click',async()=>{
  try { const text=$('validation-measured-error').value.trim();
    const measured_error_mm=text?text.split(/[\s,]+/).map(Number):null;
    await request('/api/validation/review',{capture_id:state.project.validation.id,result,measured_error_mm}); await poll(); message(result==='aligned'?'Alignment recorded. Reposition and capture again to test another pose.':'Alignment needs adjustment recorded. Review teaching and calibration before another test.'); }
  catch(e) { message(e.message,true); }
});
$('validation-image').addEventListener('error',()=>{$('validation-image').hidden=true;$('validation-placeholder').hidden=false;});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&['validate_move','return_observation'].includes(state?.busy)) stopValidation();});
$('capture-image').addEventListener('error',()=>{$('capture-image').hidden=true;render();});
(async()=>{
  try {
    defaults=await request('/api/defaults');
    for(const [index,board] of (defaults.picking_boards||[]).entries()) {
      const option=document.createElement('option'); option.value=String(index);
      option.textContent=board.target.board+' · '+board.dimensions_mm.join(' × ')+' mm · '+board.dictionary;
      $('picking-board').append(option);
    }
    for(const [id,items] of [['calibrations',defaults.calibrations.map(c=>c.path)],['manifests',defaults.manifests]])
      for(const item of items){const option=document.createElement('option');option.value=item;$(id).append(option);}
    await poll();
    fill(state.project?{...state.project.settings,project:state.project.project}:defaults.settings);
    activeStep=availableStep();
    restoreCapture();
    render();
    setInterval(poll,700);
    setInterval(()=>{if(!document.hidden&&['validate_move','return_observation'].includes(state?.busy)) request('/api/validation/heartbeat',{}).catch(()=>{});},300);
  } catch(e){message(e.message,true);}
})();
