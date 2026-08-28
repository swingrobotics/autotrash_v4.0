"""Vehicle tuning popup injected into the V1 system/settings drawer."""

VEHICLE_SETTINGS_PANEL = r'''
<style>
.vehicle-settings-entry{border-color:#4b2020!important;background:#0d0d0d!important}
.vehicle-settings-entry p{margin:0;color:#8f8f8f;font-size:10px;line-height:1.5}
#vehicle-settings-open{padding:6px 10px;border:1px solid #7a2d2d;border-radius:7px;background:#171010;color:#f0dede;font:750 9px ui-monospace,monospace;cursor:pointer}
#vehicle-settings-open:hover{border-color:#b22222;background:#231111;color:#fff}
#vehicle-settings-modal .modal-card{width:min(960px,calc(100vw - 24px));max-height:calc(100vh - 30px);overflow:hidden;display:grid;grid-template-rows:auto minmax(0,1fr) auto}
.vehicle-settings-body{min-height:0;overflow:auto;padding-right:5px;display:grid;gap:12px}
.vehicle-settings-section{padding:12px;border:1px solid #351919;border-radius:9px;background:#0b0b0b}
.vehicle-settings-section.geometry{border-color:#293d48;background:#0a0f12}.vehicle-settings-section.geometry h3{color:#b9d7e6}.vehicle-settings-section.geometry .vehicle-setting{border-color:#253944;background:#0d1418}
.vehicle-settings-section.safety{border-color:#60421f;background:#100d08}.vehicle-settings-section.safety h3{color:#e9c982}.vehicle-settings-section.safety .vehicle-setting{border-color:#493a25;background:#12100b}
.vehicle-settings-section h3{margin:0 0 3px;color:#f2eeee;font-size:11px;letter-spacing:.06em}.vehicle-settings-section>.section-help{margin:0 0 10px;color:#777;font-size:8px;line-height:1.4}
.vehicle-settings-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}
.vehicle-setting{min-width:0;padding:9px;border:1px solid #2e1b1b;border-radius:7px;background:#101010;display:grid;gap:6px;transition:border-color .12s ease,background .12s ease}.vehicle-setting.changed{border-color:#665c2e;background:#17150b}.vehicle-setting.invalid{border-color:#8b3030;background:#1a0b0b}.vehicle-setting.locked{opacity:.6}
.vehicle-setting label{display:flex;align-items:center;justify-content:space-between;gap:10px;color:#b8b0b0;font-size:9px}.vehicle-setting label span:last-child{color:#747474;font:700 8px ui-monospace,monospace}
.vehicle-setting input{width:100%;min-width:0;padding:8px 9px;border:1px solid #442222;border-radius:6px;background:#070707;color:#f2f2f2;font:750 11px ui-monospace,monospace;outline:none}.vehicle-setting input:focus{border-color:#b22222}.vehicle-setting input:disabled{opacity:.6;cursor:not-allowed}
.vehicle-setting small{color:#6f6f6f;font-size:8px;line-height:1.35}
.vehicle-geometry-note{margin-top:9px;padding:8px 9px;border:1px dashed #2b4451;border-radius:6px;color:#8ea7b3;background:#0a1115;font-size:8px;line-height:1.5}
.vehicle-settings-status{min-height:34px;padding:9px 10px;border:1px solid #352020;border-radius:7px;background:#080808;color:#9d9d9d;font:700 9px ui-monospace,monospace;line-height:1.45}.vehicle-settings-status.good{color:#66d589;border-color:#215f37;background:#07130a}.vehicle-settings-status.warn{color:#e0a7a7;border-color:#6f2929;background:#170909}
.vehicle-settings-dirty{min-height:30px;padding:7px 10px;border:1px dashed #343434;border-radius:7px;color:#858585;font:700 8px ui-monospace,monospace;line-height:1.45}.vehicle-settings-dirty.changed{border-color:#6d5f2f;color:#dbc579;background:#151207}.vehicle-settings-dirty.invalid{border-color:#743030;color:#e4a1a1;background:#160909}
.vehicle-settings-actions{padding-top:12px;display:flex;justify-content:flex-end;gap:8px;flex-wrap:wrap}.vehicle-settings-actions button{min-height:36px;padding:7px 12px;border:1px solid #493030;border-radius:7px;background:#111;color:#ddd;font:750 9px ui-monospace,monospace;cursor:pointer}.vehicle-settings-actions button.primary{border-color:#287b45;background:#0b2814;color:#8ce2aa}.vehicle-settings-actions button:disabled{opacity:.45;cursor:not-allowed}
@media(max-width:650px){.vehicle-settings-grid{grid-template-columns:1fr}#vehicle-settings-modal .modal-card{width:calc(100vw - 12px);max-height:calc(100vh - 12px)}.vehicle-settings-actions{justify-content:stretch}.vehicle-settings-actions button{flex:1}}
</style>

<div class="modal-backdrop" id="vehicle-settings-modal" hidden>
  <div class="modal-card">
    <div class="modal-head">
      <div>
        <strong>차량 설정</strong>
        <p>차체 기하 · 주행 성능 · 조향 반응 · 안전 한계 · 센서 필터를 저장하고 즉시 적용합니다.</p>
      </div>
      <button class="modal-close" id="vehicle-settings-close" type="button">×</button>
    </div>
    <div class="vehicle-settings-body">
      <div class="vehicle-settings-status" id="vehicle-settings-status">설정 불러오기 대기</div>
      <div class="vehicle-settings-dirty" id="vehicle-settings-dirty">변경 없음</div>

      <section class="vehicle-settings-section geometry">
        <h3>차체 기하</h3><p class="section-help">자율주행 조향·차량 footprint·향후 odometry에 사용하는 실제 차량 치수입니다. 차량을 정지시킨 상태에서 실측해 입력하세요.</p>
        <div class="vehicle-settings-grid">
          <div class="vehicle-setting"><label><span>휠베이스</span><span>mm</span></label><input id="vs-wheelbase-mm" type="number" min="200" max="2000" step="1"><small>앞차축 중심선 ↔ 뒤차축 중심선. 현재 PurePursuit/Bicycle 조향 계산에 직접 사용됩니다.</small></div>
          <div class="vehicle-setting"><label><span>앞 윤거</span><span>mm</span></label><input id="vs-front-track-mm" type="number" min="0" max="2000" step="1"><small>앞바퀴 좌/우 회전 중심 사이 거리. 0은 아직 미측정.</small></div>
          <div class="vehicle-setting"><label><span>뒤 윤거</span><span>mm</span></label><input id="vs-rear-track-mm" type="number" min="0" max="2000" step="1"><small>뒷바퀴 좌/우 회전 중심 사이 거리. 0은 아직 미측정.</small></div>
          <div class="vehicle-setting"><label><span>바퀴 직경</span><span>mm</span></label><input id="vs-wheel-diameter-mm" type="number" min="0" max="1000" step="1"><small>하중이 걸린 실제 타이어 외경. RPM/encoder 기반 속도·거리 계산용 기준값.</small></div>
          <div class="vehicle-setting"><label><span>실제 굴림 둘레</span><span>mm/rev</span></label><input id="vs-wheel-circumference-mm" type="number" min="0" max="4000" step="1"><small>바퀴에 표시하고 지면에서 정확히 1회전 굴려 이동한 거리. odometry에서는 직경보다 이 값을 우선 사용하기 좋습니다.</small></div>
          <div class="vehicle-setting"><label><span>차량 전체 폭</span><span>mm</span></label><input id="vs-vehicle-width-mm" type="number" min="0" max="3000" step="1"><small>좌우 가장 돌출된 부분 사이 최대 폭. 향후 footprint/장애물 여유폭 기준.</small></div>
          <div class="vehicle-setting"><label><span>차량 전체 길이</span><span>mm</span></label><input id="vs-vehicle-length-mm" type="number" min="0" max="5000" step="1"><small>앞뒤 가장 돌출된 부분 사이 최대 길이. 향후 footprint/회전 clearance 기준.</small></div>
        </div>
        <div class="vehicle-geometry-note" id="vs-geometry-note">휠베이스는 기존 530 mm 기본값으로 시작합니다. 나머지 0 값은 미측정을 뜻하며 임의 추정값으로 제어에 사용하지 않습니다.</div>
      </section>

      <section class="vehicle-settings-section">
        <h3>주행 성능</h3><p class="section-help">사람이 운전할 때의 최대 출력과 출발 응답을 조정합니다.</p>
        <div class="vehicle-settings-grid">
          <div class="vehicle-setting"><label><span>MANUAL / RECORD 최대 출력</span><span>%</span></label><input id="vs-manual-max" type="number" min="0" max="100" step="5"><small>게임패드 스틱 100%일 때 사용할 최대 구동 출력.</small></div>
          <div class="vehicle-setting"><label><span>출발 최소 PWM</span><span>PWM</span></label><input id="vs-motor-min-pwm" type="number" min="0" max="255" step="1"><small>모터가 정지상태에서 움직이기 위한 최소 출력.</small></div>
          <div class="vehicle-setting"><label><span>출발 Boost 시간</span><span>sec</span></label><input id="vs-motor-boost" type="number" min="0" max="2" step="0.05"><small>출발 순간 최소 PWM을 적용하는 시간.</small></div>
        </div>
      </section>

      <section class="vehicle-settings-section">
        <h3>조향 반응</h3><p class="section-help">조향모터의 출력, 목표각 추종 강도와 응답 속도를 조정합니다.</p>
        <div class="vehicle-settings-grid">
          <div class="vehicle-setting"><label><span>조향 최대 PWM</span><span>PWM</span></label><input id="vs-steer-max-pwm" type="number" min="35" max="255" step="1"><small>목표각으로 이동할 때 조향모터 최대 출력.</small></div>
          <div class="vehicle-setting"><label><span>조향 최소 PWM</span><span>PWM</span></label><input id="vs-steer-min-pwm" type="number" min="0" max="255" step="1"><small>미세 조향 시에도 모터가 움직이도록 하는 최소 출력.</small></div>
          <div class="vehicle-setting"><label><span>조향 반응도 Kp</span><span>Kp</span></label><input id="vs-steer-kp" type="number" min="0.5" max="20" step="0.5"><small>목표각 오차에 따라 PWM이 증가하는 비율.</small></div>
          <div class="vehicle-setting"><label><span>목표각 허용오차</span><span>deg</span></label><input id="vs-steer-tolerance" type="number" min="0.2" max="5" step="0.1"><small>이 범위 안에 들어오면 조향 출력 정지.</small></div>
          <div class="vehicle-setting"><label><span>목표각 이동속도</span><span>deg/s</span></label><input id="vs-steer-rate" type="number" min="10" max="720" step="10"><small>소프트웨어 목표각 이동 속도. 실제 구동 속도는 모터/피드백/기계 한계로 제한됩니다.</small></div>
        </div>
      </section>

      <section class="vehicle-settings-section safety">
        <h3>안전 한계</h3><p class="section-help">AUTO 출력 상한, command watchdog, LiDAR 충돌 판단 기준입니다. 변경 후에는 저속/바퀴 띄움 검증이 필요합니다.</p>
        <div class="vehicle-settings-grid">
          <div class="vehicle-setting"><label><span>AUTO 최대 안전 출력</span><span>%</span></label><input id="vs-auto-max" type="number" min="0" max="100" step="5"><small>AUTO_AI / AUTO_GPS / AUTO_LOCAL / AUTO 출력의 최종 안전 상한.</small></div>
          <div class="vehicle-setting"><label><span>구동 명령 Timeout</span><span>sec</span></label><input id="vs-motor-timeout" type="number" min="0.1" max="2" step="0.05"><small>명령 heartbeat가 끊겼을 때 모터를 정지시키는 시간.</small></div>
          <div class="vehicle-setting"><label><span>STOP 거리</span><span>m</span></label><input id="vs-lidar-stop" type="number" min="0.2" max="5" step="0.05"><small>강제 정지 기준. STOP &lt; CRAWL &lt; SLOW 순서여야 합니다.</small></div>
          <div class="vehicle-setting"><label><span>CRAWL 거리</span><span>m</span></label><input id="vs-lidar-crawl" type="number" min="0.25" max="6" step="0.05"><small>저속/재시작 보호 구간.</small></div>
          <div class="vehicle-setting"><label><span>SLOW 거리</span><span>m</span></label><input id="vs-lidar-slow" type="number" min="0.3" max="10" step="0.05"><small>감속 판단을 시작하는 거리.</small></div>
          <div class="vehicle-setting"><label><span>충돌 판정 반폭</span><span>m</span></label><input id="vs-lidar-width" type="number" min="0.2" max="1.5" step="0.01"><small>차량 중심선 좌우로 장애물을 검사할 폭의 절반. 차량 전체 폭과는 별도 안전 여유값입니다.</small></div>
        </div>
      </section>

      <section class="vehicle-settings-section">
        <h3>센서 필터</h3><p class="section-help">IMU 미세 노이즈와 회전 판정 임계값을 조정합니다.</p>
        <div class="vehicle-settings-grid">
          <div class="vehicle-setting"><label><span>Heading Deadband</span><span>deg</span></label><input id="vs-imu-heading" type="number" min="0" max="10" step="0.1"><small>이보다 작은 heading 변화는 노이즈로 무시.</small></div>
          <div class="vehicle-setting"><label><span>Attitude Deadband</span><span>deg</span></label><input id="vs-imu-attitude" type="number" min="0" max="5" step="0.05"><small>roll/pitch 미세 떨림 무시 범위.</small></div>
          <div class="vehicle-setting"><label><span>회전 판정 Yaw-rate</span><span>deg/s</span></label><input id="vs-imu-turn-rate" type="number" min="0.1" max="30" step="0.1"><small>LEFT/RIGHT 회전 상태를 판단하는 yaw-rate 기준.</small></div>
        </div>
      </section>
    </div>
    <div class="vehicle-settings-actions">
      <button id="vehicle-settings-defaults" type="button">기본값 미리보기</button>
      <button id="vehicle-settings-reload" type="button">저장값 다시 불러오기</button>
      <button class="primary" id="vehicle-settings-save" type="button">저장 및 즉시 적용</button>
    </div>
  </div>
</div>

<script>
(function(){
  const details=document.querySelector('.details-content');
  const modal=document.getElementById('vehicle-settings-modal');
  if(!details||!modal)return;

  const entry=document.createElement('section');
  entry.className='drawer-section vehicle-settings-entry';
  entry.innerHTML='<div class="drawer-section-head"><span>차량 설정</span><button id="vehicle-settings-open" type="button">설정 열기</button></div><p>차체 기하, 주행 성능, 조향 반응, 안전 한계, 센서 필터를 한 곳에서 조정합니다.</p>';
  details.insertBefore(entry,details.firstChild);

  const $=id=>document.getElementById(id);
  const status=$('vehicle-settings-status');
  const dirtyStatus=$('vehicle-settings-dirty');
  const save=$('vehicle-settings-save');
  const defaultsButton=$('vehicle-settings-defaults');
  const geometryNote=$('vs-geometry-note');
  let latest=null;
  let baseline=null;
  const fields={
    wheelbase_m:['vs-wheelbase-mm',1000],
    front_track_width_m:['vs-front-track-mm',1000],
    rear_track_width_m:['vs-rear-track-mm',1000],
    wheel_diameter_m:['vs-wheel-diameter-mm',1000],
    wheel_rolling_circumference_m:['vs-wheel-circumference-mm',1000],
    vehicle_width_m:['vs-vehicle-width-mm',1000],
    vehicle_length_m:['vs-vehicle-length-mm',1000],
    manual_max_throttle:['vs-manual-max',100],
    auto_max_throttle:['vs-auto-max',100],
    motor_min_pwm:['vs-motor-min-pwm',1],
    motor_start_boost_seconds:['vs-motor-boost',1],
    motor_timeout_seconds:['vs-motor-timeout',1],
    steer_manual_pwm:['vs-steer-max-pwm',1],
    steer_min_pwm:['vs-steer-min-pwm',1],
    steer_control_kp:['vs-steer-kp',1],
    steer_target_tolerance_degrees:['vs-steer-tolerance',1],
    steer_target_rate_dps:['vs-steer-rate',1],
    lidar_stop_distance_m:['vs-lidar-stop',1],
    lidar_crawl_distance_m:['vs-lidar-crawl',1],
    lidar_slow_distance_m:['vs-lidar-slow',1],
    lidar_safety_half_width_m:['vs-lidar-width',1],
    imu_heading_deadband_degrees:['vs-imu-heading',1],
    imu_attitude_deadband_degrees:['vs-imu-attitude',1],
    imu_turn_rate_threshold_dps:['vs-imu-turn-rate',1],
  };
  const geometryKeys=['wheelbase_m','front_track_width_m','rear_track_width_m','wheel_diameter_m','wheel_rolling_circumference_m','vehicle_width_m','vehicle_length_m'];

  function setStatus(message,kind=''){status.textContent=message;status.className=`vehicle-settings-status ${kind}`}
  function valueEquals(a,b){return Number.isFinite(Number(a))&&Number.isFinite(Number(b))&&Math.abs(Number(a)-Number(b))<1e-9}
  function fill(values){Object.entries(fields).forEach(([key,[id,scale]])=>{const element=$(id);if(element&&values&&values[key]!==undefined)element.value=Number(values[key])*scale});updateEditingState()}
  function collect(){const result={};Object.entries(fields).forEach(([key,[id,scale]])=>{result[key]=Number($(id).value)/scale});return result}
  function dirtyKeys(){if(!baseline)return[];const current=collect();return Object.keys(fields).filter(key=>!valueEquals(current[key],baseline[key]))}
  function clearInvalid(){Object.values(fields).forEach(([id])=>$(id)?.closest('.vehicle-setting')?.classList.remove('invalid'))}
  function markInvalid(keys){for(const key of keys){const id=fields[key]?.[0];if(id)$(id)?.closest('.vehicle-setting')?.classList.add('invalid')}}
  function validate(){
    clearInvalid();const errors=[];const invalidKeys=[];const values=collect();
    for(const [key,[id]] of Object.entries(fields)){const el=$(id);if(!el||!Number.isFinite(values[key])||!el.checkValidity()){invalidKeys.push(key);errors.push(`${el?.closest('.vehicle-setting')?.querySelector('label span')?.textContent||key} 범위 확인`)}}
    if(Number.isFinite(values.steer_min_pwm)&&Number.isFinite(values.steer_manual_pwm)&&values.steer_min_pwm>values.steer_manual_pwm){invalidKeys.push('steer_min_pwm','steer_manual_pwm');errors.push('조향 최소 PWM은 최대 PWM 이하여야 합니다.')}
    if([values.lidar_stop_distance_m,values.lidar_crawl_distance_m,values.lidar_slow_distance_m].every(Number.isFinite)&&!(values.lidar_stop_distance_m<values.lidar_crawl_distance_m&&values.lidar_crawl_distance_m<values.lidar_slow_distance_m)){invalidKeys.push('lidar_stop_distance_m','lidar_crawl_distance_m','lidar_slow_distance_m');errors.push('LiDAR 거리는 STOP < CRAWL < SLOW 순서여야 합니다.')}
    markInvalid([...new Set(invalidKeys)]);return [...new Set(errors)];
  }
  function updateGeometryNote(){
    if(!geometryNote)return;const values=collect();const missing=geometryKeys.filter(key=>key!=='wheelbase_m'&&!(Number(values[key])>0));
    const diameter=Number(values.wheel_diameter_m||0),circ=Number(values.wheel_rolling_circumference_m||0);
    let suffix='';if(diameter>0&&circ>0){const ideal=Math.PI*diameter;const diff=Math.abs(circ-ideal)/ideal*100;suffix=` · 굴림둘레와 π×직경 차이 ${diff.toFixed(1)}%`}
    geometryNote.textContent=missing.length?`미측정 ${missing.length}개 · 0 값은 아직 보정값으로 사용하지 않습니다.${suffix}`:`차체 기하 7개 입력 완료${suffix}`;
  }
  function setLocked(locked){Object.values(fields).forEach(([id])=>{const input=$(id);if(input){input.disabled=locked;input.closest('.vehicle-setting')?.classList.toggle('locked',locked)}});if(defaultsButton)defaultsButton.disabled=locked}
  function updateEditingState(){
    const changed=dirtyKeys();const changedSet=new Set(changed);Object.entries(fields).forEach(([key,[id]])=>$(id)?.closest('.vehicle-setting')?.classList.toggle('changed',changedSet.has(key)));
    const errors=validate();updateGeometryNote();dirtyStatus.className='vehicle-settings-dirty';
    if(errors.length){dirtyStatus.classList.add('invalid');dirtyStatus.textContent=`입력 확인 · ${errors.join(' / ')}`}
    else if(changed.length){dirtyStatus.classList.add('changed');dirtyStatus.textContent=`${changed.length}개 값 변경됨 · 아직 저장되지 않음`}
    else dirtyStatus.textContent='변경 없음';
    save.disabled=!latest?.editable||errors.length>0||changed.length===0;
  }
  async function api(path,options={}){const response=await fetch(path,{cache:'no-store',...options});let body={};try{body=await response.json()}catch{}if(!response.ok)throw new Error(body.error||`HTTP ${response.status}`);return body}
  async function load(force=false){
    if(!force&&dirtyKeys().length&&!window.confirm('저장하지 않은 변경사항을 버리고 저장값을 다시 불러올까요?'))return;
    setStatus('차량 설정을 불러오는 중...');save.disabled=true;
    try{latest=await api('/api/vehicle/settings');baseline={...latest.settings};fill(latest.settings);setLocked(!latest.editable);setStatus(latest.editable?'정지 상태 · 편집 가능':latest.edit_block_reason||'현재 주행 상태에서는 변경할 수 없습니다.',latest.editable?'good':'warn');updateEditingState()}
    catch(error){setStatus(error.message,'warn');save.disabled=true;setLocked(true)}
  }
  function close(){if(dirtyKeys().length&&!window.confirm('저장하지 않은 변경사항이 있습니다. 닫을까요?'))return;modal.hidden=true}

  $('vehicle-settings-open').addEventListener('click',()=>{modal.hidden=false;load(true)});
  $('vehicle-settings-close').addEventListener('click',close);
  modal.addEventListener('click',event=>{if(event.target===modal)close()});
  document.addEventListener('keydown',event=>{if(event.key==='Escape'&&!modal.hidden)close()});
  $('vehicle-settings-reload').addEventListener('click',()=>load(false));
  defaultsButton.addEventListener('click',()=>{if(latest?.defaults){fill(latest.defaults);setStatus('기본값 미리보기 · 저장 전에는 적용되지 않습니다.');updateEditingState()}});
  Object.values(fields).forEach(([id])=>$(id)?.addEventListener('input',updateEditingState));
  save.addEventListener('click',async()=>{
    const errors=validate();if(save.disabled||errors.length)return;
    save.disabled=true;setStatus('저장하고 즉시 적용하는 중...');
    try{latest=await api('/api/vehicle/settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({settings:collect()})});baseline={...latest.settings};fill(latest.settings);setLocked(!latest.editable);setStatus('저장 완료 · 차체 기하 포함 현재 런타임에 즉시 적용됨','good');updateEditingState()}
    catch(error){setStatus(error.message,'warn');updateEditingState()}
  });
})();
</script>
'''.encode('utf-8')

__all__ = ["VEHICLE_SETTINGS_PANEL"]
