"""PhotonVision-style ChArUco camera calibration panel for the primary dashboard."""

CAMERA_CALIBRATION_PANEL = r"""
<style id="camera-calibration-style">
#camera-calibration-open{min-height:28px;padding:5px 9px;border:1px solid rgba(255,255,255,.12);border-radius:4px;background:#171919;color:#d7dbd9;font:600 9px Inter,system-ui,sans-serif;cursor:pointer}
#camera-calibration-open:hover{background:#1d2020;border-color:rgba(255,255,255,.20)}
#camera-calibration-modal .modal-card{width:min(1080px,calc(100vw - 24px));max-height:calc(100vh - 24px);overflow:hidden;display:grid;grid-template-rows:auto minmax(0,1fr) auto}
.ccal-body{min-height:0;overflow:auto;padding:14px;display:grid;grid-template-columns:minmax(420px,1.35fr) minmax(330px,.9fr);gap:14px}
.ccal-preview-panel,.ccal-control-panel{min-width:0;border:1px solid rgba(255,255,255,.08);border-radius:5px;background:#0e1010}
.ccal-preview-head,.ccal-section-head{min-height:38px;padding:9px 11px;border-bottom:1px solid rgba(255,255,255,.07);display:flex;align-items:center;justify-content:space-between;gap:10px}
.ccal-preview-head strong,.ccal-section-head strong{font-size:11px;color:#e5e8e6}.ccal-preview-head span,.ccal-section-head span{font:600 9px ui-monospace,monospace;color:#858c89}
.ccal-preview-wrap{position:relative;aspect-ratio:16/9;background:#030404;overflow:hidden}#ccal-preview{display:block;width:100%;height:100%;object-fit:contain}
.ccal-preview-badge{position:absolute;left:10px;top:10px;padding:5px 7px;border:1px solid rgba(255,255,255,.12);border-radius:3px;background:rgba(6,8,8,.82);font:650 9px ui-monospace,monospace;color:#d7dbd9}
.ccal-preview-badge.good{color:#bfe0ca;border-color:rgba(85,185,120,.4)}.ccal-preview-badge.warn{color:#e0c486;border-color:rgba(210,168,93,.4)}
.ccal-guidance{padding:10px 11px;border-top:1px solid rgba(255,255,255,.07);color:#aeb4b1;font-size:10px;line-height:1.5}
.ccal-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:4px;padding:10px}.ccal-grid-cell{min-height:38px;border:1px solid rgba(255,255,255,.07);border-radius:3px;background:#141616;display:grid;place-items:center;color:#6f7673;font:700 10px ui-monospace,monospace}.ccal-grid-cell.hit{background:#152019;border-color:rgba(85,185,120,.3);color:#bfe0ca}
.ccal-control-panel{display:grid;align-content:start}.ccal-section{border-bottom:1px solid rgba(255,255,255,.07)}.ccal-section:last-child{border-bottom:0}
.ccal-fields{padding:10px;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:8px}.ccal-field{display:grid;gap:5px}.ccal-field label{font-size:9px;color:#878e8b}.ccal-field input,.ccal-field select{width:100%;min-width:0;padding:7px 8px;border:1px solid rgba(255,255,255,.10);border-radius:4px;background:#090b0b;color:#e4e7e5;font:650 10px ui-monospace,monospace}
.ccal-kpis{padding:10px;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:1px;background:rgba(255,255,255,.055)}.ccal-kpi{padding:9px;background:#101212}.ccal-kpi span{display:block;color:#777f7b;font-size:9px}.ccal-kpi strong{display:block;margin-top:4px;color:#e8ebe9;font:700 14px ui-monospace,monospace}
.ccal-status{padding:10px;color:#989f9b;font-size:9px;line-height:1.5}.ccal-status.good{color:#bfe0ca}.ccal-status.warn{color:#e0c486}.ccal-status.bad{color:#e5a1a6}
.ccal-actions{display:flex;gap:7px;flex-wrap:wrap;padding:10px}.ccal-actions button,.ccal-footer button{min-height:34px;padding:7px 10px;border:1px solid rgba(255,255,255,.11);border-radius:4px;background:#171919;color:#cbd0cd;font:600 9px Inter,system-ui,sans-serif;cursor:pointer}.ccal-actions button.primary,.ccal-footer button.primary{border-color:rgba(85,185,120,.35);background:#142019;color:#c6e2d0}.ccal-actions button.danger{border-color:rgba(212,84,93,.35);background:#251719;color:#e5a1a6}.ccal-actions button:disabled,.ccal-footer button:disabled{opacity:.38;cursor:not-allowed}
.ccal-footer{padding:11px 14px;border-top:1px solid rgba(255,255,255,.08);display:flex;justify-content:space-between;align-items:center;gap:10px;background:#101212}.ccal-footer small{color:#767d79;font-size:9px;line-height:1.4}
@media(max-width:850px){.ccal-body{grid-template-columns:1fr}.ccal-fields{grid-template-columns:1fr}#camera-calibration-modal .modal-card{width:calc(100vw - 12px);max-height:calc(100vh - 12px)}}
</style>

<div class="modal-backdrop" id="camera-calibration-modal" hidden>
  <div class="modal-card">
    <div class="modal-head"><div><strong>카메라 보정 · ChArUco</strong><p>PhotonVision 방식으로 여러 위치·거리·각도의 ChArUco 샘플을 수집해 렌즈 내부 파라미터와 왜곡을 계산합니다.</p></div><button class="modal-close" id="camera-calibration-close" type="button">×</button></div>
    <div class="ccal-body">
      <section class="ccal-preview-panel">
        <div class="ccal-preview-head"><strong>실시간 보드 검출</strong><span id="ccal-preview-meta">대기 중</span></div>
        <div class="ccal-preview-wrap"><canvas id="ccal-preview" width="1280" height="720"></canvas><div class="ccal-preview-badge warn" id="ccal-preview-badge">CHARUCO SEARCHING</div></div>
        <div class="ccal-guidance" id="ccal-guidance">보드를 평평한 판에 고정하고 카메라는 움직이지 마세요. 보드를 화면 전체 위치와 최대 약 45° 범위의 여러 각도로 움직이며 촬영합니다.</div>
        <div class="ccal-section-head"><strong>샘플 화면 분포</strong><span id="ccal-grid-meta">0 / 9 영역</span></div>
        <div class="ccal-grid" id="ccal-grid"><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div><div class="ccal-grid-cell">0</div></div>
      </section>
      <section class="ccal-control-panel">
        <div class="ccal-section">
          <div class="ccal-section-head"><strong>보드 설정</strong><span>출력물 실측값과 일치해야 함</span></div>
          <div class="ccal-fields">
            <div class="ccal-field"><label>Squares X</label><input id="ccal-squares-x" type="number" min="4" max="20" step="1" value="8"></div>
            <div class="ccal-field"><label>Squares Y</label><input id="ccal-squares-y" type="number" min="4" max="20" step="1" value="8"></div>
            <div class="ccal-field"><label>Square size (mm)</label><input id="ccal-square-mm" type="number" min="5" max="200" step="0.1" value="25.4"></div>
            <div class="ccal-field"><label>Marker size (mm)</label><input id="ccal-marker-mm" type="number" min="3" max="199" step="0.1" value="19.05"></div>
            <div class="ccal-field"><label>Dictionary</label><select id="ccal-dictionary"></select></div>
            <div class="ccal-field"><label>ChArUco pattern</label><select id="ccal-legacy"><option value="0">OpenCV 4.6+ / PhotonVision 현재</option><option value="1">Legacy (OpenCV &lt; 4.6)</option></select></div>
          </div>
          <div class="ccal-status" id="ccal-board-help">기본값은 PhotonVision 8×8 / 1-inch square / 0.75-inch marker 계열입니다. 100% 배율로 출력하고 square/marker 실제 크기를 측정해 입력하세요. 오래된 OpenCV 4.6 이전 패턴만 Legacy를 선택합니다.</div>
          <div class="ccal-actions"><button id="ccal-configure" type="button">보드 설정 적용</button></div>
        </div>
        <div class="ccal-section">
          <div class="ccal-section-head"><strong>샘플 수집</strong><span id="ccal-sample-progress">0 / 최소 12 · 권장 50</span></div>
          <div class="ccal-kpis"><div class="ccal-kpi"><span>검출 코너</span><strong id="ccal-corners">0</strong></div><div class="ccal-kpi"><span>현재 품질</span><strong id="ccal-quality">0%</strong></div><div class="ccal-kpi"><span>저장 샘플</span><strong id="ccal-samples">0</strong></div><div class="ccal-kpi"><span>위치 영역</span><strong id="ccal-coverage">0/9</strong></div></div>
          <div class="ccal-actions"><button class="primary" id="ccal-capture" type="button">현재 프레임 촬영</button><button id="ccal-remove-last" type="button">마지막 샘플 삭제</button><button class="danger" id="ccal-reset" type="button">샘플 전체 초기화</button></div>
        </div>
        <div class="ccal-section">
          <div class="ccal-section-head"><strong>보정 결과</strong><span>Reprojection Error</span></div>
          <div class="ccal-kpis"><div class="ccal-kpi"><span>상태</span><strong id="ccal-cal-state">미보정</strong></div><div class="ccal-kpi"><span>RMS</span><strong id="ccal-rms">-</strong></div><div class="ccal-kpi"><span>HFOV</span><strong id="ccal-hfov">-</strong></div><div class="ccal-kpi"><span>사용 샘플</span><strong id="ccal-used">-</strong></div></div>
          <div class="ccal-status" id="ccal-status">상태 불러오기 대기</div>
          <div class="ccal-actions"><button class="primary" id="ccal-solve" type="button">캘리브레이션 계산 및 저장</button></div>
        </div>
      </section>
    </div>
    <div class="ccal-footer"><small>12장은 계산 가능한 최소치이고, PhotonVision은 정확도를 위해 50장 이상을 권장합니다. 보정 후 카메라 위치·줌·해상도를 변경하면 다시 보정하세요.</small><button id="ccal-footer-close" type="button">닫기</button></div>
  </div>
</div>

<script>
(function(){
  const modal=document.getElementById('camera-calibration-modal');
  const camera=document.getElementById('camera-stream');
  if(!modal||!camera)return;
  const cameraHead=document.querySelector('.camera-panel .panel-head');
  if(cameraHead&&!document.getElementById('camera-calibration-open')){
    let actions=cameraHead.querySelector('.panel-head-actions');
    if(!actions){actions=document.createElement('div');actions.className='panel-head-actions';cameraHead.appendChild(actions)}
    const open=document.createElement('button');open.id='camera-calibration-open';open.type='button';open.textContent='카메라 보정';actions.appendChild(open);
  }
  const $=id=>document.getElementById(id),canvas=$('ccal-preview'),ctx=canvas.getContext('2d'),badge=$('ccal-preview-badge'),guidance=$('ccal-guidance'),status=$('ccal-status');
  let state=null,preview=null,pollTimer=null,drawHandle=null,opened=false,requestInFlight=false;
  const num=(id,fallback=0)=>{const value=Number($(id)?.value);return Number.isFinite(value)?value:fallback};
  function boardPayload(){return {squares_x:Math.round(num('ccal-squares-x',8)),squares_y:Math.round(num('ccal-squares-y',8)),square_length_m:num('ccal-square-mm',25.4)/1000,marker_length_m:num('ccal-marker-mm',19.05)/1000,dictionary:$('ccal-dictionary').value||'DICT_4X4_1000',legacy_pattern:$('ccal-legacy').value==='1',minimum_corners:Number(state?.config?.minimum_corners||12),required_samples:Number(state?.config?.required_samples||12),recommended_samples:Math.max(50,Number(state?.config?.recommended_samples||50))}}
  async function api(path,options={}){const response=await fetch(path,{cache:'no-store',...options});let body={};try{body=await response.json()}catch(_error){}if(!response.ok)throw new Error(body.error||`HTTP ${response.status}`);return body}
  function setStatus(message,kind=''){status.textContent=message;status.className=`ccal-status ${kind}`}
  function fillConfig(snapshot){const cfg=snapshot?.config||{};if(document.activeElement?.closest?.('.ccal-field'))return;$('ccal-squares-x').value=cfg.squares_x??8;$('ccal-squares-y').value=cfg.squares_y??8;$('ccal-square-mm').value=((cfg.square_length_m??.0254)*1000).toFixed(2);$('ccal-marker-mm').value=((cfg.marker_length_m??.01905)*1000).toFixed(2);$('ccal-legacy').value=cfg.legacy_pattern?'1':'0';const select=$('ccal-dictionary'),dictionaries=snapshot?.dictionaries||[],current=cfg.dictionary||'DICT_4X4_1000';if(select.dataset.loaded!==dictionaries.join('|')){select.innerHTML='';for(const name of dictionaries){const option=document.createElement('option');option.value=name;option.textContent=name;select.appendChild(option)}select.dataset.loaded=dictionaries.join('|')}if([...select.options].some(option=>option.value===current))select.value=current}
  function renderGrid(snapshot){const values=snapshot?.coverage_grid||Array(9).fill(0),cells=[...$('ccal-grid').children];cells.forEach((cell,index)=>{const value=Number(values[index]||0);cell.textContent=String(value);cell.classList.toggle('hit',value>0)});const occupied=Number(snapshot?.occupied_coverage_cells||0);$('ccal-grid-meta').textContent=`${occupied} / 9 영역`;$('ccal-coverage').textContent=`${occupied}/9`}
  function rmsLabel(value){return Number.isFinite(value)?`${value.toFixed(value<1?3:2)} px`:'-'}
  function renderState(snapshot){state=snapshot;fillConfig(snapshot);renderGrid(snapshot);const count=Number(snapshot?.sample_count||0),required=Number(snapshot?.required_samples||12),recommended=Math.max(50,Number(snapshot?.recommended_samples||50));$('ccal-samples').textContent=String(count);$('ccal-sample-progress').textContent=`${count} / 최소 ${required} · 권장 ${recommended}`;const cal=snapshot?.calibration||{},usable=cal.vision_usable!==false&&Boolean(cal.calibrated);$('ccal-cal-state').textContent=!cal.calibrated?'미보정':usable?'적용 가능':'보정값 거부';$('ccal-rms').textContent=rmsLabel(Number(cal.rms_error));$('ccal-hfov').textContent=Number.isFinite(Number(cal.horizontal_fov_degrees))?`${Number(cal.horizontal_fov_degrees).toFixed(1)}°`:'-';$('ccal-used').textContent=cal.samples?String(cal.samples):'-';const editable=Boolean(snapshot?.editable),block=snapshot?.edit_block_reason;$('ccal-configure').disabled=!editable;$('ccal-capture').disabled=!editable||!preview?.valid||!snapshot?.aruco_available;$('ccal-remove-last').disabled=!editable||count<1;$('ccal-reset').disabled=!editable||count<1;$('ccal-solve').disabled=!editable||!snapshot?.ready_to_calibrate;if(!snapshot?.aruco_available){setStatus('cv2.aruco가 없습니다. opencv-contrib-python-headless 설치가 필요합니다.','bad')}else if(!editable){setStatus(block||'차량을 정지한 뒤 보정 작업을 진행하세요.','warn')}else if(cal.calibrated&&!usable){setStatus(`보정 파일은 저장됐지만 RMS ${rmsLabel(Number(cal.rms_error))}로 1px 기준을 넘어서 차선/비전에는 적용하지 않습니다. 샘플을 다시 수집하세요.`,'bad')}else if(snapshot?.warnings?.length){setStatus(snapshot.warnings.join(' '),'warn')}else if(cal.calibrated){setStatus(`현재 보정값 적용 중 · ${cal.quality||'USABLE'} · RMS ${rmsLabel(Number(cal.rms_error))} · 사용 샘플 ${cal.samples||'-'}/${recommended} 권장`,count<recommended?'warn':'good')}else if(count>=required&&count<recommended){setStatus(`계산은 가능하지만 ${count}장은 최소 수준입니다. 가능하면 ${recommended}장까지 다양한 위치·거리·각도를 추가하세요.`,'warn')}else{setStatus('ChArUco 샘플을 화면 전체 위치와 다양한 각도에서 수집하세요.') }}
  function drawPreview(){if(!opened)return;const w=Math.max(2,camera.naturalWidth||1280),h=Math.max(2,camera.naturalHeight||720);if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h}try{ctx.drawImage(camera,0,0,w,h)}catch(_error){}const result=preview;if(result?.image_size&&Array.isArray(result.charuco_corners)){const sx=w/Math.max(1,Number(result.image_size[0])),sy=h/Math.max(1,Number(result.image_size[1]));ctx.save();ctx.lineWidth=Math.max(1.5,w/700);for(const marker of result.marker_corners||[]){const points=marker?.corners||[];if(points.length<4)continue;ctx.strokeStyle='rgba(76,226,208,.85)';ctx.beginPath();points.forEach((point,index)=>{const x=Number(point[0])*sx,y=Number(point[1])*sy;if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.closePath();ctx.stroke()}const ids=result.charuco_ids||[];(result.charuco_corners||[]).forEach((point,index)=>{const x=Number(point[0])*sx,y=Number(point[1])*sy;ctx.fillStyle=result.valid?'#9dcc82':'#d5b878';ctx.beginPath();ctx.arc(x,y,Math.max(2,w/500),0,Math.PI*2);ctx.fill();if(index%7===0&&ids[index]!==undefined){ctx.fillStyle='rgba(240,242,237,.9)';ctx.font=`${Math.max(9,Math.round(w/120))}px ui-monospace,monospace`;ctx.fillText(String(ids[index]),x+4,y-4)}});ctx.restore()}drawHandle=requestAnimationFrame(drawPreview)}
  function renderPreview(result){preview=result;const corners=Number(result?.detected_corners||0),quality=Math.max(0,Math.min(1,Number(result?.quality||0)));$('ccal-corners').textContent=String(corners);$('ccal-quality').textContent=`${Math.round(quality*100)}%`;$('ccal-preview-meta').textContent=result?.frame_sequence!==undefined?`FRAME ${result.frame_sequence??'-'}`:'대기 중';guidance.textContent=result?.guidance||'ChArUco 보드를 카메라에 보여주세요.';if(result?.valid){badge.textContent=`CHARUCO LOCK · ${corners} CORNERS · Q${Math.round(quality*100)}%`;badge.className='ccal-preview-badge good'}else{badge.textContent=`CHARUCO SEARCHING · ${corners} CORNERS`;badge.className='ccal-preview-badge warn'}if(state)renderState(state)}
  async function refreshState(){try{renderState(await api('/api/camera/calibration'))}catch(error){setStatus(String(error.message||error),'bad')}}
  async function refreshPreview(){if(!opened||requestInFlight||document.hidden)return;requestInFlight=true;try{renderPreview(await api('/api/camera/calibration/preview'))}catch(error){guidance.textContent=String(error.message||error);badge.textContent='CHARUCO PREVIEW ERROR';badge.className='ccal-preview-badge warn'}finally{requestInFlight=false}}
  function start(){if(opened)return;opened=true;modal.hidden=false;refreshState();refreshPreview();pollTimer=setInterval(refreshPreview,450);drawPreview()}function stop(){opened=false;modal.hidden=true;if(pollTimer){clearInterval(pollTimer);pollTimer=null}if(drawHandle){cancelAnimationFrame(drawHandle);drawHandle=null}}
  $('camera-calibration-open')?.addEventListener('click',start);$('camera-calibration-close').addEventListener('click',stop);$('ccal-footer-close').addEventListener('click',stop);modal.addEventListener('click',event=>{if(event.target===modal)stop()});
  $('ccal-configure').addEventListener('click',async()=>{try{renderState(await api('/api/camera/calibration/configure',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({config:boardPayload()})}));setStatus('보드 설정을 적용했습니다.','good')}catch(error){setStatus(String(error.message||error),'bad')}});
  $('ccal-capture').addEventListener('click',async()=>{const button=$('ccal-capture');button.disabled=true;try{const result=await api('/api/camera/calibration/capture',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});renderState(result);const novelty=Number(result?.captured?.novelty);setStatus(Number.isFinite(novelty)&&novelty<.12?'샘플 저장됨 · 이전 샘플과 비슷합니다. 다음에는 위치/거리/각도를 크게 바꾸세요.':'샘플 저장됨 · 보드를 다른 위치/거리/각도로 옮겨 다음 샘플을 촬영하세요.',Number.isFinite(novelty)&&novelty<.12?'warn':'good')}catch(error){setStatus(String(error.message||error),'bad')}finally{if(state)renderState(state)}});
  $('ccal-remove-last').addEventListener('click',async()=>{try{renderState(await api('/api/camera/calibration/remove-last',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}));setStatus('마지막 샘플을 삭제했습니다.')}catch(error){setStatus(String(error.message||error),'bad')}});
  $('ccal-reset').addEventListener('click',async()=>{if(!confirm('촬영한 ChArUco 샘플을 모두 삭제할까요? 현재 저장된 카메라 보정값은 유지됩니다.'))return;try{renderState(await api('/api/camera/calibration/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}));setStatus('촬영 샘플을 모두 초기화했습니다.')}catch(error){setStatus(String(error.message||error),'bad')}});
  $('ccal-solve').addEventListener('click',async()=>{const button=$('ccal-solve');button.disabled=true;setStatus('카메라 파라미터 계산 중…');try{const result=await api('/api/camera/calibration/solve',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});renderState(result);const cal=result?.calibration||{},rms=Number(cal.rms_error);if(cal.vision_usable===false){setStatus(`보정 계산됨 · RMS ${rmsLabel(rms)} · 1px 기준 초과로 비전 적용은 차단했습니다. 샘플을 다시 촬영하세요.`,'bad')}else{setStatus(`보정 완료 · RMS ${rmsLabel(rms)} · 차선/비전에 적용되었습니다.`,Number.isFinite(rms)&&rms<=.5?'good':'warn')}}catch(error){setStatus(String(error.message||error),'bad')}finally{if(state)renderState(state)}});
})();
</script>
""".encode("utf-8")

__all__ = ["CAMERA_CALIBRATION_PANEL"]
