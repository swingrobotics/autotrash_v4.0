'''User-facing recording management for the unified dashboard.'''

import camera_calibration_panel as _camera_calibration_panel
import server_v2_release as _release

from user_management_hmi import USER_MANAGEMENT_HMI
from record_replay_auto_hmi import RECORD_REPLAY_AUTO_HMI
from record_replay_offline_ufld_hmi import RECORD_REPLAY_OFFLINE_UFLD_HMI
from record_replay_candidate_overlay_hmi import RECORD_REPLAY_CANDIDATE_OVERLAY_HMI
from gps_model_lifecycle_hmi import GPS_MODEL_LIFECYCLE_HMI
from settings_page_shell import SETTINGS_PAGE_SHELL
from settings_hmi_polish import SETTINGS_HMI_POLISH
from ai_model_selection_hmi import AI_MODEL_SELECTION_HMI
from compute_worker_hmi import COMPUTE_WORKER_HMI
from compute_training_hmi import COMPUTE_TRAINING_HMI
from compute_gps_training_hmi import COMPUTE_GPS_TRAINING_HMI
from compute_rover_api import install_compute_rover_api
from compute_install_policy import install_compute_candidate_policy
from compute_gps_training_bridge import install_compute_gps_training_bridge
from gps_training_quality_preview import install_gps_training_quality_preview
from record_storage_runtime import install_record_storage_runtime
from camera_mount_calibration import CAMERA_MOUNT_HMI
from worker_ufld_bridge import CAMERA_WORKER_UFLD_HMI, install_worker_ufld_bridge

# server_v2_final imports unified_dashboard_data_tools before importing the
# camera calibration panel constant. Extend that panel here so the primary V1
# operator page gets the camera-mount UI without duplicating server routing.
if b'id="camera-mount-modal"' not in _camera_calibration_panel.CAMERA_CALIBRATION_PANEL:
    _camera_calibration_panel.CAMERA_CALIBRATION_PANEL += CAMERA_MOUNT_HMI

# server_v2_final imports this module before install_gps_ai() wraps the release
# handler. Install removable RECORD routing first so every later handler/runtime
# sees the USB-aware session resolver and storage status endpoint. Compute
# transfer/model policy, GPS training transfer/quality preview, and live UFLD
# are layered on top while motor/safety authority stays on the rover.
install_record_storage_runtime(_release.full.legacy)
install_compute_rover_api()
install_compute_candidate_policy()
install_compute_gps_training_bridge()
install_gps_training_quality_preview()
install_worker_ufld_bridge()

UNIFIED_DASHBOARD_DATA_TOOLS = r'''
<style id="record-replay-seek-style">
#record-replay-media-slot{margin-top:10px}
#record-replay-status{display:none!important}
#record-replay-seek{margin-top:12px;padding:10px 2px 2px}
#record-replay-seek .rr-seek-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:7px;color:var(--muted);font-size:9px}
#record-replay-seek .rr-seek-head b{color:var(--text);font:750 10px ui-monospace,monospace;white-space:nowrap}
#record-replay-slider{--rr-progress:0%;appearance:none;-webkit-appearance:none;width:100%;height:18px;margin:0;padding:0;background:transparent;cursor:pointer;outline:none}
#record-replay-slider::-webkit-slider-runnable-track{height:7px;border:1px solid rgba(255,255,255,.12);border-radius:999px;background:linear-gradient(90deg,var(--ok) 0,var(--ok) var(--rr-progress),rgba(255,255,255,.10) var(--rr-progress),rgba(255,255,255,.10) 100%)}
#record-replay-slider::-moz-range-track{height:7px;border:1px solid rgba(255,255,255,.12);border-radius:999px;background:rgba(255,255,255,.10)}
#record-replay-slider::-moz-range-progress{height:7px;border-radius:999px;background:var(--ok)}
#record-replay-slider::-webkit-slider-thumb{-webkit-appearance:none;width:15px;height:15px;margin-top:-5px;border:2px solid #d9fff6;border-radius:50%;background:var(--ok);box-shadow:0 0 0 3px rgba(65,228,210,.14)}
#record-replay-slider::-moz-range-thumb{width:13px;height:13px;border:2px solid #d9fff6;border-radius:50%;background:var(--ok);box-shadow:0 0 0 3px rgba(65,228,210,.14)}
#record-replay-slider:focus-visible::-webkit-slider-thumb{box-shadow:0 0 0 4px rgba(65,228,210,.28)}
#record-replay-context{min-height:16px;margin-top:4px;color:var(--muted);font-size:9px;line-height:1.45}
#record-replay-context.good{color:var(--ok)}#record-replay-context.bad{color:var(--bad)}
</style>
<script>
(function(){
 const dataGrid=document.querySelector('#view-data .grid');if(!dataGrid)return;
 const panel=document.createElement('div');panel.className='panel span12';panel.innerHTML=`
  <h2>저장된 주행 기록 관리</h2>
  <p class="sectionnote">기록 이름을 정리하거나, 원하는 시점의 주행 정보를 확인하고, 필요 없는 기록을 삭제할 수 있습니다.</p>
  <div class="row"><select id="record-manage-session"></select><input id="record-manage-label" placeholder="기록 설명"><button id="record-label-save">설명 저장</button><button id="record-delete" class="danger">기록 삭제</button></div>
  <div id="record-replay-media-slot"><div id="record-replay-status" hidden></div></div>
  <div id="record-replay-seek">
   <div class="rr-seek-head"><span>확인할 시점</span><b><span id="record-replay-offset">0.0초</span> / <span id="record-replay-duration">0.0초</span></b></div>
   <input id="record-replay-slider" type="range" min="0" max="0" value="0" step="0.1" aria-label="주행 기록 확인 시점">
   <div id="record-replay-context">기록을 선택한 뒤 바를 움직여 확인할 시점을 선택하세요.</div>
  </div>`;dataGrid.appendChild(panel);
 let sessions=[];
 function selected(){return sessions.find(x=>x.session===document.getElementById('record-manage-session').value)}
 function setSeekVisual(value,total){const current=Math.max(0,Number(value)||0);const duration=Math.max(0,Number(total)||0);const ratio=duration>0?Math.max(0,Math.min(1,current/duration)):0;const slider=document.getElementById('record-replay-slider');slider.style.setProperty('--rr-progress',`${(ratio*100).toFixed(3)}%`);document.getElementById('record-replay-offset').textContent=`${current.toFixed(1)}초`;document.getElementById('record-replay-duration').textContent=`${duration.toFixed(1)}초`}
 async function refreshSessions(){try{const response=await api('/api/recordings');sessions=response.sessions||[];const select=document.getElementById('record-manage-session');const before=select.value;fill(select,sessions,'session',x=>`${x.label||x.session}`,before);const item=selected();if(item)document.getElementById('record-manage-label').value=item.label||''}catch(error){console.error(error)}}
 const select=document.getElementById('record-manage-session');
 const slider=document.getElementById('record-replay-slider');
 const context=document.getElementById('record-replay-context');
 const hiddenStatus=document.getElementById('record-replay-status');
 select.onchange=()=>{const item=selected();document.getElementById('record-manage-label').value=item?.label||'';slider.max='0';slider.value='0';setSeekVisual(0,0);context.className='';context.textContent=item?'영상과 바를 준비하는 중입니다.':'기록을 선택해 주세요.'};
 slider.oninput=()=>{setSeekVisual(slider.value,slider.max);context.className='';context.textContent='손을 놓으면 해당 시점의 영상과 기록을 불러옵니다.'};
 slider.onchange=()=>{context.className='';context.textContent='선택한 시점을 불러오는 중입니다.'};
 document.getElementById('record-label-save').onclick=()=>action(async()=>{const item=selected();if(!item)throw new Error('세션을 선택하세요');await post('/api/recordings/label',{session:item.session,label:document.getElementById('record-manage-label').value});await refreshSessions()});
 document.getElementById('record-delete').onclick=()=>{const item=selected();if(!item)return alert('삭제할 기록을 선택해 주세요.');if(!confirm('선택한 주행 기록을 삭제할까요? 삭제 후에는 되돌릴 수 없습니다.'))return;action(async()=>{await post('/api/recordings/delete',{session:item.session});await refreshSessions();slider.max='0';slider.value='0';setSeekVisual(0,0);context.className='good';context.textContent='삭제 완료'})};
 const statusObserver=new MutationObserver(()=>{const small=hiddenStatus.querySelector('small');const strong=hiddenStatus.querySelector('strong');const text=(small?.textContent||strong?.textContent||'').trim();if(!text)return;context.textContent=text.replace('당시 주행 상태:','당시 주행 상태 ·');context.className=strong?.classList.contains('warn')?'bad':'good'});statusObserver.observe(hiddenStatus,{childList:true,subtree:true,characterData:true});
 refreshSessions();
 const sessionsTimer=setInterval(refreshSessions,3000);
 const seekVisualTimer=setInterval(()=>setSeekVisual(slider.value,slider.max),200);
 window.addEventListener('pagehide',()=>{clearInterval(sessionsTimer);clearInterval(seekVisualTimer);statusObserver.disconnect()},{once:true});
})();
</script>
'''.encode('utf-8') + USER_MANAGEMENT_HMI + RECORD_REPLAY_AUTO_HMI + GPS_MODEL_LIFECYCLE_HMI + SETTINGS_PAGE_SHELL + SETTINGS_HMI_POLISH + RECORD_REPLAY_OFFLINE_UFLD_HMI + AI_MODEL_SELECTION_HMI + COMPUTE_WORKER_HMI + COMPUTE_TRAINING_HMI + COMPUTE_GPS_TRAINING_HMI + RECORD_REPLAY_CANDIDATE_OVERLAY_HMI + CAMERA_MOUNT_HMI + CAMERA_WORKER_UFLD_HMI

__all__ = ["UNIFIED_DASHBOARD_DATA_TOOLS"]
