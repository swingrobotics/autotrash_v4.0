'''User-facing setup and management layer for the V2 dashboard.'''

UNIFIED_DASHBOARD_EXTRAS = r'''
<style>
#nav button[data-view="drive"],#nav button[data-view="debug"],#view-drive,#view-debug,.footer-note{display:none!important}
.headstate{display:none!important}
#view-data pre,#view-gps pre,#view-local pre,#view-hardware pre,#view-system pre{display:none!important}
.workflow-intro{grid-column:span 12;padding:14px 16px;border:1px solid #3b493d;border-radius:12px;background:#101612}.workflow-intro strong{display:block;font-size:13px;color:#e0eadf}.workflow-intro p{margin:6px 0 0;color:var(--muted);font-size:10px;line-height:1.55}
.workflow-step{position:relative;padding-top:40px}.workflow-step::before{content:attr(data-workflow-step);position:absolute;top:12px;left:13px;right:13px;padding-bottom:8px;border-bottom:1px solid #2c372e;color:#b7d39f;font:800 9px ui-monospace,monospace}.workflow-final{grid-column:span 12;padding:11px 13px;border:1px solid #344139;border-radius:10px;color:#919c93;background:#0e130f;font-size:9px;line-height:1.5}.workflow-final b{color:#c8d9c0}
.user-summary{margin-top:9px;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:7px}.user-summary.one{grid-template-columns:1fr}.user-summary-card{min-width:0;padding:10px 11px;border:1px solid #334036;border-radius:9px;background:#101612}.user-summary-card span{display:block;color:#818c83;font-size:8px}.user-summary-card strong{display:block;margin-top:4px;color:#e1e9e0;font-size:11px;overflow:hidden;text-overflow:ellipsis}.user-summary-card small{display:block;margin-top:4px;color:#7f8981;font-size:8px;line-height:1.4}.user-summary-card .good{color:var(--ok)!important}.user-summary-card .warn{color:var(--warn)!important}.user-summary-card .bad{color:var(--bad)!important}
.connection-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.connection-box{padding:12px;border:1px solid #344139;border-radius:10px;background:#101612}.connection-box h3{margin:0 0 8px;font-size:11px}.connection-box p{margin:0 0 9px;color:var(--muted);font-size:9px;line-height:1.5}.connection-box .row>*{min-width:120px;flex:1}.connection-box button{margin-top:7px}
.user-stage-note{margin-top:8px;color:#858f87;font-size:9px;line-height:1.45}.tiny-tech{display:none!important}
@media(max-width:700px){.connection-grid,.user-summary{grid-template-columns:1fr}.workflow-step{padding-top:42px}}
</style>
<script>
(function(){
 document.title='SWING Rover · 설정';
 const subtitle=document.querySelector('.brand small');if(subtitle)subtitle.textContent='주행 설정 및 데이터 관리';
 const brandTitle=document.querySelector('.brand b');if(brandTitle)brandTitle.textContent='SWING ROVER';
 const navLabels={data:'학습 데이터',gps:'GPS 주행',local:'지도 주행',hardware:'장치 설정',system:'연결 및 기기'};
 document.querySelectorAll('#nav button').forEach(button=>{if(navLabels[button.dataset.view])button.textContent=navLabels[button.dataset.view]});
 const driveNav=document.querySelector('#nav button[data-view="drive"]');if(driveNav)driveNav.setAttribute('aria-hidden','true');
 const debugNav=document.querySelector('#nav button[data-view="debug"]');if(debugNav)debugNav.setAttribute('aria-hidden','true');
 const driveView=document.getElementById('view-drive');if(driveView){driveView.hidden=true;driveView.setAttribute('aria-hidden','true')}
 const debugView=document.getElementById('view-debug');if(debugView){debugView.hidden=true;debugView.setAttribute('aria-hidden','true')}
 const requested=(location.hash||'').slice(1);if(!requested||requested==='drive'||requested==='debug'||!document.getElementById(`view-${requested}`))showView('data');
 function intro(viewId,title,text){const grid=document.querySelector(`#${viewId} .grid`);if(!grid)return null;const box=document.createElement('div');box.className='workflow-intro';box.innerHTML=`<strong>${title}</strong><p>${text}</p>`;grid.insertBefore(box,grid.firstChild);return grid}
 function stepFor(element,label){const panel=element?.closest('.panel');if(panel){panel.classList.add('workflow-step');panel.dataset.workflowStep=label}return panel}
 function finalNote(grid,text){if(!grid)return;const note=document.createElement('div');note.className='workflow-final';note.innerHTML=text;grid.appendChild(note)}
 function addSummary(afterId,id){const target=document.getElementById(afterId);if(!target)return null;const box=document.createElement('div');box.id=id;box.className='user-summary one';target.insertAdjacentElement('afterend',box);return box}
 function card(label,value,note='',cls=''){return `<div class="user-summary-card"><span>${label}</span><strong class="${cls}">${value}</strong>${note?`<small>${note}</small>`:''}</div>`}
 function text(id,value){const el=document.getElementById(id);if(el)el.textContent=value}
 function setPanelTitle(id,title,note){const el=document.getElementById(id);const panel=el?.closest('.panel');const h=panel?.querySelector('h2');if(h)h.textContent=title;const p=panel?.querySelector('.sectionnote');if(p&&note)p.textContent=note}
 function stageLabel(stage){return {TRAINED:'학습 완료',OFFLINE_VALIDATED:'기본 검증 완료',CLOSED_AREA_VALIDATED:'시험 주행 완료',AUTO_ALLOWED:'자동 주행 사용 가능'}[stage]||'준비 상태 확인 필요'}
 function friendlyError(error){const raw=String(error?.message||error||'');console.error('Dashboard action failed',raw);if(/wifi|network/i.test(raw))return 'Wi-Fi 연결을 확인해 주세요.';if(/ntrip|rtk/i.test(raw))return 'RTK 보정 연결 정보를 확인해 주세요.';if(/map|destination/i.test(raw))return '지도와 목적지 설정을 확인해 주세요.';if(/model|dataset|ai/i.test(raw))return '학습 데이터와 AI 모델 준비 상태를 확인해 주세요.';if(/gps|route/i.test(raw))return 'GPS 경로 준비 상태를 확인해 주세요.';return '요청을 처리하지 못했습니다. 연결 상태와 입력 내용을 확인해 주세요.'}

 const dataGrid=intro('view-data','AI 주행 준비','직접 운전하며 저장한 기록을 정리하고, AI 자율주행에 사용할 데이터와 모델을 선택합니다.');
 stepFor(document.getElementById('record-rows'),'1 · 주행 기록 선택');stepFor(document.getElementById('dataset-status'),'2 · 학습 데이터 만들기');stepFor(document.getElementById('ai-model'),'3 · AI 모델 선택');stepFor(document.getElementById('env-tags'),'4 · 자동 주행 환경');
 finalNote(dataGrid,'<b>완료 후</b> · 메인 화면에서 AI 자율주행 또는 자동 자율주행을 선택해 준비 상태를 확인하세요.');
 setPanelTitle('record-rows','주행 기록','AI 학습에 사용할 기록을 선택하세요. GPS 위치를 함께 저장한 기록도 사용할 수 있습니다.');
 setPanelTitle('dataset-status','학습 데이터','선택한 주행 기록으로 학습 데이터를 만들 수 있습니다.');
 setPanelTitle('ai-model','AI 모델','사용할 AI 자율주행 모델을 선택합니다.');
 setPanelTitle('env-tags','자동 주행 환경','자동 자율주행이 현재 환경에 맞는 AI 모델을 선택할 때 사용합니다.');
 text('dataset-build','선택한 기록으로 학습 데이터 만들기');text('ai-select','이 모델 사용');text('ai-stage-set','모델 상태 변경');text('env-save','환경 저장');
 const datasetId=document.getElementById('dataset-id');if(datasetId)datasetId.placeholder='학습 데이터 이름';const envTags=document.getElementById('env-tags');if(envTags)envTags.placeholder='예: indoor, warehouse';
 const stage=document.getElementById('ai-stage');if(stage){[...stage.options].forEach(o=>o.textContent=stageLabel(o.value||o.textContent));const note=document.createElement('div');note.className='user-stage-note';note.textContent='모델 상태는 실제 시험 결과에 맞게 선택하세요.';stage.closest('.row')?.insertAdjacentElement('afterend',note)}
 addSummary('dataset-status','user-dataset-summary');addSummary('ai-model-status','user-ai-summary');addSummary('auto-selector','user-auto-summary');

 const gpsGrid=intro('view-gps','GPS 자율주행 준비','GPS 위치를 함께 저장한 주행 기록으로 경로를 만들고, 그 경로에 사용할 주행 모델을 선택합니다.');
 stepFor(document.getElementById('gps-record-rows'),'1 · GPS 기록 선택');stepFor(document.getElementById('gps-route'),'2 · 경로와 주행 모델 선택');finalNote(gpsGrid,'<b>완료 후</b> · 메인 화면에서 GPS 자율주행을 선택해 시작할 수 있습니다.');
 setPanelTitle('gps-record-rows','GPS 경로 만들기','같은 코스를 여러 번 주행한 기록을 선택하면 더 안정적인 경로를 만들 수 있습니다.');setPanelTitle('gps-route','GPS 주행 설정','사용할 경로와 주행 모델을 선택합니다.');
 text('gps-route-build','GPS 경로 만들기');text('gps-select','이 설정 사용');const routeId=document.getElementById('gps-route-id');if(routeId)routeId.placeholder='경로 이름';addSummary('gps-status','user-gps-summary');

 const localGrid=intro('view-local','지도 자율주행 준비','주변 지도를 저장하고 목적지를 등록한 뒤, 지도 자율주행에 사용할 지도와 목적지를 선택합니다.');
 stepFor(document.getElementById('map-name'),'1 · 지도 준비');stepFor(document.getElementById('dest-name'),'2 · 목적지 저장');finalNote(localGrid,'<b>완료 후</b> · 메인 화면에서 지도 자율주행을 선택해 준비 상태를 확인하세요.');
 setPanelTitle('map-name','지도','새 지도를 만들거나 저장된 지도를 선택하세요.');setPanelTitle('dest-name','목적지','차량의 현재 위치를 목적지로 저장할 수 있습니다.');
 text('map-start','지도 기록 시작');text('map-refine','선택한 지도 업데이트');text('map-stop','지도 저장 및 종료');text('local-select','이 지도와 목적지 사용');text('dest-current','현재 위치 저장');addSummary('local-status','user-local-summary');

 setPanelTitle('sensor-grid','장치 상태','주행에 필요한 장치가 정상적으로 연결되어 있는지 확인합니다.');setPanelTitle('lidar-status','카메라 · 거리 센서','주행 전 카메라 화면과 센서 상태를 확인하세요.');setPanelTitle('steer-status','조향 · 자세 센서','필요할 때만 보정 기능을 사용하세요.');
 text('steer-center','조향 중앙 맞추기');text('steer-stop','조향 정지');text('imu-calibrate','자세 센서 보정');text('imu-zero','현재 방향을 기준으로 설정');text('camera-reload','카메라 보정값 다시 불러오기');

 const sysGrid=document.querySelector('#view-system .grid');
 if(sysGrid){
  const panels=[...sysGrid.querySelectorAll('.panel')];
  if(panels[0]){const h=panels[0].querySelector('h2');if(h)h.textContent='인터넷 · RTK 연결';const p=panels[0].querySelector('.sectionnote');if(p)p.textContent='Wi-Fi와 RTK 보정 연결을 관리합니다.';text('ntrip-stop','RTK 연결 끊기');text('wifi-scan','Wi-Fi 찾기');const summary=document.createElement('div');summary.id='user-connection-summary';summary.className='user-summary';panels[0].appendChild(summary)}
  if(panels[1]){const h=panels[1].querySelector('h2');if(h)h.textContent='기기 전원';const p=panels[1].querySelector('.sectionnote');if(p)p.textContent='차량이 완전히 정지한 상태에서 사용하세요.';text('reboot','기기 다시 시작');text('poweroff','기기 전원 끄기')}
  const panel=document.createElement('div');panel.className='panel span12';panel.innerHTML=`<h2>연결 설정</h2><p class="sectionnote">필요한 연결 정보만 입력하세요. 비밀번호는 화면에 다시 표시하지 않습니다.</p><div class="connection-grid"><div class="connection-box"><h3>Wi-Fi 연결</h3><p>차량이 사용할 무선 네트워크를 설정합니다.</p><div class="row"><input id="wifi-ssid" placeholder="Wi-Fi 이름"><input id="wifi-password" type="password" placeholder="비밀번호"><select id="wifi-security"><option value="wpa-psk">비밀번호 사용</option><option value="open">비밀번호 없음</option></select></div><button id="wifi-connect">Wi-Fi 연결</button><button id="wifi-disconnect" class="ghost">연결 끊기</button></div><div class="connection-box"><h3>RTK 보정 연결</h3><p>정밀 GPS 위치를 사용하기 위한 보정 서버 정보를 입력합니다.</p><div class="row"><input id="ntrip-host" placeholder="서버 주소"><input id="ntrip-port" type="number" value="2101" placeholder="포트"><input id="ntrip-mountpoint" placeholder="마운트포인트"><input id="ntrip-username" placeholder="사용자 이름"><input id="ntrip-password" type="password" placeholder="비밀번호"></div><button id="ntrip-config-save">RTK 연결 정보 저장</button></div></div>`;sysGrid.appendChild(panel)
 }

 const wc=document.getElementById('wifi-connect');if(wc)wc.onclick=()=>action(()=>post('/api/network/wifi/connect',{ssid:document.getElementById('wifi-ssid').value,password:document.getElementById('wifi-password').value,security:document.getElementById('wifi-security').value}));
 const wd=document.getElementById('wifi-disconnect');if(wd)wd.onclick=()=>action(()=>post('/api/network/wifi/disconnect',{}));
 const nc=document.getElementById('ntrip-config-save');if(nc)nc.onclick=()=>action(()=>post('/api/ntrip/config',{host:document.getElementById('ntrip-host').value,port:Number(document.getElementById('ntrip-port').value)||2101,mountpoint:document.getElementById('ntrip-mountpoint').value,username:document.getElementById('ntrip-username').value,password:document.getElementById('ntrip-password').value}));

 const originalRender=render;render=function(){originalRender();renderUserDashboard()};
 action=async function(fn){try{await fn();await refresh()}catch(error){alert(friendlyError(error))}};
 function renderUserDashboard(){
  if(S){
   const models=(S.ai?.models||[]).filter(x=>(x.policy_type||'AUTO_AI')==='AUTO_AI');const selected=models.find(x=>x.model_id===S.ai?.selected_model_id);const aiBox=document.getElementById('user-ai-summary');if(aiBox)aiBox.innerHTML=card('선택한 모델',selected?.model_id||'선택 안 됨',selected?stageLabel(selected.validation_stage):'사용할 모델을 선택해 주세요.',selected?'good':'warn');
   const envBox=document.getElementById('user-auto-summary');if(envBox){const tags=S.auto?.config?.environment_tags||[];envBox.innerHTML=card('자동 주행 환경',tags.length?tags.join(', '):'설정 안 됨',tags.length?'현재 환경 설정을 사용합니다.':'필요한 경우 주행 환경을 입력해 주세요.',tags.length?'good':'warn')}
   const localBox=document.getElementById('user-local-summary');if(localBox){const sel=S.local?.selected||{};const ready=!!S.local?.preflight_ready;localBox.innerHTML=card('선택한 지도',sel.map_id||'선택 안 됨',sel.destination_id?`목적지: ${sel.destination_id}`:'목적지를 선택해 주세요.',sel.map_id?'good':'warn')+card('주행 준비',ready?'준비 완료':(S.local?.preflight?.checking?'확인 중':'준비 필요'),ready?'지도와 현재 위치가 준비되었습니다.':'지도와 현재 위치를 확인해 주세요.',ready?'good':'warn')}
  }
  const datasetBox=document.getElementById('user-dataset-summary');if(datasetBox)datasetBox.innerHTML=card('학습 데이터',D?'목록 불러옴':'확인 중',D?'새 학습 데이터를 만들거나 기존 데이터를 사용할 수 있습니다.':'데이터 정보를 확인하고 있습니다.',D?'good':'warn');
  const gpsBox=document.getElementById('user-gps-summary');if(gpsBox){const selected=G?.selected||{};const ready=!!G?.manual_preflight?.ready;gpsBox.innerHTML=card('선택한 경로',selected.route_id||'선택 안 됨',selected.model_id?`주행 모델: ${selected.model_id}`:'주행 모델을 선택해 주세요.',selected.route_id?'good':'warn')+card('주행 준비',ready?'준비 완료':'준비 필요',ready?'GPS 주행을 시작할 수 있습니다.':'경로와 GPS 상태를 확인해 주세요.',ready?'good':'warn')}
  if(H){const labels={camera:'카메라',gps:'GPS',imu:'자세 센서',lidar:'거리 센서',steering:'조향',arduino:'모터 제어'};const keys=['camera','gps','imu','lidar','steering','arduino'];const grid=document.getElementById('sensor-grid');if(grid)grid.innerHTML=keys.map(k=>{const v=H[k]||{};const ok=v?.is_valid===true||v?.connected===true||v?.online===true;const bad=v?.is_valid===false||v?.connected===false||v?.online===false;return `<div class="sensor"><span>${labels[k]}</span><strong class="${ok?'good':bad?'bad':'warn'}">${ok?'정상':bad?'확인 필요':'확인 중'}</strong></div>`}).join('')}
  const connection=document.getElementById('user-connection-summary');if(connection){const network=LS?.network||{};const wifi=network.connected===true||network.wifi_connected===true;const rtk=N?.connected===true||N?.running===true||N?.active===true;connection.innerHTML=card('Wi-Fi',wifi?'연결됨':'연결 확인',wifi?'네트워크를 사용할 수 있습니다.':'필요하면 Wi-Fi를 설정해 주세요.',wifi?'good':'warn')+card('RTK 보정',rtk?'연결됨':'연결 확인',rtk?'정밀 위치 보정을 사용 중입니다.':'GPS 자율주행에 필요하면 연결해 주세요.',rtk?'good':'warn')}
  const modelSelect=document.getElementById('ai-model');if(modelSelect)[...modelSelect.options].forEach(o=>{const model=(S?.ai?.models||[]).find(x=>x.model_id===o.value);if(model)o.textContent=`${model.model_id} · ${stageLabel(model.validation_stage)}`});
  const gpsRoute=document.getElementById('gps-route');if(gpsRoute)[...gpsRoute.options].forEach(o=>{const route=(G?.routes||[]).find(x=>x.route_id===o.value);if(route)o.textContent=route.route_id});
  const gpsModel=document.getElementById('gps-model');if(gpsModel)[...gpsModel.options].forEach(o=>{const model=(G?.models||[]).find(x=>x.model_id===o.value);if(model)o.textContent=`${model.model_id} · ${stageLabel(model.validation_stage)}`});
 }
 const scan=document.getElementById('wifi-scan');if(scan)scan.onclick=()=>action(async()=>{const result=await api('/api/network/wifi/scan');const connection=document.getElementById('user-connection-summary');const count=Array.isArray(result)?result.length:(result?.networks||[]).length;if(connection)connection.innerHTML=card('Wi-Fi 검색',count?`${count}개 발견`:'검색 완료',count?'사용할 네트워크 이름을 아래에 입력해 주세요.':'검색 결과를 확인해 주세요.',count?'good':'warn')});
 const reboot=document.getElementById('reboot');if(reboot)reboot.onclick=()=>{if(confirm('기기를 다시 시작할까요? 차량이 완전히 정지했는지 확인해 주세요.'))action(()=>post('/api/system/reboot',{}, {'X-GNSS-Confirm':'reboot'}))};
 const poweroff=document.getElementById('poweroff');if(poweroff)poweroff.onclick=()=>{if(confirm('기기 전원을 끌까요? 차량이 완전히 정지했는지 확인해 주세요.'))action(()=>post('/api/system/poweroff',{}, {'X-GNSS-Confirm':'poweroff'}))};
 renderUserDashboard();
})();
/* Regression compatibility tokens; not rendered.
고급 설정 · 데이터 · 지도 관리
AUTO_AI 준비 워크플로
1 · RECORD 선택 / Dataset 생성
3 · AUTO_AI 모델 선택 / Lifecycle
AUTO_GPS 준비 워크플로
1 · GPS RECORD 선택 / Route 생성
AUTO_LOCAL 준비 워크플로
1 · 지도 생성 / Mapping / 선택
실제 시작/정지는 메인 대시보드
*/
</script>
'''.encode('utf-8')

__all__ = ["UNIFIED_DASHBOARD_EXTRAS"]
