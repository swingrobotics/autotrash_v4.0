'''Dashboard workflow for synchronized model-vs-human RECORD preview.'''

from compute_record_preview_bridge import install_compute_record_preview_bridge


install_compute_record_preview_bridge()


RECORD_MODEL_PREVIEW_HMI = r'''
<style id="record-model-preview-hmi-style">
#record-model-preview{margin-top:14px;padding-top:13px;border-top:1px solid rgba(255,255,255,.08)}
#record-model-preview .rmp-head{display:flex;align-items:flex-end;justify-content:space-between;gap:10px;flex-wrap:wrap}
#record-model-preview .rmp-head h3{margin:0;font-size:12px}
#record-model-preview .rmp-head .pill{font-size:8px}
#record-model-preview .rmp-grid{display:grid;grid-template-columns:minmax(220px,1fr) 150px auto auto;gap:7px;align-items:end;margin-top:9px}
#record-model-preview .rmp-field label{display:block;margin-bottom:5px;color:var(--muted);font-size:8px}
#record-model-preview select{width:100%}
#record-model-preview .rmp-progress{height:7px;margin-top:9px;border:1px solid rgba(255,255,255,.10);border-radius:999px;overflow:hidden;background:#0d1010}
#record-model-preview .rmp-progress>i{display:block;width:0;height:100%;background:var(--ok);transition:width .2s ease}
#record-model-preview-status{margin-top:8px;padding:8px 10px;border-left:3px solid rgba(255,255,255,.15);background:#111414;color:var(--muted);font-size:9px;line-height:1.5;white-space:pre-wrap}
#record-model-preview-status.good{border-left-color:var(--ok);color:#b9d8a6}
#record-model-preview-status.warn{border-left-color:var(--warn);color:#d7c08b}
#record-model-preview-status.bad{border-left-color:var(--bad);color:#e2a49f}
#record-model-preview-result{display:none;margin-top:10px}
#record-model-preview-result.show{display:block}
#record-model-preview-video{display:block;width:100%;max-height:520px;background:#000;border:1px solid #303932;border-radius:8px}
#record-model-preview .rmp-metrics{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:1px;margin-top:8px;background:rgba(255,255,255,.07)}
#record-model-preview .rmp-metric{padding:8px;background:#151818;min-width:0}
#record-model-preview .rmp-metric span{display:block;color:var(--muted);font-size:8px}
#record-model-preview .rmp-metric b{display:block;margin-top:4px;font:650 10px ui-monospace,monospace;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
#record-model-preview .rmp-result-actions{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin-top:8px}
#record-model-preview .rmp-note{color:var(--muted);font-size:8px;line-height:1.45}
@media(max-width:900px){#record-model-preview .rmp-grid{grid-template-columns:1fr 130px}#record-model-preview .rmp-metrics{grid-template-columns:repeat(3,minmax(0,1fr))}}
@media(max-width:620px){#record-model-preview .rmp-grid{grid-template-columns:1fr}#record-model-preview .rmp-metrics{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style>
<script>
(function(){
 const WORKER='http://127.0.0.1:8765';
 const sessionSelect=document.getElementById('record-manage-session');
 const seek=document.getElementById('record-replay-seek');
 if(!sessionSelect||!seek)return;

 const root=document.createElement('div');
 root.id='record-model-preview';
 root.innerHTML=`
  <div class="rmp-head">
   <div><h3>AI 계산 프리뷰</h3><div class="rmp-note">저장된 카메라·LiDAR·IMU를 당시 시점대로 다시 맞추고, GPS 모델은 GNSS와 기준 Route까지 함께 재생합니다. 모터 제어 권한은 없습니다.</div></div>
   <span id="record-model-preview-worker" class="pill">Worker 확인 중</span>
  </div>
  <div class="rmp-grid">
   <div class="rmp-field"><label>계산할 설치 모델</label><select id="record-model-preview-model"><option value="">모델 확인 중</option></select></div>
   <div class="rmp-field"><label id="record-model-preview-step-label">추론 간격</label><select id="record-model-preview-step"><option value="1">모든 프레임</option><option value="2">2 프레임마다</option><option value="5">5 프레임마다</option></select></div>
   <button id="record-model-preview-start" class="primary" type="button" disabled>AI 프리뷰 계산</button>
   <button id="record-model-preview-cancel" type="button" disabled>취소</button>
  </div>
  <div class="rmp-progress"><i id="record-model-preview-bar"></i></div>
  <div id="record-model-preview-status">RECORD와 모델을 선택하세요.</div>
  <div id="record-model-preview-result">
   <video id="record-model-preview-video" controls preload="metadata" playsinline></video>
   <div class="rmp-metrics">
    <div class="rmp-metric"><span>정책</span><b id="rmp-policy">--</b></div>
    <div class="rmp-metric"><span>조향 MAE</span><b id="rmp-steer-mae">--</b></div>
    <div class="rmp-metric"><span>최대 조향 오차</span><b id="rmp-steer-max">--</b></div>
    <div class="rmp-metric"><span>스로틀 MAE</span><b id="rmp-throttle-mae">--</b></div>
    <div class="rmp-metric"><span>LiDAR / IMU sync</span><b id="rmp-sensor-sync">--</b></div>
    <div class="rmp-metric"><span>GNSS sync</span><b id="rmp-gnss-sync">--</b></div>
   </div>
   <div class="rmp-result-actions"><button id="record-model-preview-csv" type="button">프레임 비교 CSV</button><span id="record-model-preview-summary" class="rmp-note"></span></div>
  </div>`;
 seek.insertAdjacentElement('afterend',root);

 const q=id=>document.getElementById(id);
 const modelSelect=q('record-model-preview-model');
 const stepSelect=q('record-model-preview-step');
 const stepLabel=q('record-model-preview-step-label');
 const startButton=q('record-model-preview-start');
 const cancelButton=q('record-model-preview-cancel');
 const workerBadge=q('record-model-preview-worker');
 const status=q('record-model-preview-status');
 const bar=q('record-model-preview-bar');
 const result=q('record-model-preview-result');
 const previewVideo=q('record-model-preview-video');
 const csvButton=q('record-model-preview-csv');
 let models=[];
 let workerReady=false;
 let currentJob='';
 let artifactToken='';
 let artifactWorkerUrls=[];
 let polling=false;
 let videoUrl='';
 let csvUrl='';
 let csvName='record-model-preview.csv';

 const number=(value,fallback=0)=>{const n=Number(value);return Number.isFinite(n)?n:fallback};
 const pct=value=>`${(Math.max(0,Math.min(1,number(value,0)))*100).toFixed(0)}%`;
 const errText=error=>String(error?.message||error||'알 수 없는 오류');
 async function fetchJson(url,options={}){
   const response=await fetch(url,{cache:'no-store',...options});
   let data={};try{data=await response.json()}catch(_error){}
   if(!response.ok)throw new Error(data.error||`HTTP ${response.status}`);
   return data;
 }
 function roverPost(path,body){return fetchJson(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})}
 function workerPost(path,body){return fetchJson(WORKER+path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})}

 function releaseUrls(){
   if(videoUrl){URL.revokeObjectURL(videoUrl);videoUrl=''}
   if(csvUrl){URL.revokeObjectURL(csvUrl);csvUrl=''}
   previewVideo.removeAttribute('src');previewVideo.load();
 }
 function clearResult(){releaseUrls();result.classList.remove('show');bar.style.width='0%'}
 function selectedModel(){return models.find(model=>model.model_id===modelSelect.value)||null}
 function modelLabel(model){
   const policy=String(model.policy_type||'AUTO_AI');
   const route=policy==='AUTO_GPS'&&model.route_id?` · Route ${model.route_id}`:'';
   const temporal=model.temporal_gps?` · temporal ${Number(model.temporal_history_steps||5)}F`:'';
   const stage=model.validation_stage?` · ${model.validation_stage}`:'';
   return `${policy} · ${model.model_id}${route}${temporal}${stage}`;
 }
 function syncReady(){
   const model=selectedModel();
   const temporal=!!model?.temporal_gps;
   if(temporal){
     stepSelect.value='1';
     stepSelect.disabled=true;
     stepLabel.textContent=`추론 간격 · temporal ${Number(model.temporal_history_steps||5)}F`;
   }else{
     stepSelect.disabled=false;
     stepLabel.textContent='추론 간격';
   }
   const ready=workerReady&&!!sessionSelect.value&&!!model&&model.preview_available!==false&&!currentJob;
   startButton.disabled=!ready;
 }
 function setStatus(text,kind=''){
   status.className=kind;
   status.textContent=text;
 }
 async function refreshModels(){
   try{
     const data=await fetchJson('/api/v2/compute/preview-models');
     const before=modelSelect.value;
     models=Array.isArray(data.models)?data.models:[];
     if(!models.length){
       modelSelect.innerHTML='<option value="">프리뷰 가능한 설치 모델 없음</option>';
       modelSelect.disabled=true;
     }else{
       modelSelect.disabled=false;
       modelSelect.innerHTML=models.map(model=>`<option value="${String(model.model_id).replace(/"/g,'&quot;')}" ${model.preview_available===false?'disabled':''}>${modelLabel(model)}${model.preview_available===false?' · manifest 없음':''}</option>`).join('');
       if(models.some(model=>model.model_id===before&&model.preview_available!==false))modelSelect.value=before;
       else{
         const first=models.find(model=>model.preview_available!==false);
         if(first)modelSelect.value=first.model_id;
       }
     }
   }catch(error){
     models=[];modelSelect.disabled=true;modelSelect.innerHTML='<option value="">모델 목록 확인 실패</option>';
   }
   syncReady();
 }
 async function checkWorker(){
   try{
     const data=await fetchJson(WORKER+'/api/v1/status');
     workerReady=!!data.capabilities?.record_model_preview&&!!data.capabilities?.record_model_preview_artifacts&&!!data.capabilities?.record_model_preview_h264;
     workerBadge.textContent=workerReady?`${data.hostname||'PC'} · H.264 프리뷰 준비됨`:'Worker 업데이트 필요';
     workerBadge.className=`pill ${workerReady?'good':'warn'}`;
   }catch(error){
     workerReady=false;workerBadge.textContent='Compute Worker 미연결';workerBadge.className='pill warn';
   }
   syncReady();
 }
 function phaseName(value){return ({QUEUED:'대기',SYNCING:'RECORD 동기화',SYNCING_MODEL:'모델 동기화',SYNCING_ROUTE:'GPS Route 동기화',MODEL_PREVIEW:'AI 재계산',ENCODING_PREVIEW:'H.264 영상 변환',SUCCEEDED:'완료',FAILED:'실패',CANCELED:'취소'}[value]||value||'처리 중')}

 async function fetchArtifact(name,type){
   const response=await fetch('/api/v2/compute/preview-artifact',{
     method:'POST',cache:'no-store',headers:{'Content-Type':'application/json'},
     body:JSON.stringify({worker_urls:artifactWorkerUrls,job_id:currentJob,artifact_token:artifactToken,artifact:name})
   });
   if(!response.ok){let message=`HTTP ${response.status}`;try{const data=await response.json();message=data.error||message}catch(_error){}throw new Error(message)}
   const buffer=await response.arrayBuffer();
   return URL.createObjectURL(new Blob([buffer],{type}));
 }
 async function showResult(job){
   const data=job.result||{};
   setStatus('계산은 완료되었습니다. 결과 영상을 PC Worker에서 불러오는 중…','warn');
   const [newVideo,newCsv]=await Promise.all([
     fetchArtifact('preview_video','video/mp4'),
     fetchArtifact('preview_csv','text/csv;charset=utf-8'),
   ]);
   releaseUrls();videoUrl=newVideo;csvUrl=newCsv;
   csvName=`${String(data.session||sessionSelect.value||'record')}.${String(data.model_id||modelSelect.value||'model')}.preview.csv`;
   previewVideo.src=videoUrl;previewVideo.load();
   q('rmp-policy').textContent=`${data.policy_type||'-'}${data.route_id?` / ${data.route_id}`:''}${data.temporal_gps?' / temporal':''}`;
   const steer=Number(data.mean_abs_steering_error_degrees),maxSteer=Number(data.maximum_abs_steering_error_degrees),throttle=Number(data.mean_abs_throttle_error);
   q('rmp-steer-mae').textContent=Number.isFinite(steer)?`${steer.toFixed(2)}°`:'--';
   q('rmp-steer-max').textContent=Number.isFinite(maxSteer)?`${maxSteer.toFixed(2)}°`:'--';
   q('rmp-throttle-mae').textContent=Number.isFinite(throttle)?throttle.toFixed(3):'--';
   q('rmp-sensor-sync').textContent=`${pct(data.lidar_sync_ratio)} / ${pct(data.imu_sync_ratio)}`;
   q('rmp-gnss-sync').textContent=data.gnss_sync_ratio===null||data.gnss_sync_ratio===undefined?'N/A':pct(data.gnss_sync_ratio);
   const cadence=data.temporal_gps?' · temporal은 모든 프레임 추론':'';
   q('record-model-preview-summary').textContent=`${Number(data.inferred_frames||0).toLocaleString()}회 추론 · ${Number(data.source_frames||0).toLocaleString()} frames · H.264${cadence} · 제어 권한 NONE`;
   result.classList.add('show');
   setStatus('AI 계산 프리뷰 완료 · 주황 MODEL 경로와 녹색 점선 HUMAN 경로를 비교하세요.','good');
 }
 async function startPreview(){
   const session=sessionSelect.value;
   const model=selectedModel();
   if(!session)return alert('먼저 RECORD를 선택하세요.');
   if(!model)return alert('계산할 모델을 선택하세요.');
   if(!workerReady)return alert('Compute Worker를 최신 버전으로 설치하고 실행해 주세요.');
   clearResult();currentJob='';artifactToken='';artifactWorkerUrls=[];syncReady();
   setStatus(model.temporal_gps?'temporal GPS 모델 · 0.5초 이력을 유지하기 위해 모든 프레임으로 계산합니다.':'차량에서 선택 RECORD와 모델의 읽기 권한을 준비하는 중…','warn');
   try{
     const grant=await roverPost('/api/v2/compute/preview-transfer',{session,model_id:model.model_id});
     const actual=grant.model||model;
     const job=await workerPost('/api/v1/jobs',{
       kind:'preview_record_model',
       rover_url:location.origin,
       transfer_token:grant.token,
       session,
       model_id:actual.model_id,
       sample_every:actual.temporal_gps?1:(Number(stepSelect.value)||1),
     });
     currentJob=job.job_id;artifactToken=job.artifact_token||'';artifactWorkerUrls=Array.isArray(job.worker_urls)?job.worker_urls:[];polling=true;
     cancelButton.disabled=false;startButton.disabled=true;poll();
   }catch(error){
     currentJob='';artifactToken='';artifactWorkerUrls=[];setStatus('프리뷰 시작 실패 · '+errText(error),'bad');syncReady();
   }
 }
 async function poll(){
   if(!currentJob||!polling)return;
   try{
     const job=await fetchJson(WORKER+'/api/v1/jobs/'+encodeURIComponent(currentJob));
     bar.style.width=`${Math.round(number(job.progress,0)*100)}%`;
     if(job.state==='FAILED'){
       polling=false;cancelButton.disabled=true;setStatus(`프리뷰 실패 · ${job.error||job.message||'알 수 없는 오류'}`,'bad');currentJob='';artifactToken='';artifactWorkerUrls=[];syncReady();return;
     }
     if(job.state==='CANCELED'){
       polling=false;cancelButton.disabled=true;setStatus('AI 계산 프리뷰가 취소되었습니다.','warn');currentJob='';artifactToken='';artifactWorkerUrls=[];syncReady();return;
     }
     if(job.state==='SUCCEEDED'){
       polling=false;cancelButton.disabled=true;bar.style.width='100%';
       try{await showResult(job)}catch(error){setStatus('결과 파일 로딩 실패 · '+errText(error),'bad')}
       currentJob='';artifactToken='';artifactWorkerUrls=[];syncReady();return;
     }
     setStatus(`${phaseName(job.phase)} · ${Math.round(number(job.progress,0)*100)}%\n${job.message||''}`,'warn');
     setTimeout(poll,650);
   }catch(error){setStatus('Worker 상태 확인 실패 · '+errText(error),'bad');setTimeout(poll,1200)}
 }
 async function cancelPreview(){
   if(!currentJob)return;
   try{await fetchJson(WORKER+'/api/v1/jobs/'+encodeURIComponent(currentJob),{method:'DELETE'})}catch(error){setStatus('취소 요청 실패 · '+errText(error),'bad')}
 }

 startButton.addEventListener('click',startPreview);
 cancelButton.addEventListener('click',cancelPreview);
 csvButton.addEventListener('click',()=>{
   if(!csvUrl)return;
   const anchor=document.createElement('a');anchor.href=csvUrl;anchor.download=csvName;document.body.appendChild(anchor);anchor.click();anchor.remove();
 });
 modelSelect.addEventListener('change',()=>{if(!currentJob){clearResult();const model=selectedModel();setStatus(model?.temporal_gps?'temporal GPS 모델: 최근 5프레임의 IMU yaw-rate + 이전 조향 이력을 사용합니다.':'선택한 모델로 RECORD를 다시 계산할 수 있습니다.');syncReady()}});
 sessionSelect.addEventListener('change',()=>{if(!currentJob){clearResult();setStatus(sessionSelect.value?'이 RECORD에 적용할 모델을 선택하세요.':'RECORD를 선택하세요.');syncReady()}});
 stepSelect.addEventListener('change',syncReady);
 window.addEventListener('pagehide',()=>{polling=false;releaseUrls()},{once:true});

 refreshModels();checkWorker();
 const workerTimer=setInterval(checkWorker,5000);
 const modelTimer=setInterval(refreshModels,10000);
 window.addEventListener('pagehide',()=>{clearInterval(workerTimer);clearInterval(modelTimer)},{once:true});
})();
</script>
'''.encode('utf-8')


__all__ = ["RECORD_MODEL_PREVIEW_HMI"]
