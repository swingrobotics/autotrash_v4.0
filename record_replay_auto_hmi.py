'''RECORD video replay with synchronized UFLD lane overlay.'''

from record_replay_media import install_record_replay_media_endpoints


install_record_replay_media_endpoints()


RECORD_REPLAY_AUTO_HMI = r'''
<style>
#record-replay-media{margin-top:10px}
.record-replay-stage{
  position:relative;width:100%;aspect-ratio:16/9;overflow:hidden;
  border:1px solid #303932;border-radius:9px;background:#050705
}
#record-replay-video,#record-replay-frame{
  display:block;width:100%;height:100%;object-fit:contain;background:#000
}
#record-replay-frame{display:none}
#record-replay-lane-canvas{
  position:absolute;inset:0;width:100%;height:100%;pointer-events:none
}
#record-replay-lane-meta{
  position:absolute;right:10px;top:10px;padding:6px 9px;border:1px solid #48634a;
  border-radius:6px;background:#050805d9;color:#b9d8a6;font:700 10px ui-monospace,monospace
}
.record-replay-toolbar{display:flex;gap:7px;align-items:center;margin-top:7px;flex-wrap:wrap}
.record-replay-toolbar button{min-height:30px}
.record-replay-metrics{
  display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:6px;margin-top:7px
}
.record-replay-metric{padding:7px 8px;border:1px solid #303932;border-radius:7px;background:#111511}
.record-replay-metric span{display:block;color:#919d93;font-size:9px}
.record-replay-metric strong{display:block;margin-top:3px;font-size:11px;overflow:hidden;text-overflow:ellipsis}
@media(max-width:760px){.record-replay-metrics{grid-template-columns:repeat(2,minmax(0,1fr))}}
</style>
<script>
(function(){
 const select=document.getElementById('record-manage-session');
 const slider=document.getElementById('record-replay-slider');
 const status=document.getElementById('record-replay-status');
 const offsetLabel=document.getElementById('record-replay-offset');
 if(!select||!slider||!status)return;

 const host=status.parentElement;
 const media=document.createElement('div');
 media.id='record-replay-media';
 media.innerHTML=`
   <div class="record-replay-stage" id="record-replay-stage">
     <video id="record-replay-video" controls preload="metadata" playsinline></video>
     <img id="record-replay-frame" alt="기록 영상 프레임">
     <canvas id="record-replay-lane-canvas"></canvas>
     <div id="record-replay-lane-meta">RECORD · 영상 선택 대기</div>
   </div>
   <div class="record-replay-toolbar">
     <button id="record-replay-fallback-play" type="button" style="display:none">프레임 재생</button>
     <span class="sectionnote" id="record-replay-media-note">저장된 카메라 영상과 같은 시점의 UFLD 차선을 함께 표시합니다.</span>
   </div>
   <div class="record-replay-metrics">
     <div class="record-replay-metric"><span>차선 백엔드</span><strong id="record-replay-backend">--</strong></div>
     <div class="record-replay-metric"><span>신뢰도</span><strong id="record-replay-confidence">--</strong></div>
     <div class="record-replay-metric"><span>UFLD 추론</span><strong id="record-replay-inference">--</strong></div>
     <div class="record-replay-metric"><span>제어 권한</span><strong id="record-replay-authority">--</strong></div>
   </div>`;
 host.insertBefore(media,status);

 const stage=document.getElementById('record-replay-stage');
 const video=document.getElementById('record-replay-video');
 const frame=document.getElementById('record-replay-frame');
 const canvas=document.getElementById('record-replay-lane-canvas');
 const ctx=canvas.getContext('2d');
 const meta=document.getElementById('record-replay-lane-meta');
 const fallbackPlay=document.getElementById('record-replay-fallback-play');
 const note=document.getElementById('record-replay-media-note');
 const backendEl=document.getElementById('record-replay-backend');
 const confidenceEl=document.getElementById('record-replay-confidence');
 const inferenceEl=document.getElementById('record-replay-inference');
 const authorityEl=document.getElementById('record-replay-authority');

 let currentSession='';
 let fallback=false;
 let fallbackTimer=null;
 let replayTimer=null;
 let replayBusy=false;
 let pendingReplayOffset=null;
 let lastReplayAt=0;
 let lastPerception=null;

 function number(value,fallbackValue=0){const result=Number(value);return Number.isFinite(result)?result:fallbackValue}
 function truthy(value){return value===true||String(value).toLowerCase()==='true'||String(value)==='1'}
 function jsonValue(value,fallbackValue){
   if(value===null||value===undefined||value==='')return fallbackValue;
   if(typeof value==='object')return value;
   try{return JSON.parse(value)}catch(_error){return fallbackValue}
 }
 function sessionUrl(path,extra=''){
   return `${path}?session=${encodeURIComponent(currentSession)}${extra}`;
 }
 function setStageRatio(width,height){
   width=number(width,0);height=number(height,0);
   if(width>0&&height>0)stage.style.aspectRatio=`${width}/${height}`;
 }
 function resizeCanvas(){
   const width=Math.max(1,Math.round(stage.clientWidth));
   const height=Math.max(1,Math.round(stage.clientHeight));
   if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}
 }
 function linePoints(row,key){
   const document=jsonValue(row?.[key],{});
   const values=Array.isArray(document?.points)?document.points:[];
   return values.filter(point=>Array.isArray(point)&&point.length>=2&&Number.isFinite(Number(point[0]))&&Number.isFinite(Number(point[1])));
 }
 function drawLine(points,sx,sy,dashed=false){
   if(points.length<2)return;
   ctx.save();ctx.strokeStyle='#9dcc82';ctx.lineWidth=Math.max(2,canvas.width/420);ctx.lineJoin='round';ctx.lineCap='round';
   if(dashed){ctx.strokeStyle='#f6c760';ctx.setLineDash([8,6])}
   ctx.beginPath();points.forEach((point,index)=>{const x=number(point[0])*sx,y=number(point[1])*sy;if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.stroke();ctx.restore();
 }
 function clearLane(message='RECORD · UFLD 결과 없음'){
   lastPerception=null;resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);
   meta.textContent=message;meta.style.color='#d5b878';backendEl.textContent='--';confidenceEl.textContent='--';inferenceEl.textContent='--';authorityEl.textContent='NONE';
 }
 function renderLane(row){
   lastPerception=row||null;resizeCanvas();ctx.clearRect(0,0,canvas.width,canvas.height);
   if(!row){clearLane();return}
   const size=jsonValue(row.lane_image_size_json,[]);
   const sourceWidth=number(size?.[0],video.videoWidth||frame.naturalWidth||640);
   const sourceHeight=number(size?.[1],video.videoHeight||frame.naturalHeight||360);
   setStageRatio(sourceWidth,sourceHeight);resizeCanvas();
   const sx=canvas.width/Math.max(1,sourceWidth),sy=canvas.height/Math.max(1,sourceHeight);
   const left=linePoints(row,'lane_left_json'),right=linePoints(row,'lane_right_json'),center=linePoints(row,'lane_center_json');
   const detected=truthy(row.lane_detected);
   if(detected&&left.length>1&&right.length>1){
     ctx.save();ctx.fillStyle='rgba(157,204,130,.10)';ctx.beginPath();
     left.forEach((point,index)=>{const x=number(point[0])*sx,y=number(point[1])*sy;if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});
     [...right].reverse().forEach(point=>ctx.lineTo(number(point[0])*sx,number(point[1])*sy));ctx.closePath();ctx.fill();ctx.restore();
   }
   drawLine(left,sx,sy,false);drawLine(right,sx,sy,false);drawLine(center,sx,sy,true);
   const confidence=Math.max(0,Math.min(1,number(row.lane_confidence,0)));
   const backend=String(row.lane_backend||'--');
   const inference=number(row.lane_inference_ms,NaN);
   const authority=String(row.lane_control_authority||'NONE');
   backendEl.textContent=backend;
   confidenceEl.textContent=Number.isFinite(confidence)?`${(confidence*100).toFixed(0)}%`:'--';
   inferenceEl.textContent=Number.isFinite(inference)?`${inference.toFixed(1)} ms`:'--';
   authorityEl.textContent=authority;
   meta.textContent=detected?`UFLD LOCK ${(confidence*100).toFixed(0)}% · ${backend}`:`UFLD SEARCH · ${String(row.lane_error||'NO LANE')}`;
   meta.style.color=detected?'#b9d8a6':'#d5b878';
 }
 function updateSeek(offset,duration){
   slider.max=String(duration);slider.value=String(offset);
   if(offsetLabel)offsetLabel.textContent=`${offset.toFixed(1)}초`;
   const durationEl=document.getElementById('record-replay-duration');if(durationEl)durationEl.textContent=`${duration.toFixed(1)}초`;
   const ratio=duration>0?Math.max(0,Math.min(1,offset/duration)):0;slider.style.setProperty('--rr-progress',`${(ratio*100).toFixed(3)}%`);
 }
 function renderReplay(result,row,ufldStatus){
   const state=result?.state||{};
   const offset=number(result?.offset_seconds,0),duration=number(result?.duration_seconds,0);
   updateSeek(offset,duration);
   if(row){renderLane(row)}
   else if(ufldStatus?.state==='running')clearLane('RECORD · UFLD 분석 중');
   else if(ufldStatus&&!ufldStatus.available&&!ufldStatus.native_ufld)clearLane('RECORD · UFLD 다시 분석 필요');
   else renderLane(state.perception||null);
   const mode=state?.vehicle_state?.mode||state?.vehicle_state?.canonical_mode||'';
   status.innerHTML=`<div class="user-summary-card"><span>확인한 시점</span><strong>${offset.toFixed(1)}초 / ${duration.toFixed(1)}초</strong><small>${mode?`당시 주행 상태: ${mode}`:'영상과 센서 기록을 불러왔습니다.'}</small></div>`;
 }
 async function fetchUfldAt(offset){
   const response=await fetch(sessionUrl('/api/recordings/ufld-analysis',`&offset=${encodeURIComponent(number(offset,0).toFixed(3))}`),{cache:'no-store'});
   let data={};try{data=await response.json()}catch(_error){}
   if(!response.ok)throw new Error(data?.error||`HTTP ${response.status}`);
   return data;
 }
 async function runReplayLoop(seekMedia=false){
   if(replayBusy||!currentSession)return;
   replayBusy=true;
   try{
     while(currentSession&&pendingReplayOffset!==null){
       const requestedOffset=number(pendingReplayOffset,0);pendingReplayOffset=null;
       const [replayResult,ufldResult]=await Promise.allSettled([
         post('/api/recordings/replay',{session:currentSession,offset_seconds:requestedOffset}),
         fetchUfldAt(requestedOffset),
       ]);
       if(replayResult.status!=='fulfilled'){
         console.error(replayResult.reason);status.innerHTML='<div class="user-summary-card"><span>주행 기록</span><strong class="warn">이 시점의 기록을 불러오지 못했습니다.</strong></div>';continue;
       }
       const result=replayResult.value;
       const state=result?.state||{};
       const ufld=ufldResult.status==='fulfilled'?ufldResult.value:null;
       let row=state.perception||null;
       if(ufld){
         if(ufld.available)row=ufld.row||null;
         else if(!ufld.native_ufld)row=null;
       }
       renderReplay(result,row,ufld);
       const actualOffset=number(result?.offset_seconds,requestedOffset);
       if(seekMedia&&!fallback&&Number.isFinite(video.duration)&&Math.abs(video.currentTime-actualOffset)>.15){video.currentTime=actualOffset}
       if(fallback)showFallbackFrame(actualOffset);
       seekMedia=false;
     }
   }finally{replayBusy=false}
 }
 function replayAt(offset,seekMedia=false){
   pendingReplayOffset=number(offset,0);
   runReplayLoop(seekMedia);
 }
 function scheduleReplay(offset,delay=100){
   clearTimeout(replayTimer);replayTimer=setTimeout(()=>replayAt(offset,false),delay);
 }
 function showFallbackFrame(offset){
   if(!currentSession)return;
   frame.src=sessionUrl('/api/recordings/frame',`&offset=${encodeURIComponent(number(offset,0).toFixed(3))}&_=${Date.now()}`);
 }
 function enableFallback(reason='브라우저에서 기록 MP4 직접 재생 불가'){
   if(fallback)return;fallback=true;video.pause();video.style.display='none';frame.style.display='block';fallbackPlay.style.display='inline-block';note.textContent=`${reason} · 서버 JPEG 프레임 재생으로 전환했습니다.`;showFallbackFrame(number(slider.value,0));replayAt(number(slider.value,0),false);
 }
 function stopFallbackPlayback(){if(fallbackTimer){clearInterval(fallbackTimer);fallbackTimer=null}fallbackPlay.textContent='프레임 재생'}
 function startFallbackPlayback(){
   if(!fallback||fallbackTimer)return;fallbackPlay.textContent='일시정지';
   fallbackTimer=setInterval(()=>{
     const max=number(slider.max,0);let next=number(slider.value,0)+0.20;if(max>0&&next>max){stopFallbackPlayback();return}
     slider.value=String(next);if(offsetLabel)offsetLabel.textContent=`${next.toFixed(1)}초`;showFallbackFrame(next);
     if(Date.now()-lastReplayAt>350){lastReplayAt=Date.now();replayAt(next,false)}
   },200);
 }
 fallbackPlay.onclick=()=>fallbackTimer?stopFallbackPlayback():startFallbackPlayback();

 function loadSession(){
   const session=select.value||'';
   if(!session||session===currentSession)return;
   stopFallbackPlayback();currentSession=session;fallback=false;lastPerception=null;pendingReplayOffset=null;
   video.style.display='block';frame.style.display='none';fallbackPlay.style.display='none';note.textContent='저장된 카메라 영상과 같은 시점의 UFLD 차선을 함께 표시합니다.';
   video.src=sessionUrl('/api/recordings/video');video.load();clearLane('RECORD · 영상 로딩');replayAt(0,false);
 }
 const originalSelectChange=select.onchange;
 select.onchange=(event)=>{if(typeof originalSelectChange==='function')originalSelectChange.call(select,event);currentSession='';loadSession()};

 const originalInput=slider.oninput;
 slider.oninput=(event)=>{
   if(typeof originalInput==='function')originalInput.call(slider,event);
   const offset=number(slider.value,0);
   if(fallback)showFallbackFrame(offset);else if(Number.isFinite(video.duration))video.currentTime=Math.min(offset,video.duration||offset);
   scheduleReplay(offset,90);
 };
 slider.addEventListener('change',()=>replayAt(number(slider.value,0),true));

 video.addEventListener('loadedmetadata',()=>{
   if(video.videoWidth&&video.videoHeight)setStageRatio(video.videoWidth,video.videoHeight);
   if(Number.isFinite(video.duration)&&video.duration>0&&number(slider.max,0)<=0)slider.max=String(video.duration);
 });
 video.addEventListener('canplay',()=>{if(!fallback)note.textContent='영상 재생 중 UFLD sidecar를 재생 시점에 맞춰 동기화합니다.'});
 video.addEventListener('error',()=>enableFallback());
 video.addEventListener('timeupdate',()=>{
   if(fallback||video.paused)return;
   const offset=number(video.currentTime,0);slider.value=String(offset);if(offsetLabel)offsetLabel.textContent=`${offset.toFixed(1)}초`;
   if(Date.now()-lastReplayAt>350){lastReplayAt=Date.now();replayAt(offset,false)}
 });
 video.addEventListener('seeked',()=>replayAt(number(video.currentTime,0),false));
 frame.addEventListener('load',()=>{if(frame.naturalWidth&&frame.naturalHeight)setStageRatio(frame.naturalWidth,frame.naturalHeight);if(lastPerception)renderLane(lastPerception)});
 window.addEventListener('resize',()=>{if(lastPerception)renderLane(lastPerception);else resizeCanvas()});
 window.addEventListener('pagehide',stopFallbackPlayback,{once:true});

 const sessionWatcher=setInterval(()=>{if(select.value&&select.value!==currentSession)loadSession()},500);
 window.addEventListener('pagehide',()=>clearInterval(sessionWatcher),{once:true});
 loadSession();
})();
</script>
'''.encode('utf-8')

__all__ = ["RECORD_REPLAY_AUTO_HMI"]
