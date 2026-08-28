'''Diagnostic overlay for rejected raw UFLD candidates in offline RECORD replay.'''

# Importing this module installs the backend sidecar-field patch before an
# operator can start/restart an offline UFLD analysis.
import record_replay_ufld_candidate_patch  # noqa: F401
from record_model_preview_hmi import RECORD_MODEL_PREVIEW_HMI


RECORD_REPLAY_CANDIDATE_OVERLAY_HMI = r'''
<style>
#record-replay-candidate-canvas{
  position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:4
}
#record-replay-candidate-meta{
  position:absolute;left:10px;bottom:10px;z-index:5;padding:8px 11px;
  border:2px solid #ffb000;border-radius:7px;background:#050505f2;color:#ffd45a;
  box-shadow:0 0 0 1px #000,0 2px 12px #000c;
  font:800 11px ui-monospace,monospace;display:none
}
</style>
<script>
(function(){
 const stage=document.getElementById('record-replay-stage');
 const select=document.getElementById('record-manage-session');
 const slider=document.getElementById('record-replay-slider');
 const video=document.getElementById('record-replay-video');
 if(!stage||!select||!slider)return;

 const canvas=document.createElement('canvas');
 canvas.id='record-replay-candidate-canvas';
 const badge=document.createElement('div');
 badge.id='record-replay-candidate-meta';
 stage.appendChild(canvas);stage.appendChild(badge);
 const ctx=canvas.getContext('2d');
 const candidateColors=['#00e5ff','#ff3bd4','#ffb000','#b8ff00'];
 let timer=null,lastFetch=0,token=0,lastObservedOffset=null;

 function number(value,fallback=0){const n=Number(value);return Number.isFinite(n)?n:fallback}
 function truthy(value){return value===true||String(value).toLowerCase()==='true'||String(value)==='1'}
 function parsed(value,fallback){
   if(value===null||value===undefined||value==='')return fallback;
   if(typeof value==='object')return value;
   try{return JSON.parse(value)}catch(_error){return fallback}
 }
 function resize(){
   const w=Math.max(1,Math.round(stage.clientWidth)),h=Math.max(1,Math.round(stage.clientHeight));
   if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h}
 }
 function clear(){resize();ctx.clearRect(0,0,canvas.width,canvas.height);badge.style.display='none'}
 function pathFor(points){
   ctx.beginPath();points.forEach((p,i)=>{const x=number(p?.[0])*canvas.width,y=number(p?.[1])*canvas.height;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});
 }
 function label(text,x,y,color){
   ctx.save();ctx.font='800 13px ui-monospace,monospace';
   const padX=5,padY=4,width=ctx.measureText(text).width+padX*2,height=20;
   const left=Math.max(2,Math.min(canvas.width-width-2,x));
   const top=Math.max(2,Math.min(canvas.height-height-2,y-height));
   ctx.fillStyle='rgba(0,0,0,.88)';ctx.fillRect(left,top,width,height);
   ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.strokeRect(left+.5,top+.5,width-1,height-1);
   ctx.fillStyle=color;ctx.fillText(text,left+padX,top+height-padY-2);ctx.restore();
 }
 function draw(row){
   clear();if(!row)return;
   if(truthy(row.lane_detected))return;
   if(!Object.prototype.hasOwnProperty.call(row,'lane_candidates_json')){
     if(String(row.lane_error||'').includes('NEURAL_EGO_LANE_PAIR_REQUIRED')){
       badge.textContent='후보선 좌표 없음 · 기존 영상을 UFLD 재분석하세요';badge.style.display='block';
     }
     return;
   }
   const lanes=parsed(row.lane_candidates_json,[]);
   if(!Array.isArray(lanes)||!lanes.length){
     badge.textContent=`UFLD 후보 없음 · ${String(row.lane_error||'자차선 판정 실패')}`;badge.style.display='block';return;
   }
   resize();
   lanes.forEach((lane,index)=>{
     const points=Array.isArray(lane?.normalized_points)?lane.normalized_points:[];
     if(points.length<2)return;
     const color=candidateColors[index%candidateColors.length];
     ctx.save();ctx.lineCap='round';ctx.lineJoin='round';ctx.setLineDash([14,9]);
     // Dark under-stroke keeps the diagnostic line visible over white road paint.
     ctx.strokeStyle='rgba(0,0,0,.92)';ctx.lineWidth=Math.max(7,canvas.width/210);pathFor(points);ctx.stroke();
     ctx.strokeStyle=color;ctx.lineWidth=Math.max(4,canvas.width/340);pathFor(points);ctx.stroke();ctx.restore();
     const p=points[Math.max(0,points.length-1)];
     label(`UFLD ${lane?.lane_id??index} ${(number(lane?.confidence)*100).toFixed(0)}%`,number(p?.[0])*canvas.width+7,number(p?.[1])*canvas.height-3,color);
   });
   badge.textContent=`UFLD 원시 후보 ${lanes.length}개 · 자차선 안전 판정 실패`;badge.style.display='block';
 }
 async function refresh(offset){
   const session=select.value||'';if(!session){clear();return}
   const mine=++token;
   try{
     const response=await fetch(`/api/recordings/ufld-analysis?session=${encodeURIComponent(session)}&offset=${encodeURIComponent(number(offset).toFixed(3))}`,{cache:'no-store'});
     const data=await response.json();if(mine!==token)return;if(!response.ok){clear();return}draw(data?.row||null);
   }catch(_error){if(mine===token)clear()}
 }
 function schedule(offset,delay=100){clearTimeout(timer);timer=setTimeout(()=>refresh(offset),delay)}
 function observeReplayClock(force=false){
   const offset=number(slider.value,0);
   if(!force&&lastObservedOffset!==null&&Math.abs(offset-lastObservedOffset)<0.045)return;
   lastObservedOffset=offset;
   schedule(offset,35);
 }
 slider.addEventListener('input',()=>{lastObservedOffset=number(slider.value,0);schedule(slider.value,70)});
 slider.addEventListener('change',()=>{lastObservedOffset=number(slider.value,0);refresh(slider.value)});
 select.addEventListener('change',()=>{lastObservedOffset=null;setTimeout(()=>observeReplayClock(true),50)});
 if(video){
   video.addEventListener('timeupdate',()=>{if(video.paused)return;const now=Date.now();if(now-lastFetch<350)return;lastFetch=now;lastObservedOffset=number(video.currentTime,0);refresh(video.currentTime)});
   video.addEventListener('seeked',()=>{lastObservedOffset=number(video.currentTime,0);refresh(video.currentTime)});
 }
 // JPEG fallback advances slider.value programmatically and does not fire the
 // native input/change events. Observe the replay clock itself so candidate
 // overlays follow every automatically displayed frame as well.
 const replayClockTimer=setInterval(()=>observeReplayClock(false),120);
 window.addEventListener('resize',()=>schedule(slider.value,20));
 window.addEventListener('pagehide',()=>{clearInterval(replayClockTimer);clearTimeout(timer)},{once:true});
 setTimeout(()=>observeReplayClock(true),250);
})();
</script>
'''.encode('utf-8') + RECORD_MODEL_PREVIEW_HMI


__all__ = ["RECORD_REPLAY_CANDIDATE_OVERLAY_HMI"]
