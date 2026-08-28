'''One-click offline UFLD reanalysis for finalized RECORD sessions.'''

from record_replay_ufld_frames import install_frame_replay_ufld
from record_replay_ufld import install_record_replay_ufld_endpoints


install_frame_replay_ufld()
install_record_replay_ufld_endpoints()


RECORD_REPLAY_OFFLINE_UFLD_HMI = r'''
<style>
#record-replay-offline-status{font-size:10px;color:#919d93}
#record-replay-offline-analyze.good{border-color:#31553a;background:#17331d;color:#9dcc82}
#record-replay-offline-analyze:disabled{opacity:.55;cursor:not-allowed}
</style>
<script>
(function(){
 const select=document.getElementById('record-manage-session');
 const slider=document.getElementById('record-replay-slider');
 const toolbar=document.querySelector('#record-replay-media .record-replay-toolbar');
 if(!select||!slider||!toolbar)return;

 const button=document.createElement('button');
 button.id='record-replay-offline-analyze';
 button.type='button';
 button.textContent='UFLD 다시 분석';
 const label=document.createElement('span');
 label.id='record-replay-offline-status';
 label.textContent='';
 toolbar.appendChild(button);
 toolbar.appendChild(label);

 let session='';
 let pollTimer=null;
 let requestToken=0;

 function stopPoll(){if(pollTimer){clearInterval(pollTimer);pollTimer=null}}
 function errorText(error){
   const text=String(error||'');
   if(text.includes('UFLD_ANALYSIS_REQUIRES_DISARMED'))return '차량을 DISARMED로 바꾼 뒤 분석해 주세요.';
   if(text.includes('STOP_RECORDING_BEFORE_UFLD_ANALYSIS'))return '진행 중인 RECORD를 먼저 종료해 주세요.';
   if(text.includes('UFLD_MODEL_UNAVAILABLE'))return 'Worker UFLD 연결/모델을 먼저 확인해 주세요.';
   if(text.includes('UFLD_ANALYSIS_BUSY'))return '다른 주행 기록을 분석 중입니다.';
   if(text.includes('UFLD_ANALYSIS_INTERRUPTED'))return '차량 상태가 바뀌어 분석을 중단했습니다.';
   return text||'UFLD 분석 오류';
 }
 async function getStatus(offset=null){
   if(!session)return null;
   const suffix=offset===null?'':`&offset=${encodeURIComponent(Number(offset||0).toFixed(3))}`;
   const response=await fetch(`/api/recordings/ufld-analysis?session=${encodeURIComponent(session)}${suffix}`,{cache:'no-store'});
   const data=await response.json();
   if(!response.ok)throw new Error(data?.error||`HTTP ${response.status}`);
   return data;
 }
 function refreshReplay(){slider.dispatchEvent(new Event('change',{bubbles:true}))}
 function renderStatus(data){
   if(!data)return;
   button.style.display='inline-block';
   if(data.state==='running'){
     const pct=Math.max(0,Math.min(100,Number(data.progress||0)*100));
     button.disabled=true;
     button.classList.remove('good');
     button.textContent='UFLD 분석 중';
     label.textContent=`${pct.toFixed(0)}% · ${Number(data.processed||0)} / ${Number(data.total||0)||'?'} 시점`;
     if(!pollTimer)pollTimer=setInterval(()=>refreshStatus(false),1000);
     return;
   }
   stopPoll();
   button.disabled=false;
   button.textContent='UFLD 다시 분석';
   if(data.state==='completed'&&data.available){
     button.classList.add('good');
     const rows=Number(data.metadata?.rows||data.processed||0);
     const period=Number(data.metadata?.sample_period_seconds||0);
     const duration=Number(data.metadata?.recorded_duration_seconds||0);
     label.textContent=`UFLD 재분석 완료 · ${rows}개 시점${duration?` · 실제 ${duration.toFixed(1)}초`:''}${period?` · ${period.toFixed(2)}초 간격`:''}`;
     refreshReplay();
     return;
   }
   button.classList.remove('good');
   if(data.state==='failed'){
     label.textContent=errorText(data.error);
   }else if(data.native_ufld){
     label.textContent='RECORD 당시 UFLD 기록이 있어도 현재 Worker/모델로 다시 분석할 수 있습니다.';
   }else{
     label.textContent='UFLD 결과가 없으면 현재 Worker/모델로 이 RECORD를 다시 분석합니다.';
   }
 }
 async function refreshStatus(resetSession=true){
   const selected=select.value||'';
   if(resetSession&&selected!==session){session=selected;stopPoll()}
   if(!session){button.style.display='none';label.textContent='';return}
   const token=++requestToken;
   try{
     const data=await getStatus();
     if(token!==requestToken)return;
     renderStatus(data);
   }catch(error){
     if(token!==requestToken)return;
     button.disabled=false;button.style.display='inline-block';label.textContent=errorText(error.message);
   }
 }
 button.onclick=async()=>{
   if(!session)return;
   button.disabled=true;label.textContent='현재 Worker UFLD로 재분석 시작 요청 중…';
   try{
     const data=await post('/api/recordings/ufld-analysis',{session,force:true});
     renderStatus(data);
   }catch(error){
     button.disabled=false;label.textContent=errorText(error.message||error);
   }
 };
 select.addEventListener('change',()=>setTimeout(()=>refreshStatus(true),0));
 const watcher=setInterval(()=>{if(select.value&&select.value!==session)refreshStatus(true)},700);
 window.addEventListener('pagehide',()=>{stopPoll();clearInterval(watcher)},{once:true});
 setTimeout(()=>refreshStatus(true),100);
})();
</script>
'''.encode('utf-8')


__all__ = ["RECORD_REPLAY_OFFLINE_UFLD_HMI"]
