"""User-facing PC compute worker status for the settings page."""

COMPUTE_WORKER_HMI = r'''
<style id="compute-worker-hmi-style">
#compute-worker-panel .compute-worker-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:7px}
#compute-worker-panel .compute-worker-cell{min-width:0;padding:8px;border:1px solid var(--line);border-radius:5px;background:rgba(255,255,255,.012)}
#compute-worker-panel .compute-worker-cell span{display:block;color:var(--muted);font-size:9px;letter-spacing:.05em}
#compute-worker-panel .compute-worker-cell strong{display:block;margin-top:4px;overflow:hidden;text-overflow:ellipsis;font:700 11px ui-monospace,monospace;white-space:nowrap}
#compute-worker-panel .compute-worker-actions{display:flex;gap:7px;align-items:center;flex-wrap:wrap;margin-top:9px}
#compute-worker-panel .compute-worker-detail{margin-top:8px;color:var(--muted);font-size:9px;line-height:1.55}
@media(max-width:850px){#compute-worker-panel .compute-worker-grid{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:520px){#compute-worker-panel .compute-worker-grid{grid-template-columns:1fr}}
</style>
<script>
(function(){
 const WORKER='http://127.0.0.1:8765';
 const grid=document.querySelector('#view-system .grid');
 if(!grid)return;
 const panel=document.createElement('div');
 panel.id='compute-worker-panel';
 panel.className='panel span12';
 panel.innerHTML=`
   <h2>컴퓨팅 PC</h2>
   <p class="sectionnote">이 화면을 열고 있는 Windows PC의 SWING Compute Worker를 확인합니다. 차량의 모터·조향·E-STOP·LiDAR 안전 기능은 Worker와 독립적으로 Raspberry Pi에서 동작합니다.</p>
   <div id="compute-worker-grid" class="compute-worker-grid">
    <div class="compute-worker-cell"><span>WORKER</span><strong id="cw-state">확인 중</strong></div>
    <div class="compute-worker-cell"><span>PC</span><strong id="cw-host">-</strong></div>
    <div class="compute-worker-cell"><span>CPU / RAM</span><strong id="cw-cpu">-</strong></div>
    <div class="compute-worker-cell"><span>AI COMPUTE</span><strong id="cw-ai">-</strong></div>
   </div>
   <div class="compute-worker-actions"><button id="cw-open">Worker 상태 열기</button><button id="cw-test">연결 시험</button><span id="cw-note" class="pill">localhost:8765</span></div>
   <div id="cw-detail" class="compute-worker-detail">Windows에서 SWING Compute Worker 앱을 열고 'Worker 시작'을 누르세요. 앱이 없으면 GitHub Releases의 Setup.exe로 설치합니다.</div>`;
 grid.appendChild(panel);
 const el=id=>document.getElementById(id);
 const gib=n=>Number.isFinite(Number(n))?(Number(n)/1073741824).toFixed(1)+' GB':'-';
 let lastOk=0;
 async function workerFetch(path,options={}){
   const controller=new AbortController();
   const timer=setTimeout(()=>controller.abort(),1200);
   try{return await fetch(WORKER+path,{cache:'no-store',...options,signal:controller.signal})}
   finally{clearTimeout(timer)}
 }
 function disconnected(error){
   el('cw-state').textContent='미연결';el('cw-state').className='warn';
   el('cw-host').textContent='-';el('cw-cpu').textContent='-';el('cw-ai').textContent='-';
   el('cw-detail').textContent='Windows의 SWING Compute Worker 앱에서 Worker 시작을 눌러 주세요. '+(error?.message||'');
 }
 async function refresh(){
   try{
     const response=await workerFetch('/api/v1/status');
     if(!response.ok)throw new Error('HTTP '+response.status);
     const s=await response.json();const c=s.capabilities||{};
     lastOk=Date.now();el('cw-state').textContent='연결됨 · v'+(s.version||'-');el('cw-state').className='good';
     el('cw-host').textContent=s.hostname||'-';
     const threads=s.cpu?.logical_count||'-';el('cw-cpu').textContent=`${threads} threads · ${gib(s.memory?.total_bytes)}`;
     const compute=c.cuda?(c.gpu_name||'CUDA GPU'):c.local_cpu_training?'CPU 학습':'상태 확인만';
     el('cw-ai').textContent=compute;
     const features=[];if(c.openvino)features.push('OpenVINO');if(c.onnx_runtime)features.push('ONNX Runtime');if(c.record_cache)features.push('RECORD cache');
     el('cw-detail').textContent=`${features.join(' · ')||'기본 Worker'} 사용 가능 · 저장공간 ${gib(s.disk?.free_bytes)} 남음 · ${s.data_root||''}`;
   }catch(error){if(Date.now()-lastOk>3000)disconnected(error)}
 }
 el('cw-open').onclick=()=>window.open(WORKER+'/','_blank','noopener');
 el('cw-test').onclick=async()=>{
   el('cw-test').disabled=true;
   try{
     const response=await workerFetch('/api/v1/jobs',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({kind:'diagnostic'})});
     const result=await response.json();if(!response.ok)throw new Error(result.error||'연결 시험 실패');
     el('cw-note').textContent='시험 작업 '+result.job_id+' 시작';await refresh();
   }catch(error){el('cw-note').textContent='연결 시험 실패';disconnected(error)}
   finally{el('cw-test').disabled=false}
 };
 refresh();setInterval(refresh,2500);
})();
</script>
'''.encode('utf-8')

__all__ = ["COMPUTE_WORKER_HMI"]
