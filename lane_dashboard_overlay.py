'''Server-authoritative lane overlay for the primary operator dashboard.'''

from lane_neural_preview import install_lane_neural_preview_endpoint


# Install a read-only diagnostic endpoint on the legacy handler. The endpoint
# resolves the final HybridLaneController lazily after production startup.
install_lane_neural_preview_endpoint()


LANE_DASHBOARD_OVERLAY = r'''
<style>
/* Disable the legacy browser lane detector UI. The dashboard renders the exact
   same server geometry used by the Pi perception stack. */
#lane-canvas{opacity:0!important}
#lane-meta{display:none!important}
#lane-physical-canvas{
  position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:2
}
#detection-canvas{z-index:3}
#lane-physical-meta{
  position:absolute;right:14px;top:12px;z-index:5;padding:7px 10px;
  border:1px solid #f6c76066;border-radius:6px;background:#05080bcc;
  color:#f6c760;font:700 10px ui-monospace,monospace;pointer-events:none
}
#lane-neural-preview-toggle{
  min-height:28px;padding:5px 9px;border:1px solid #31515a;border-radius:7px;
  background:#0b171b;color:#8da0a7;font:750 8px ui-monospace,monospace;
  letter-spacing:.06em;cursor:pointer;white-space:nowrap
}
#lane-neural-preview-toggle:hover{border-color:#41e4d288;color:#b9d9df}
#lane-neural-preview-toggle.active{
  border-color:#41e4d2;background:#102b31;color:#9de7de;
  box-shadow:0 0 12px #41e4d222
}
</style>
<script>
(function(){
  try{laneDetectionStopped=true}catch(_error){}

  const image=document.getElementById('camera-stream');
  const wrap=image?.closest('.camera-wrap');
  if(!image||!wrap||document.getElementById('lane-physical-canvas'))return;

  const canvas=document.createElement('canvas');
  canvas.id='lane-physical-canvas';
  canvas.className='lane-canvas';
  wrap.appendChild(canvas);
  const ctx=canvas.getContext('2d');

  const meta=document.createElement('div');
  meta.id='lane-physical-meta';
  meta.textContent='LANE PI CV · 준비 중';
  wrap.appendChild(meta);

  const storageKey='swing.lane.neuralPreview';
  let previewEnabled=false;
  try{previewEnabled=localStorage.getItem(storageKey)==='1'}catch(_error){}

  const header=image.closest('.camera-panel')?.querySelector('.panel-head');
  const actions=header?.querySelector('.panel-head-actions')||header;
  const previewButton=document.createElement('button');
  previewButton.id='lane-neural-preview-toggle';
  previewButton.type='button';
  previewButton.title='MANUAL/DISARMED에서 외부 UFLD 차선 모델을 표시용으로 실행합니다. 모터 제어에는 연결되지 않습니다.';
  if(actions)actions.appendChild(previewButton);

  let requestInFlight=false;
  let stopped=false;
  let lastSequence=null;
  let frames=0;
  let fpsStarted=performance.now();
  let displayFps=0;

  function renderPreviewButton(){
    previewButton.classList.toggle('active',previewEnabled);
    previewButton.textContent=previewEnabled?'UFLD 미리보기 ON':'UFLD 미리보기';
  }

  function setPreviewEnabled(value){
    previewEnabled=Boolean(value);
    try{localStorage.setItem(storageKey,previewEnabled?'1':'0')}catch(_error){}
    lastSequence=null;frames=0;fpsStarted=performance.now();displayFps=0;
    renderPreviewButton();
    meta.textContent=previewEnabled?'UFLD PREVIEW · 준비 중':'LANE PI CV · 준비 중';
  }

  previewButton.addEventListener('click',()=>setPreviewEnabled(!previewEnabled));
  renderPreviewButton();

  function resize(){
    const width=Math.max(1,Math.round(image.clientWidth||image.naturalWidth||1280));
    const height=Math.max(1,Math.round(image.clientHeight||image.naturalHeight||720));
    if(canvas.width!==width||canvas.height!==height){canvas.width=width;canvas.height=height}
  }

  function points(document){
    const value=document?.points;
    if(!Array.isArray(value))return[];
    return value.filter(point=>
      Array.isArray(point)&&point.length>=2&&
      Number.isFinite(Number(point[0]))&&Number.isFinite(Number(point[1]))
    );
  }

  function drawPolyline(polyline,sx,sy,color,width=2,dashed=false){
    if(polyline.length<2)return;
    ctx.save();
    ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineJoin='round';ctx.lineCap='round';
    if(dashed)ctx.setLineDash([9,7]);
    ctx.beginPath();
    polyline.forEach((point,index)=>{
      const x=Number(point[0])*sx,y=Number(point[1])*sy;
      if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
    });
    ctx.stroke();ctx.restore();
  }

  function draw(result){
    resize();
    ctx.clearRect(0,0,canvas.width,canvas.height);
    const sourceWidth=Number(result?.image_size?.[0]||640);
    const sourceHeight=Number(result?.image_size?.[1]||360);
    const sx=canvas.width/Math.max(1,sourceWidth);
    const sy=canvas.height/Math.max(1,sourceHeight);

    const roiTop=Number(result?.roi?.top);
    if(Number.isFinite(roiTop)){
      ctx.save();ctx.strokeStyle='rgba(255,255,255,.18)';ctx.lineWidth=1;ctx.setLineDash([6,7]);
      ctx.beginPath();ctx.moveTo(0,roiTop*sy);ctx.lineTo(canvas.width,roiTop*sy);ctx.stroke();ctx.restore();
    }

    const left=points(result?.left_line);
    const right=points(result?.right_line);
    const center=points(result?.center_line);

    if(result?.detected&&left.length>1&&right.length>1){
      ctx.save();ctx.fillStyle=previewEnabled?'rgba(76,226,208,.09)':'rgba(157,204,130,.10)';ctx.beginPath();
      left.forEach((point,index)=>{
        const x=Number(point[0])*sx,y=Number(point[1])*sy;
        if(index===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)
      });
      [...right].reverse().forEach(point=>
        ctx.lineTo(Number(point[0])*sx,Number(point[1])*sy)
      );
      ctx.closePath();ctx.fill();ctx.restore();
      const boundaryColor=previewEnabled?'#41e4d2':'#9dcc82';
      drawPolyline(left,sx,sy,boundaryColor,Math.max(2,canvas.width/420));
      drawPolyline(right,sx,sy,boundaryColor,Math.max(2,canvas.width/420));
      drawPolyline(center,sx,sy,'#f6c760',Math.max(2,canvas.width/520),true);
    }else{
      drawPolyline(left,sx,sy,'rgba(255,176,64,.82)',2,true);
      drawPolyline(right,sx,sy,'rgba(255,176,64,.82)',2,true);
    }

    ctx.save();ctx.strokeStyle='rgba(76,226,208,.70)';ctx.lineWidth=1.3;ctx.setLineDash([5,7]);
    const top=(Number.isFinite(roiTop)?roiTop:sourceHeight*.42)*sy;
    ctx.beginPath();ctx.moveTo(canvas.width/2,canvas.height);ctx.lineTo(canvas.width/2,Math.max(0,top));ctx.stroke();ctx.restore();
  }

  function updateFps(sequence){
    if(sequence!==null&&sequence!==undefined&&sequence!==lastSequence){
      frames+=1;lastSequence=sequence
    }
    const now=performance.now(),elapsed=now-fpsStarted;
    if(elapsed>=1000){displayFps=frames*1000/elapsed;frames=0;fpsStarted=now}
  }

  function setMeta(result){
    const confidence=Math.max(0,Math.min(1,Number(result?.confidence||0)));
    const marking=String(result?.marking||'EDGE');
    const backend=String(result?.backend||(previewEnabled?'UFLD_ONNX':'CLASSICAL_CV')).replaceAll('_',' ');
    const age=Number(result?.data_age);
    const ageText=Number.isFinite(age)?` · ${(age*1000).toFixed(0)}ms`:'';
    const diag=result?.preview_diagnostics||{};
    const inference=Number(diag?.inference_ms);
    const inferenceText=previewEnabled&&Number.isFinite(inference)?` · NN ${inference.toFixed(0)}ms`:'';
    const slow=previewEnabled&&diag?.latency_allowed===false?' · SLOW':'';
    const prefix=previewEnabled?'UFLD PREVIEW':'LANE';
    if(result?.detected){
      const inferred=result?.inferred_left||result?.inferred_right?' · 추정 경계 포함':'';
      meta.textContent=`${prefix} LOCK ${(confidence*100).toFixed(0)}% · PI ${backend} ${displayFps.toFixed(1)} FPS · ${marking}${inferred}${inferenceText}${slow}${ageText}`;
      meta.style.color=previewEnabled?'#9de7de':'#9dcc82';meta.style.borderColor=previewEnabled?'#2d8078':'#31553a';
    }else{
      meta.textContent=`${prefix} SEARCHING · PI ${backend} ${displayFps.toFixed(1)} FPS · ${String(result?.error||'NO LANE')}${inferenceText}${slow}${ageText}`;
      meta.style.color='#f6c760';meta.style.borderColor='#6a5424';
    }
  }

  async function refresh(){
    if(stopped||requestInFlight||document.hidden)return;
    requestInFlight=true;
    try{
      const endpoint=previewEnabled?'/api/lane/neural-preview':'/api/lane';
      const response=await fetch(endpoint,{cache:'no-store'});
      const result=await response.json();
      if(!response.ok){
        if(previewEnabled&&response.status===409){
          setPreviewEnabled(false);
        }
        throw new Error(String(result?.error||`HTTP ${response.status}`));
      }
      updateFps(result?.frame_sequence);
      draw(result);setMeta(result);
    }catch(error){
      meta.textContent=`${previewEnabled?'UFLD PREVIEW':'LANE PI CV'} · ${String(error?.message||error)}`;
      meta.style.color='#e48f8f';meta.style.borderColor='#6f2929';
    }finally{requestInFlight=false}
  }

  const timer=setInterval(refresh,180);
  window.addEventListener('pagehide',()=>{
    stopped=true;clearInterval(timer)
  },{once:true});
  refresh();
})();
</script>
'''.encode('utf-8')

__all__ = ['LANE_DASHBOARD_OVERLAY']
