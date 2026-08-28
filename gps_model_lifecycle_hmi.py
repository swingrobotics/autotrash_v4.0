'''Guided route/model selection, interactive route preview, and lifecycle controls for AUTO_GPS.'''

from gps_route_preview_api import install_gps_route_preview_api


install_gps_route_preview_api()


GPS_MODEL_LIFECYCLE_HMI = r'''
<style>
#gps-model-help{margin-top:9px;padding:9px 10px;border-left:3px solid rgba(255,255,255,.14);background:#111512;color:#8e9790;font-size:9px;line-height:1.5}
#gps-model-help.good{border-left-color:var(--ok);color:#aebcad}#gps-model-help.warn{border-left-color:var(--warn);color:#c1b58f}#gps-model-help.bad{border-left-color:var(--bad);color:#c99ca3}
#gps-route-preview-wrap{margin-top:10px;padding:10px;border:1px solid rgba(255,255,255,.08);border-radius:8px;background:#0c100d}
#gps-route-preview-head{display:flex;justify-content:space-between;gap:12px;align-items:flex-start;margin-bottom:8px}
#gps-route-preview-head b{font-size:10px;color:#d8e0d8}#gps-route-preview-head span{font:8px ui-monospace,monospace;color:#7f8981;text-align:right}
#gps-route-map{position:relative;width:100%;height:320px;overflow:hidden;border:1px solid rgba(255,255,255,.07);border-radius:7px;background:#0c120e;cursor:grab;touch-action:none;user-select:none}
#gps-route-map.dragging{cursor:grabbing}
#gps-route-map-tiles{position:absolute;inset:0;overflow:hidden;background:#111712;will-change:transform}
#gps-route-map-tiles img{position:absolute;width:256px;height:256px;max-width:none;object-fit:cover;filter:brightness(.76) saturate(.72) contrast(1.08);pointer-events:none}
#gps-route-map-fallback{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;padding:18px;text-align:center;color:#778179;font:9px ui-monospace,monospace;background-color:#0d130f;background-image:linear-gradient(rgba(255,255,255,.035) 1px,transparent 1px),linear-gradient(90deg,rgba(255,255,255,.035) 1px,transparent 1px);background-size:40px 40px;pointer-events:none}
#gps-route-map-fallback.tiles-ready{display:none}
#gps-route-preview{position:absolute;inset:0;display:block;width:100%;height:100%;z-index:3;pointer-events:none;will-change:transform}
#gps-route-map-controls{position:absolute;left:8px;top:8px;z-index:6;display:flex;align-items:center;gap:4px;padding:4px;border:1px solid rgba(255,255,255,.12);border-radius:8px;background:rgba(8,12,9,.88);backdrop-filter:blur(5px)}
#gps-route-map-controls button{min-width:34px;height:34px;padding:0 8px;border-radius:6px;background:#19211a;color:#edf4ed;border:1px solid #465248;font-size:13px;line-height:1}
#gps-route-map-controls button:hover:not(:disabled){border-color:var(--accent)}#gps-route-map-controls button:disabled{opacity:.4}
#gps-route-map-controls .fit{font-size:8px;font-weight:800;letter-spacing:.04em}#gps-route-map-zoom{min-width:30px;text-align:center;color:#c6d1c8;font:8px ui-monospace,monospace}
#gps-route-map-attribution{position:absolute;right:5px;bottom:4px;z-index:4;padding:2px 5px;border-radius:3px;background:rgba(8,12,9,.78);color:#a8b0aa;font-size:7px;line-height:1.3}
#gps-route-map-attribution a{color:#c2cbc4;text-decoration:none}
#gps-route-preview-note{margin-top:7px;color:#7f8981;font-size:8px;line-height:1.5;white-space:pre-wrap}
#gps-model-lifecycle{margin-top:10px;padding-top:10px;border-top:1px solid rgba(255,255,255,.07)}
#gps-model-lifecycle .gps-life-line{display:flex;justify-content:space-between;gap:10px;align-items:center}
#gps-model-lifecycle span{color:#858c88;font-size:9px}#gps-model-lifecycle b{font-size:10px}
#gps-model-lifecycle button{width:100%;margin-top:8px}#gps-model-lifecycle p{margin:7px 0 0;color:#7f8783;font-size:9px;line-height:1.45}
#gps-model:disabled{opacity:.68;cursor:not-allowed}
@media(max-width:650px){#gps-route-map{height:260px}#gps-route-preview-head{display:block}#gps-route-preview-head span{display:block;margin-top:4px;text-align:left}#gps-route-map-controls button{min-width:40px;height:40px}}
</style>
<script>
(function(){
 const q=id=>document.getElementById(id);
 const routeSelect=q('gps-route'),modelSelect=q('gps-model'),useButton=q('gps-select');
 if(!routeSelect||!modelSelect)return;
 const order=['TRAINED','OFFLINE_VALIDATED','CLOSED_AREA_VALIDATED','AUTO_ALLOWED'];
 const stageLabel=stage=>({TRAINED:'학습 완료',OFFLINE_VALIDATED:'기본 검증 완료',CLOSED_AREA_VALIDATED:'시험 주행 완료',AUTO_ALLOWED:'자동 주행 사용 가능'}[stage]||'확인 필요');
 const next=stage=>{const i=order.indexOf(stage);return i>=0&&i<order.length-1?order[i+1]:null};
 let draftRoute='',draftModel='',routeDirty=false,modelDirty=false,previewSerial=0;
 const previewCache=new Map();
 const mapView={routeId:'',zoom:null,fitZoom:null,centerLat:null,centerLon:null};
 let drag=null;

 function ensureUi(){
  const panel=modelSelect.closest('.panel');if(!panel)return;
  if(!q('gps-model-help')){const help=document.createElement('div');help.id='gps-model-help';help.className='warn';modelSelect.parentElement?.insertAdjacentElement('afterend',help)}
  if(!q('gps-route-preview-wrap')){
   const wrap=document.createElement('div');wrap.id='gps-route-preview-wrap';
   wrap.innerHTML='<div id="gps-route-preview-head"><b>GPS 예상 경로 지도</b><span id="gps-route-preview-meta">Route를 선택하세요</span></div><div id="gps-route-map"><div id="gps-route-map-tiles"></div><div id="gps-route-map-fallback">지도 타일을 불러오는 중입니다.<br>인터넷 연결이 없으면 경로 선만 표시됩니다.</div><svg id="gps-route-preview" role="img" aria-label="GPS normalized route preview"></svg><div id="gps-route-map-controls"><button id="gps-map-minus" type="button" aria-label="지도 축소">−</button><button id="gps-map-fit" class="fit" type="button" aria-label="전체 경로 보기">FIT</button><span id="gps-route-map-zoom">Z--</span><button id="gps-map-plus" type="button" aria-label="지도 확대">+</button></div><div id="gps-route-map-attribution">© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener noreferrer">OpenStreetMap</a> contributors</div></div><div id="gps-route-preview-note">학습 모델이 따라야 할 정규화 기준 Route를 실제 지도 위에 표시합니다. + / − 확대축소, 드래그 이동, FIT 전체 경로 보기를 사용할 수 있습니다.</div>';
   const status=q('gps-status');(status||modelSelect.parentElement)?.insertAdjacentElement('afterend',wrap);
   q('gps-map-plus')?.addEventListener('click',()=>changeZoom(1));
   q('gps-map-minus')?.addEventListener('click',()=>changeZoom(-1));
   q('gps-map-fit')?.addEventListener('click',fitCurrentRoute);
   const map=q('gps-route-map');
   map?.addEventListener('pointerdown',beginDrag);
   map?.addEventListener('pointermove',moveDrag);
   map?.addEventListener('pointerup',endDrag);
   map?.addEventListener('pointercancel',endDrag);
  }
  if(!q('gps-model-lifecycle')){const box=document.createElement('div');box.id='gps-model-lifecycle';box.innerHTML='<div class="gps-life-line"><span>GPS 모델 검증 상태</span><b id="gps-life-current">확인 중</b></div><button id="gps-life-next" type="button">다음 검증 단계로 변경</button><p>실제 검증을 완료한 경우에만 다음 단계로 변경하세요. “시험 주행 완료”부터 GPS 자율주행에 사용할 수 있습니다.</p>';panel.appendChild(box);q('gps-life-next').onclick=promote}
 }

 function option(select,value,text,selected=false){const item=document.createElement('option');item.value=value;item.textContent=text;item.selected=!!selected;select.appendChild(item);return item}
 function allRoutes(){return Array.isArray(G?.routes)?G.routes:[]}
 function allModels(){return (Array.isArray(G?.models)?G.models:[]).filter(model=>String(model.policy_type||'AUTO_GPS').toUpperCase()==='AUTO_GPS')}
 function modelsForRoute(routeId){return allModels().filter(model=>String(model.route_id||'')===String(routeId||''))}
 function selectedModel(){const id=modelSelect.value;return allModels().find(model=>String(model.model_id||'')===String(id||''))}

 function syncSelectors(beforeRoute,beforeModel){
  ensureUi();
  const routes=allRoutes(),routeIds=new Set(routes.map(route=>String(route.route_id||'')));
  const serverRoute=String(G?.selected?.route_id||''),serverModel=String(G?.selected?.model_id||'');
  let routeId='';
  if(routeDirty&&draftRoute&&routeIds.has(draftRoute))routeId=draftRoute;
  else if(beforeRoute&&routeIds.has(String(beforeRoute)))routeId=String(beforeRoute);
  else if(serverRoute&&routeIds.has(serverRoute))routeId=serverRoute;
  else if(routes.length)routeId=String(routes[0].route_id||'');
  routeSelect.innerHTML='';
  if(!routes.length){option(routeSelect,'','저장된 GPS Route 없음',true);routeSelect.disabled=true;routeId=''}else{routeSelect.disabled=false;for(const route of routes){const id=String(route.route_id||'');option(routeSelect,id,`${id} · ${Number(route.point_count||0).toLocaleString()} pts`,id===routeId)}}
  if(routeDirty&&serverRoute===draftRoute&&(!draftModel||serverModel===draftModel)){routeDirty=false}

  const matching=modelsForRoute(routeId),matchingIds=new Set(matching.map(model=>String(model.model_id||'')));
  let modelId='';
  if(modelDirty&&draftModel&&matchingIds.has(draftModel))modelId=draftModel;
  else if(beforeModel&&matchingIds.has(String(beforeModel)))modelId=String(beforeModel);
  else if(serverRoute===routeId&&serverModel&&matchingIds.has(serverModel))modelId=serverModel;
  else if(matching.length===1)modelId=String(matching[0].model_id||'');
  modelSelect.innerHTML='';
  const help=q('gps-model-help');
  if(!routeId){option(modelSelect,'','Route를 먼저 선택하세요',true);modelSelect.disabled=true;if(help){help.className='warn';help.textContent='GPS Route를 먼저 생성하거나 선택하세요.'}}
  else if(!matching.length){option(modelSelect,'','이 Route용 GPS 모델 없음',true);modelSelect.disabled=true;if(help){help.className=allModels().length?'warn':'bad';help.textContent=allModels().length?'설치된 GPS 모델은 있지만 이 Route에 학습된 모델이 없습니다. 해당 Route로 GPS 학습을 실행하세요.':'설치된 GPS 학습 모델이 없습니다. GPS 모델 학습이 완료된 뒤 차량 등록까지 성공해야 이 목록에 표시됩니다.'}}
  else{modelSelect.disabled=false;for(const model of matching){const id=String(model.model_id||'');option(modelSelect,id,`${id} · ${stageLabel(model.validation_stage)}`,id===modelId)}if(help){help.className='good';help.textContent=`${routeId}에 연결된 GPS 모델 ${matching.length}개 · 모델을 선택한 뒤 “이 설정 사용”을 누르세요.`}}
  if(modelDirty&&serverRoute===routeId&&serverModel===draftModel){modelDirty=false;draftModel=''}
  const active=!!G?.controller?.active;
  if(useButton){useButton.disabled=!routeId||!modelId||active;useButton.textContent=active?'GPS 자율주행 종료 후 변경':serverRoute===routeId&&serverModel===modelId&&modelId?'현재 설정 사용 중':'이 설정 사용'}
  syncLifecycle();requestPreview(routeId);
 }

 function syncLifecycle(){
  ensureUi();const model=selectedModel(),active=!!G?.controller?.active;const current=q('gps-life-current'),button=q('gps-life-next');
  if(current)current.textContent=model?stageLabel(model.validation_stage):(modelSelect.disabled?'사용 가능한 모델 없음':'경로에 맞는 모델을 선택하세요.');
  if(button){const target=model?next(model.validation_stage):null;button.disabled=!model||!target||active;button.textContent=active?'GPS 자율주행 종료 후 변경':!model?'모델을 먼저 선택하세요':target?`${stageLabel(target)}로 변경`:'검증 단계 완료'}
 }

 async function promote(){const model=selectedModel();if(!model)return;const target=next(model.validation_stage);if(!target)return;if(G?.controller?.active)return alert('GPS 자율주행을 먼저 종료해 주세요.');if(!confirm(`${model.model_id} GPS 모델을 “${stageLabel(target)}” 상태로 변경할까요?\n\n실제 검증을 완료한 경우에만 진행하세요.`))return;try{await post('/api/v2/ai/lifecycle',{model_id:model.model_id,stage:target,confirm:target});await refresh()}catch(error){console.error(error);alert('모델 검증 단계를 변경하지 못했습니다. 현재 상태와 검증 순서를 확인해 주세요.')}}

 function svgElement(name,attrs={}){const el=document.createElementNS('http://www.w3.org/2000/svg',name);for(const [key,value] of Object.entries(attrs))el.setAttribute(key,String(value));return el}
 function clampLat(value){return Math.max(-85.05112878,Math.min(85.05112878,Number(value)||0))}
 function worldPoint(latitude,longitude,zoom){const lat=clampLat(latitude),lon=Number(longitude)||0,n=2**zoom,size=256*n,rad=lat*Math.PI/180;return{x:(lon+180)/360*size,y:(1-Math.log(Math.tan(rad)+1/Math.cos(rad))/Math.PI)/2*size}}
 function worldToLatLon(x,y,zoom){const size=256*(2**zoom),lon=x/size*360-180,n=Math.PI-2*Math.PI*y/size,lat=180/Math.PI*Math.atan(Math.sinh(n));return{latitude:clampLat(lat),longitude:Math.max(-180,Math.min(180,lon))}}
 function chooseZoom(points,width,height,padding){for(let zoom=19;zoom>=3;zoom--){const projected=points.map(p=>worldPoint(p.latitude,p.longitude,zoom)),xs=projected.map(p=>p.x),ys=projected.map(p=>p.y),spanX=Math.max(...xs)-Math.min(...xs),spanY=Math.max(...ys)-Math.min(...ys);if(spanX<=Math.max(20,width-padding*2)&&spanY<=Math.max(20,height-padding*2))return zoom}return 3}
 function routeCenter(points,zoom){const projected=points.map(p=>worldPoint(p.latitude,p.longitude,zoom)),xs=projected.map(p=>p.x),ys=projected.map(p=>p.y),world={x:(Math.min(...xs)+Math.max(...xs))/2,y:(Math.min(...ys)+Math.max(...ys))/2};return worldToLatLon(world.x,world.y,zoom)}

 function renderTiles(zoom,width,height,centerWorld){
  const layer=q('gps-route-map-tiles'),fallback=q('gps-route-map-fallback');if(!layer||!fallback)return;
  layer.style.transform='';layer.innerHTML='';fallback.classList.remove('tiles-ready');fallback.innerHTML='지도 타일을 불러오는 중입니다.<br>인터넷 연결이 없으면 경로 선만 표시됩니다.';
  const left=centerWorld.x-width/2,top=centerWorld.y-height/2,worldTiles=2**zoom;
  const minTileX=Math.floor(left/256)-1,maxTileX=Math.floor((left+width)/256)+1,minTileY=Math.max(0,Math.floor(top/256)-1),maxTileY=Math.min(worldTiles-1,Math.floor((top+height)/256)+1);
  let loaded=0,failed=0,total=0;
  for(let tileY=minTileY;tileY<=maxTileY;tileY++)for(let tileX=minTileX;tileX<=maxTileX;tileX++){
   total++;const wrappedX=((tileX%worldTiles)+worldTiles)%worldTiles,img=document.createElement('img');img.alt='';img.draggable=false;img.loading='eager';img.referrerPolicy='no-referrer';img.style.left=`${Math.round(tileX*256-left)}px`;img.style.top=`${Math.round(tileY*256-top)}px`;img.src=`https://tile.openstreetmap.org/${zoom}/${wrappedX}/${tileY}.png`;
   img.addEventListener('load',()=>{loaded++;if(loaded>0)fallback.classList.add('tiles-ready')},{once:true});
   img.addEventListener('error',()=>{failed++;if(failed>=total&&loaded===0){fallback.classList.remove('tiles-ready');fallback.textContent='지도 타일 연결 없음 · GPS 경로만 표시 중'}},{once:true});layer.appendChild(img)
  }
 }

 function initializeView(data,points,W,H){
  const fitZoom=chooseZoom(points,W,H,44),center=routeCenter(points,fitZoom);
  mapView.routeId=String(data.route_id||'');mapView.fitZoom=fitZoom;mapView.zoom=fitZoom;mapView.centerLat=center.latitude;mapView.centerLon=center.longitude;
 }
 function updateMapControls(){const z=Number(mapView.zoom),label=q('gps-route-map-zoom'),minus=q('gps-map-minus'),plus=q('gps-map-plus');if(label)label.textContent=Number.isFinite(z)?`Z${z}`:'Z--';if(minus)minus.disabled=!Number.isFinite(z)||z<=3;if(plus)plus.disabled=!Number.isFinite(z)||z>=19}

 function drawPreview(data,forceFit=false){
  ensureUi();const map=q('gps-route-map'),svg=q('gps-route-preview'),meta=q('gps-route-preview-meta'),note=q('gps-route-preview-note'),tiles=q('gps-route-map-tiles'),fallback=q('gps-route-map-fallback');if(!svg||!map)return;svg.style.transform='';svg.innerHTML='';if(tiles){tiles.style.transform='';tiles.innerHTML=''}if(fallback){fallback.classList.remove('tiles-ready');fallback.textContent='Route를 선택하면 지도를 표시합니다.'}
  const points=(Array.isArray(data?.points)?data.points:[]).filter(p=>Number.isFinite(Number(p.latitude))&&Number.isFinite(Number(p.longitude)));if(!points.length){mapView.routeId='';mapView.zoom=null;updateMapControls();if(meta)meta.textContent='경로 좌표 없음';if(note)note.textContent='선택한 Route의 경로 좌표를 표시할 수 없습니다.';return}
  const W=Math.max(280,Math.round(map.clientWidth||640)),H=Math.max(220,Math.round(map.clientHeight||320));
  if(forceFit||mapView.routeId!==String(data.route_id||'')||!Number.isFinite(Number(mapView.zoom)))initializeView(data,points,W,H);
  const zoom=Math.max(3,Math.min(19,Math.round(Number(mapView.zoom)))),centerWorld=worldPoint(mapView.centerLat,mapView.centerLon,zoom),left=centerWorld.x-W/2,top=centerWorld.y-H/2,screen=world=>[world.x-left,world.y-top],projected=points.map(p=>worldPoint(p.latitude,p.longitude,zoom)),routeScreen=projected.map(screen);
  svg.setAttribute('viewBox',`0 0 ${W} ${H}`);renderTiles(zoom,W,H,centerWorld);
  const outline=svgElement('polyline',{fill:'none',stroke:'rgba(6,10,7,.82)','stroke-width':'9','stroke-linecap':'round','stroke-linejoin':'round',points:routeScreen.map(p=>`${p[0].toFixed(1)},${p[1].toFixed(1)}`).join(' ')}),line=svgElement('polyline',{fill:'none',stroke:'#b6d59b','stroke-width':'5','stroke-linecap':'round','stroke-linejoin':'round',points:routeScreen.map(p=>`${p[0].toFixed(1)},${p[1].toFixed(1)}`).join(' ')});svg.appendChild(outline);svg.appendChild(line);
  const start=routeScreen[0],end=routeScreen[routeScreen.length-1];svg.appendChild(svgElement('circle',{cx:start[0],cy:start[1],r:8,fill:'#62bc69',stroke:'#f1fff0','stroke-width':'2'}));svg.appendChild(svgElement('circle',{cx:end[0],cy:end[1],r:8,fill:'#d2a94d',stroke:'#fff5d0','stroke-width':'2'}));
  const st=svgElement('text',{x:start[0]+11,y:start[1]-10,fill:'#e4ffe0','font-size':'12','font-weight':'700','font-family':'ui-monospace,monospace',stroke:'#0a0e0b','stroke-width':'3','paint-order':'stroke'});st.textContent='START';svg.appendChild(st);const et=svgElement('text',{x:end[0]+11,y:end[1]-10,fill:'#fff0bd','font-size':'12','font-weight':'700','font-family':'ui-monospace,monospace',stroke:'#0a0e0b','stroke-width':'3','paint-order':'stroke'});et.textContent='END';svg.appendChild(et);
  updateMapControls();
  const length=Number(data?.quality?.normalized_length_m);if(meta)meta.textContent=`${data.route_id} · ${Number(data.point_count||points.length).toLocaleString()} pts${Number.isFinite(length)?` · ${length.toFixed(1)} m`:''} · 지도 Z${zoom}`;
  const model=selectedModel(),fallbackUsed=!!data?.quality?.contains_dgps_fallback,first=points[0],last=points[points.length-1];if(note)note.textContent=`${model?`모델 ${model.model_id} · ${stageLabel(model.validation_stage)}\n`:''}정규화 기준 Route${fallbackUsed?' · DGPS fallback 포함':''}\nSTART ${Number(first.latitude).toFixed(7)}, ${Number(first.longitude).toFixed(7)}  →  END ${Number(last.latitude).toFixed(7)}, ${Number(last.longitude).toFixed(7)}\n조작: + / − 확대축소 · 드래그 이동 · FIT 전체 경로\n※ 지도 배경은 OpenStreetMap 타일입니다. 인터넷이 없으면 지도 배경 없이 GPS 경로 선은 계속 표시됩니다.\n※ 이 선은 GPS AI가 따라야 할 학습 기준 경로이며, 모델의 미래 실제 궤적 rollout은 아닙니다.`;
 }

 function currentPreview(){return previewCache.get(routeSelect.value)}
 function changeZoom(delta){const data=currentPreview();if(!data||!Number.isFinite(Number(mapView.zoom)))return;mapView.zoom=Math.max(3,Math.min(19,Math.round(Number(mapView.zoom))+delta));drawPreview(data)}
 function fitCurrentRoute(){const data=currentPreview();if(data)drawPreview(data,true)}
 function beginDrag(event){if(event.button!==undefined&&event.button!==0)return;if(event.target?.closest?.('#gps-route-map-controls')||event.target?.closest?.('#gps-route-map-attribution'))return;const data=currentPreview();if(!data||!Number.isFinite(Number(mapView.zoom)))return;const map=q('gps-route-map'),zoom=Number(mapView.zoom),centerWorld=worldPoint(mapView.centerLat,mapView.centerLon,zoom);drag={pointerId:event.pointerId,startX:event.clientX,startY:event.clientY,centerWorld};map?.setPointerCapture?.(event.pointerId);map?.classList.add('dragging');event.preventDefault()}
 function moveDrag(event){if(!drag||event.pointerId!==drag.pointerId)return;const dx=event.clientX-drag.startX,dy=event.clientY-drag.startY,tiles=q('gps-route-map-tiles'),svg=q('gps-route-preview');if(tiles)tiles.style.transform=`translate(${dx}px,${dy}px)`;if(svg)svg.style.transform=`translate(${dx}px,${dy}px)`;event.preventDefault()}
 function endDrag(event){if(!drag||event.pointerId!==drag.pointerId)return;const dx=event.clientX-drag.startX,dy=event.clientY-drag.startY,zoom=Number(mapView.zoom),newCenter=worldToLatLon(drag.centerWorld.x-dx,drag.centerWorld.y-dy,zoom),map=q('gps-route-map'),tiles=q('gps-route-map-tiles'),svg=q('gps-route-preview');mapView.centerLat=newCenter.latitude;mapView.centerLon=newCenter.longitude;if(tiles)tiles.style.transform='';if(svg)svg.style.transform='';map?.classList.remove('dragging');try{map?.releasePointerCapture?.(event.pointerId)}catch{}drag=null;const data=currentPreview();if(data)drawPreview(data)}

 async function requestPreview(routeId){
  ensureUi();if(!routeId){drawPreview(null);return}if(previewCache.has(routeId)){drawPreview(previewCache.get(routeId));return}const serial=++previewSerial,meta=q('gps-route-preview-meta');if(meta)meta.textContent='경로 불러오는 중…';try{const data=await api(`/api/v2/gps-ai/route-preview?route_id=${encodeURIComponent(routeId)}`);if(serial!==previewSerial)return;previewCache.set(routeId,data);drawPreview(data)}catch(error){if(serial!==previewSerial)return;console.error(error);const svg=q('gps-route-preview'),tiles=q('gps-route-map-tiles');if(svg)svg.innerHTML='';if(tiles)tiles.innerHTML='';if(meta)meta.textContent='경로 미리보기 실패';const note=q('gps-route-preview-note');if(note)note.textContent='Route 좌표를 불러오지 못했습니다. 라즈베리 코드가 최신인지 확인하세요.'}}

 routeSelect.addEventListener('change',()=>{draftRoute=routeSelect.value;routeDirty=true;draftModel='';modelDirty=false;mapView.routeId='';syncSelectors(draftRoute,'')});
 modelSelect.addEventListener('change',()=>{draftModel=modelSelect.value;modelDirty=true;syncLifecycle();const cached=currentPreview();if(cached)drawPreview(cached)});
 let resizeTimer=null;window.addEventListener('resize',()=>{clearTimeout(resizeTimer);resizeTimer=setTimeout(()=>{const cached=currentPreview();if(cached)drawPreview(cached)},140)});
 const previous=render;render=function(){const beforeRoute=routeDirty?draftRoute:routeSelect.value,beforeModel=modelDirty?draftModel:modelSelect.value;previous();syncSelectors(beforeRoute,beforeModel)};
 ensureUi();syncSelectors(routeSelect.value,modelSelect.value);
})();
</script>
'''.encode('utf-8')

__all__ = ["GPS_MODEL_LIFECYCLE_HMI"]
