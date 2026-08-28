'''User-facing AUTO_AI model selection and AUTO fallback environment guidance.'''

AI_MODEL_SELECTION_HMI = r'''
<style id="ai-model-selection-hmi-style">
#ai-model-empty-note{margin-top:10px;padding:10px 12px;border-left:3px solid #806a3d;background:#15140f;color:#bdb49d;font-size:9px;line-height:1.55}
#ai-model-empty-note strong{display:block;margin-bottom:3px;color:#dfca91;font-size:10px}
#auto-ai-environment-help{margin-top:10px;padding:10px 12px;background:#111414;border:1px solid rgba(255,255,255,.07);border-radius:4px;color:#8f9893;font-size:9px;line-height:1.55}
#auto-ai-environment-help strong{color:#c6ccc8;font-weight:650}
#auto-ai-environment-available{display:block;margin-top:5px;color:#a6aea9}
#ai-model:disabled,#env-tags:disabled{opacity:.62;cursor:not-allowed}
</style>
<script>
(function(){
 const q=id=>document.getElementById(id);
 const select=q('ai-model');
 if(!select)return;
 let draftModel='';
 let modelDirty=false;

 const modelPanel=select.closest('.panel');
 let empty=q('ai-model-empty-note');
 if(!empty&&modelPanel){
   empty=document.createElement('div');
   empty.id='ai-model-empty-note';
   empty.hidden=true;
   empty.innerHTML='<strong>설치된 AI 모델이 없습니다.</strong><span>학습 데이터를 만드는 것만으로 주행 모델이 생성되지는 않습니다. 학습·검증·설치까지 완료된 AUTO_AI 모델이 생기면 이 목록에 표시됩니다.</span>';
   const summary=q('user-ai-summary');
   (summary||select.parentElement)?.insertAdjacentElement('afterend',empty);
 }

 const env=q('env-tags');
 const envPanel=env?.closest('.panel');
 if(envPanel){
   envPanel.dataset.workflowStep='선택 · AUTO 모드 AI 전환 조건';
   const title=envPanel.querySelector('h2');
   if(title)title.textContent='AUTO 모드 AI 전환 조건';
   const note=envPanel.querySelector('.sectionnote');
   if(note)note.textContent='AUTO 모드는 GPS 자율주행과 지도 자율주행을 먼저 확인한 뒤, 현재 환경과 일치하는 검증된 AI 모델이 있을 때만 AI 자율주행을 자동 선택합니다.';
   let help=q('auto-ai-environment-help');
   if(!help){
     help=document.createElement('div');
     help.id='auto-ai-environment-help';
     help.innerHTML='<strong>언제 사용하는 설정인가요?</strong><span>메인 화면에서 “자동 자율주행(AUTO)”을 사용할 때만 적용됩니다. AI 자율주행을 직접 선택할 때는 이 값이 필요하지 않습니다.</span><span id="auto-ai-environment-available"></span>';
     envPanel.appendChild(help);
   }
   const save=q('env-save');if(save)save.textContent='AUTO 환경 저장';
   env.placeholder='예: indoor, warehouse';
 }

 select.addEventListener('change',()=>{
   draftModel=select.value;
   modelDirty=true;
 });

 const previousRender=render;
 render=function(){
   const before=modelDirty?draftModel:select.value;
   previousRender();
   const models=(S?.ai?.models||[]).filter(model=>(model.policy_type||'AUTO_AI')==='AUTO_AI');
   const ids=new Set(models.map(model=>String(model.model_id||'')));
   const serverSelected=String(S?.ai?.selected_model_id||'');
   const use=q('ai-select');
   const lifecycle=q('setup-lifecycle');

   if(!models.length){
     select.innerHTML='<option value="">설치된 AI 모델 없음</option>';
     select.disabled=true;
     if(use){use.disabled=true;use.textContent='사용할 모델 없음'}
     if(empty)empty.hidden=false;
     if(lifecycle)lifecycle.hidden=true;
   }else{
     select.disabled=false;
     if(modelDirty&&draftModel&&ids.has(draftModel))select.value=draftModel;
     else if(before&&ids.has(before))select.value=before;
     if(serverSelected&&draftModel===serverSelected){modelDirty=false;draftModel=''}
     if(use){use.disabled=!select.value||!!S?.ai?.controller?.active;use.textContent=serverSelected===select.value&&serverSelected?'현재 모델 사용 중':'이 모델 사용'}
     if(empty)empty.hidden=true;
     if(lifecycle)lifecycle.hidden=false;
   }

   if(env){
     const autoModels=models.filter(model=>model.validation_stage==='AUTO_ALLOWED');
     const tags=[...new Set(autoModels.flatMap(model=>Array.isArray(model.environments)?model.environments:[]).map(tag=>String(tag).trim()).filter(Boolean))].sort();
     const save=q('env-save');
     const available=q('auto-ai-environment-available');
     if(!autoModels.length||!tags.length){
       env.disabled=true;
       if(save)save.disabled=true;
       if(available)available.textContent='현재 AUTO 모드의 AI 전환에 사용할 수 있는 환경 정보가 등록된 모델이 없습니다.';
     }else{
       env.disabled=false;
       if(save)save.disabled=false;
       if(available)available.textContent=`사용 가능한 환경: ${tags.join(', ')}`;
       env.placeholder=`예: ${tags.slice(0,3).join(', ')}`;
     }
   }
 };
})();
</script>
'''.encode('utf-8')

__all__ = ["AI_MODEL_SELECTION_HMI"]
