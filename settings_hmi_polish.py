'''Final presentation polish for the settings and data management page.'''

SETTINGS_HMI_POLISH = r'''
<style id="settings-hmi-polish">
:root{
  --settings-bg:#090b0b;
  --settings-surface:#101313;
  --settings-surface-2:#151818;
  --settings-surface-3:#1a1d1d;
  --settings-line:rgba(255,255,255,.075);
  --settings-line-strong:rgba(255,255,255,.12);
  --settings-text:#e8ece9;
  --settings-muted:#8d9591;
  --settings-muted-2:#6f7773;
  --settings-accent:#86ad69;
  --settings-accent-soft:rgba(134,173,105,.10);
  --settings-warn:#d3ae66;
  --settings-bad:#d86c76;
}

html,body{background:var(--settings-bg)!important}
body{color:var(--settings-text)!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important}
header{position:sticky!important;top:0!important;background:rgba(9,11,11,.97)!important;border-bottom:1px solid var(--settings-line)!important;backdrop-filter:none!important}
header .top{height:auto!important;max-width:1450px;margin:0 auto;padding:18px 20px 12px!important;align-items:flex-start!important}
.settings-page-context{display:flex!important;flex-direction:column!important;align-items:flex-start!important;gap:11px!important}
.settings-page-context .brand{display:block!important}
.settings-page-context .brand b{font-size:22px!important;line-height:1.2!important;letter-spacing:-.02em!important;font-weight:720!important;color:#f0f2f0!important}
.settings-page-context .brand small{margin-top:6px!important;font-size:10px!important;line-height:1.45!important;color:var(--settings-muted)!important}
.settings-back-link{min-height:0!important;padding:0!important;border:0!important;background:transparent!important;border-radius:0!important;color:#aeb5b1!important;font-size:10px!important;font-weight:560!important}
.settings-back-link:hover{border:0!important;background:transparent!important;color:#fff!important}

#nav{display:grid!important;grid-template-columns:repeat(5,minmax(0,1fr))!important;gap:0!important;max-width:1410px;margin:0 auto 12px!important;padding:0!important;border:1px solid var(--settings-line)!important;border-radius:6px!important;background:#0f1212!important;overflow:hidden!important}
#nav button{position:relative!important;min-width:0!important;min-height:46px!important;padding:0 12px!important;border:0!important;border-right:1px solid var(--settings-line)!important;border-radius:0!important;background:transparent!important;color:#aab1ad!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:10px!important;font-weight:620!important;letter-spacing:0!important;box-shadow:none!important}
#nav button:last-child{border-right:0!important}
#nav button:hover:not(:disabled){background:#131616!important;color:#dce0dd!important}
#nav button.active{background:var(--settings-accent-soft)!important;color:#b3d39a!important;box-shadow:inset 0 -2px 0 var(--settings-accent)!important}
#nav button.active::before{content:"";position:absolute;left:18px;right:18px;bottom:0;height:2px;background:var(--settings-accent)}

main{max-width:1450px!important;margin:0 auto!important;padding:4px 20px 30px!important}
.grid{gap:12px!important}
.view.active>.grid{align-items:start}
.workflow-intro{padding:12px 2px 15px!important;margin:0 0 2px!important;border:0!important;border-bottom:1px solid var(--settings-line)!important;border-radius:0!important;background:transparent!important}
.workflow-intro strong{font-size:14px!important;font-weight:680!important;color:#e7ebe8!important;letter-spacing:-.01em!important}
.workflow-intro p{max-width:900px;margin-top:6px!important;color:var(--settings-muted)!important;font-size:9px!important;line-height:1.65!important}

.panel{background:var(--settings-surface)!important;border:1px solid var(--settings-line)!important;border-radius:6px!important;padding:17px 18px!important;box-shadow:none!important}
.panel:hover{border-color:var(--settings-line-strong)!important}
.panel h2,.panel h3{font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;color:#e5e9e6!important;letter-spacing:-.01em!important}
.panel h2{margin-bottom:7px!important;font-size:12px!important;font-weight:680!important}
.panel h3{font-size:11px!important;font-weight:650!important}
.sectionnote{max-width:760px!important;margin:0 0 14px!important;color:var(--settings-muted)!important;font-size:9px!important;line-height:1.6!important}

.workflow-step{padding-top:43px!important}
.workflow-step::before{top:15px!important;left:18px!important;right:18px!important;padding:0 0 8px!important;border-bottom:1px solid var(--settings-line)!important;color:#9caf91!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:8px!important;font-weight:650!important;letter-spacing:.02em!important}
.workflow-final{padding:10px 12px!important;border:0!important;border-left:2px solid rgba(134,173,105,.46)!important;border-radius:0!important;background:#0d1010!important;color:#838b87!important;font-size:9px!important;line-height:1.55!important}
.workflow-final b{color:#bac6b4!important;font-weight:650!important}

.row{gap:8px!important}
.row>*{min-width:120px}
label{color:#929a96!important;font-size:9px!important;font-weight:520!important}
input,select,textarea{min-height:38px!important;padding:8px 10px!important;border:1px solid var(--settings-line-strong)!important;border-radius:4px!important;background:var(--settings-surface-2)!important;color:#e3e7e4!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:10px!important;box-shadow:none!important;outline:none!important}
input:hover,select:hover,textarea:hover{border-color:rgba(255,255,255,.18)!important}
input:focus,select:focus,textarea:focus{border-color:rgba(134,173,105,.70)!important;box-shadow:0 0 0 2px rgba(134,173,105,.10)!important}
input::placeholder,textarea::placeholder{color:#69716d!important}
input[type="checkbox"]{min-height:auto!important;accent-color:var(--settings-accent)}

button{min-height:36px!important;padding:7px 11px!important;border:1px solid var(--settings-line-strong)!important;border-radius:4px!important;background:var(--settings-surface-3)!important;color:#d6dbd7!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:9px!important;font-weight:620!important;letter-spacing:0!important;box-shadow:none!important}
button:hover:not(:disabled){border-color:rgba(255,255,255,.20)!important;background:#202424!important;color:#fff!important}
button.primary{border-color:rgba(134,173,105,.42)!important;background:#263222!important;color:#c7dfb5!important}
button.primary:hover:not(:disabled){border-color:rgba(134,173,105,.72)!important;background:#2e3d29!important}
button.danger{border-color:rgba(216,108,118,.44)!important;background:#2a1719!important;color:#f0a4ab!important}
button.ghost{background:transparent!important}
button:disabled{opacity:.42!important}

.tablewrap{border-top:1px solid var(--settings-line)!important;border-bottom:1px solid var(--settings-line)!important}
table{font-size:9px!important}
th,td{padding:9px 8px!important;border-bottom:1px solid var(--settings-line)!important}
th{color:#7f8783!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:8px!important;font-weight:620!important;letter-spacing:.02em!important}
td{color:#c8ceca!important}
tbody tr:last-child td{border-bottom:0!important}
tbody tr:hover{background:rgba(255,255,255,.018)!important}

.user-summary{gap:1px!important;margin-top:11px!important;border:1px solid var(--settings-line)!important;border-radius:5px!important;background:var(--settings-line)!important;overflow:hidden!important}
.user-summary-card{padding:10px 11px!important;border:0!important;border-radius:0!important;background:#121515!important}
.user-summary-card span{color:#737c77!important;font-size:8px!important}
.user-summary-card strong{margin-top:4px!important;color:#dbe0dc!important;font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;font-size:10px!important;font-weight:650!important}
.user-summary-card small{color:#7f8783!important;font-size:8px!important;line-height:1.5!important}
.setup-inline{border-left-width:2px!important;border-radius:0!important;background:#0e1111!important}
.setup-overview{padding:0 0 15px!important;border:0!important;border-bottom:1px solid var(--settings-line)!important;border-radius:0!important;background:transparent!important}
.setup-overview-head{margin-bottom:10px!important}
.setup-overview-head strong{font-size:11px!important;color:#d8ddda!important}
.setup-overview-head small{font-size:8px!important;color:#747c78!important}
.setup-overview-grid{gap:1px!important;border:1px solid var(--settings-line)!important;border-radius:5px!important;background:var(--settings-line)!important;overflow:hidden!important}
.setup-overview-card{padding:10px 12px!important;background:#111414!important}
.setup-overview-card b{font-size:10px!important;font-weight:650!important}
.setup-overview-card small{font-size:8px!important;color:#737b77!important}

.connection-grid{gap:12px!important}
.connection-box{padding:14px!important;border:1px solid var(--settings-line)!important;border-radius:5px!important;background:#121515!important}
.connection-box h3{font-size:10px!important;font-weight:650!important}
.connection-box p{color:#7f8783!important;font-size:8px!important;line-height:1.55!important}
.sensorgrid{gap:1px!important;border:1px solid var(--settings-line)!important;border-radius:5px!important;background:var(--settings-line)!important;overflow:hidden!important}
.sensor{padding:11px!important;border:0!important;border-radius:0!important;background:#121515!important}
.sensor span{font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif!important;color:#777f7b!important;font-size:8px!important}
.sensor strong{font-size:10px!important}

.good{color:#9fc585!important}.warn{color:var(--settings-warn)!important}.bad{color:#e1848c!important}

@media(max-width:1050px){
  #nav{margin-left:12px!important;margin-right:12px!important}
  main{padding-left:12px!important;padding-right:12px!important}
}
@media(max-width:700px){
  header .top{padding:12px 12px 10px!important}
  .settings-page-context .brand b{font-size:18px!important}
  #nav{display:flex!important;overflow-x:auto!important;border-radius:5px!important}
  #nav button{flex:0 0 auto!important;min-width:128px!important;min-height:42px!important}
  main{padding:2px 10px 24px!important}
  .panel{padding:14px!important}
  .workflow-step::before{left:14px!important;right:14px!important}
}
</style>
<script>
(function(){
  document.body.classList.add('settings-hmi-page');
  const nav=document.getElementById('nav');
  if(nav)nav.setAttribute('aria-label','설정 항목');
  const back=document.getElementById('settings-back');
  if(back)back.setAttribute('aria-label','차량 대시보드로 돌아가기');

  const viewNames={
    data:'주행 데이터',
    gps:'GPS 자율주행',
    local:'지도 자율주행',
    hardware:'장치 점검',
    system:'연결·전원'
  };
  document.querySelectorAll('#nav button').forEach(button=>{
    const name=viewNames[button.dataset.view];
    if(!name)return;
    button.textContent=name;
    button.setAttribute('aria-label',name);
  });

  const markActive=()=>{
    document.querySelectorAll('#nav button').forEach(button=>button.setAttribute('aria-current',button.classList.contains('active')?'page':'false'));
  };
  markActive();
  nav?.addEventListener('click',()=>requestAnimationFrame(markActive));
})();
</script>
'''.encode('utf-8')

__all__ = ["SETTINGS_HMI_POLISH"]
