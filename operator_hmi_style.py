'''Presentation-only styling for the primary vehicle operator dashboard.'''

OPERATOR_HMI_STYLE = r'''
<style id="operator-hmi-style">
:root{
  --bg:#090a0a;
  --panel:#111313;
  --panel-2:#171919;
  --line:rgba(255,255,255,.085);
  --muted:#8d9491;
  --text:#eef0ef;
  --cyan:#eef0ef;
  --green:#55b978;
  --amber:#d2a85d;
  --red:#d4545d;
  --critical:#b4232e;
}
html,body{background:var(--bg);color:var(--text)}
body{font-family:Inter,"Pretendard","Noto Sans KR",system-ui,-apple-system,sans-serif;letter-spacing:0}
body,button,input,select,textarea{font-family:Inter,"Pretendard","Noto Sans KR",system-ui,-apple-system,sans-serif}
*{text-shadow:none}

/* Header: connectivity / driving state / critical operator controls */
header{
  min-height:64px;height:64px;padding:0 20px;gap:18px;
  background:#0d0f0f;border-bottom:1px solid var(--line);
  box-shadow:none;backdrop-filter:none;
}
.brand{gap:12px;flex:0 0 auto}.brand-logo{width:72px;height:40px}.brand h1{font-size:14px;font-weight:650;letter-spacing:0;color:#f3f4f3}
.header-status{min-width:0;flex:1;display:flex;align-items:center;justify-content:flex-end;gap:8px}
.network-summary{display:flex;align-items:center;gap:14px;padding-right:16px;margin-right:4px;border-right:1px solid var(--line)}
.network-pill{min-width:auto;padding:0;border:0;border-radius:0;background:transparent;box-shadow:none}
.network-pill span{font:500 10px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#777e7b;letter-spacing:0}
.network-pill strong{margin-top:2px;font:650 11px ui-monospace,"SFMono-Regular",Consolas,monospace;color:#d8dcda}
#internet-state.good::before,#pc-ping.good::before{content:"●";margin-right:5px;color:var(--green);font-size:8px}
#internet-state.warn::before,#pc-ping.warn::before{content:"▲";margin-right:5px;color:var(--amber);font-size:8px}
#internet-state.good,#pc-ping.good{color:#d8dcda}#internet-state.warn,#pc-ping.warn{color:#d8dcda}
.header-action-button,.restart-button,.power-button,#v2-main-action,#v2-main-estop,#v2-main-reset{
  min-height:34px;padding:7px 10px;border:1px solid var(--line);border-radius:4px;
  background:#171919;color:#c8ccca;box-shadow:none;font-size:10px;font-weight:600;letter-spacing:0
}
.header-action-button:hover,.restart-button:hover{border-color:rgba(255,255,255,.16);background:#1c1f1f;color:#f1f2f1}
#ntrip-settings-open,#wifi-settings-open{border-color:var(--line)!important;background:#151717!important;color:#b8bdbb!important}
#ntrip-settings-open.active,#wifi-settings-open.active{border-color:rgba(85,185,120,.45)!important;background:#142018!important;color:#d8eee0!important}
#autonomy-open{margin-left:8px;border-color:rgba(255,255,255,.13)!important;background:#1b1d1d!important;color:#f0f1f0!important;font-weight:650!important}
#autonomy-open.v2-mode-selected{border-color:rgba(255,255,255,.13)!important;background:#1b1d1d!important;color:#f0f1f0!important}
#v2-main-runtime{min-height:34px;max-width:250px;padding:7px 10px;border:0!important;border-left:1px solid var(--line)!important;border-radius:0!important;background:transparent!important;color:#c7ccca!important;font:600 10px Inter,"Pretendard","Noto Sans KR",sans-serif!important}
#v2-main-runtime.auto{color:#dbe9df!important;background:transparent!important}
#v2-main-action{border-color:rgba(85,185,120,.30)!important;background:#142019!important;color:#c6e2d0!important}
#v2-main-action.running{border-color:rgba(210,168,93,.35)!important;background:#211c13!important;color:#e4c88f!important}
#v2-main-estop{border-color:#d0444f!important;background:var(--critical)!important;color:#fff!important;font-weight:750!important}
#v2-main-estop::before{content:"! ";font-weight:900}
#v2-main-reset{border-color:rgba(210,168,93,.4)!important;background:#211c13!important;color:#e4c88f!important}
.restart-button{border-color:var(--line);background:#171919;color:#c8ccca}
.power-button{border-color:rgba(212,84,93,.45);background:#251719;color:#e5a1a6}
.power-button:hover{border-color:#d4545d;background:#331b1e;color:#ffd9dc}

/* Main structure stays intact; surfaces become calm and functional. */
main{background:transparent;gap:12px;padding:12px 14px}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:5px;box-shadow:none}
.panel-head{height:40px;padding:0 12px;background:#121414;border-bottom:1px solid var(--line)}
.panel-title{font:650 11px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#dfe2e0;letter-spacing:0}
.camera-panel{border-color:rgba(255,255,255,.11)}
.camera-wrap{background:#050606}.camera-overlay{display:none!important}
.camera-meta,.detection-meta,.lane-meta{
  padding:5px 7px;border:1px solid rgba(255,255,255,.08);border-radius:3px;
  background:rgba(7,8,8,.78);box-shadow:none;
  font:600 9px ui-monospace,"SFMono-Regular",Consolas,monospace;color:#d8dcda
}
.camera-meta{left:10px;bottom:9px;color:#b9bfbc}.lane-meta{right:10px;top:9px}.detection-meta{right:10px;bottom:9px}

/* Flat engineering status badges: status meaning, not decoration. */
.tag{padding:3px 6px;border:0;border-radius:3px;background:#1a1c1c;color:#a7adaa;font:600 9px Inter,"Pretendard","Noto Sans KR",sans-serif}
.tag.live{border:0;background:#142019;color:#bfe0ca}.tag.live::before{content:"●";margin-right:5px;color:var(--green);font-size:8px}
.tag.warn,.section-state.warn{color:#dfbf81}.tag.warn::before,.section-state.warn::before{content:"▲";margin-right:5px;color:var(--amber);font-size:8px}
.section-state.good{color:#c8dfd0}.section-state.good::before{content:"●";margin-right:5px;color:var(--green);font-size:8px}
.section-state{font:600 9px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#a4aaa7}

/* LiDAR: one panel, grouped instrumentation, no collection of outlined cards. */
.lidar-panel{background:#101212}.lidar-view{background:#080909!important;border:0;box-shadow:none}
.lidar-summary{padding:0;gap:0;border-top:1px solid var(--line);background:#101212}
.lidar-summary-values{gap:0;background:transparent}
.lidar-summary-value{
  padding:9px 10px;border:0!important;border-right:1px solid rgba(255,255,255,.055)!important;
  border-bottom:1px solid rgba(255,255,255,.055)!important;border-radius:0!important;background:transparent!important;box-shadow:none!important
}
.lidar-summary-value:nth-child(3n){border-right:0!important}
.lidar-summary-value span,.lidar-drive-readout span{font:500 10px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#7f8783;letter-spacing:0}
.lidar-summary-value strong{margin-top:5px;color:#eef0ef!important;font:650 16px ui-monospace,"SFMono-Regular",Consolas,monospace!important}
.lidar-summary-value.coordinate strong{font-size:11px!important;color:#d9dddb!important}.lidar-summary-value small{font:500 9px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#7f8783}
.lidar-drive{padding:12px;border:0!important;border-radius:0!important;border-left:1px solid var(--line)!important;background:#0e1010!important;box-shadow:none!important}
.lidar-drive-readout strong{color:#f2f3f2!important;font:700 24px ui-monospace,"SFMono-Regular",Consolas,monospace!important}
.lidar-drive-track{width:6px;background:#242727;border-radius:2px}.lidar-drive-fill{background:#d7dbd9!important;box-shadow:none!important}.lidar-drive-fill.reverse{background:var(--amber)!important}

/* Settings drawer behaves like an instrument service panel, not a card gallery. */
.details-panel{background:#101212;border-right:1px solid var(--line);box-shadow:12px 0 28px rgba(0,0,0,.45)}
.details-panel .panel-head{background:#121414}.details-panel .device-toggle{background:#1a1c1c!important;border:1px solid var(--line)!important;color:#c9cecb!important;box-shadow:none!important}
.details-content{background:#0d0f0f;gap:16px;padding:16px}
.drawer-section{padding:0 0 16px;border:0;border-bottom:1px solid var(--line);border-radius:0;background:transparent}
.drawer-section:last-child{border-bottom:0}.drawer-section-head{margin-bottom:10px;padding:0!important;color:#d9dddb;font-size:11px;font-weight:650}
.drawer-section-head::before{display:none!important}
.system-status-grid{gap:0;border-top:1px solid rgba(255,255,255,.055)}
.system-status-item{min-height:54px;padding:9px 2px;border:0!important;border-bottom:1px solid rgba(255,255,255,.055)!important;border-radius:0!important;background:transparent!important;box-shadow:none!important}
.system-status-item:nth-child(odd){padding-right:10px}.system-status-item:nth-child(even){padding-left:10px;border-left:1px solid rgba(255,255,255,.055)!important}
.system-status-item span,.detail-value span{font:500 9px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#7f8783;letter-spacing:0}
.system-status-item strong,.detail-value strong{font:650 12px ui-monospace,"SFMono-Regular",Consolas,monospace;color:#e8ebe9}
.details-values{gap:0;border-top:1px solid rgba(255,255,255,.055)}
.detail-value{padding:9px 2px;border:0!important;border-bottom:1px solid rgba(255,255,255,.055)!important;border-radius:0!important;background:transparent!important;box-shadow:none!important}
.detail-value:nth-child(odd){padding-right:10px}.detail-value:nth-child(even){padding-left:10px;border-left:1px solid rgba(255,255,255,.055)!important}
.details-calibration{gap:8px;margin-top:10px}.calibrate-button,.panel-action-button,.drawer-refresh-button,.steering-button,.settings-button{
  border:1px solid var(--line)!important;border-radius:4px!important;background:#171919!important;color:#c5cac7!important;box-shadow:none!important;
  font-family:Inter,"Pretendard","Noto Sans KR",sans-serif!important;font-weight:600!important;letter-spacing:0!important
}
.device-list{gap:0!important}.device{min-height:44px!important;padding:8px 2px!important;border:0!important;border-bottom:1px solid rgba(255,255,255,.055)!important;border-radius:0!important;background:transparent!important;box-shadow:none!important}
.device:last-child{border-bottom:0!important}.device>span:nth-child(2){font:550 10px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#d7dad8}.device-state{font:600 9px ui-monospace,"SFMono-Regular",Consolas,monospace;color:#8d9491}
.dot{width:7px;height:7px;background:#616765;box-shadow:none}.dot.on{background:var(--green)!important;box-shadow:none!important}

/* Compass and map lose decorative glow/gradient treatment. */
.compass{border:1px solid rgba(255,255,255,.10);background:#111313!important;box-shadow:none}
.needle{background:#d9dddb!important;box-shadow:none!important}.needle::after{background:#d9dddb!important;box-shadow:none!important}
.map-panel,.map,#leaflet-map,.leaflet-container{background:#101212}.map-empty{border:1px solid rgba(255,255,255,.08);border-radius:4px;background:rgba(14,16,16,.9);box-shadow:none}

/* Forms and dialogs: plain industrial controls. */
.modal-backdrop{background:rgba(0,0,0,.78);backdrop-filter:none}
.modal-card{border:1px solid rgba(255,255,255,.11);border-radius:6px;background:#111313;box-shadow:0 18px 50px rgba(0,0,0,.45)}
.modal-close{border:1px solid var(--line);border-radius:4px;background:#171919;color:#d8dcda}
.settings-field,.steering-config-field{font:500 10px Inter,"Pretendard","Noto Sans KR",sans-serif;color:#8d9491;letter-spacing:0}
.settings-field input,.settings-field textarea,.steering-config-field input{border:1px solid var(--line);border-radius:4px;background:#171919;color:#eef0ef;font-family:ui-monospace,"SFMono-Regular",Consolas,monospace;box-shadow:none}
.settings-status,.wifi-list,.wifi-connect-panel{border:1px solid var(--line);border-radius:4px;background:#0d0f0f;box-shadow:none}
.wifi-network{border:0;border-bottom:1px solid rgba(255,255,255,.055);background:transparent}.wifi-network:hover{background:#171919}.wifi-network.selected{background:#142019;color:#d7eadf}.wifi-radio-dot{background:var(--green);box-shadow:none}

/* V2 mode chooser aligned to the same HMI language. */
#autonomy-modal .modal-card{background:#111313!important;border-color:rgba(255,255,255,.11)!important}
#v2-mode-panel .v2-intro{color:#8f9692;font-family:Inter,"Pretendard","Noto Sans KR",sans-serif}
.v2-mode-state{gap:0!important;border-top:1px solid var(--line);border-bottom:1px solid var(--line)}
.v2-mode-state div{padding:10px 12px!important;border:0!important;border-right:1px solid rgba(255,255,255,.06)!important;border-radius:0!important;background:transparent!important}
.v2-mode-state div:last-child{border-right:0!important}.v2-mode-state span,.v2-mode-state small{font-family:Inter,"Pretendard","Noto Sans KR",sans-serif!important;color:#7f8783!important}.v2-mode-state b{font-family:ui-monospace,"SFMono-Regular",Consolas,monospace!important;color:#e4e7e5!important}
.v2-mode-section-head{color:#c8cdca!important;font-family:Inter,"Pretendard","Noto Sans KR",sans-serif!important;letter-spacing:0!important}.v2-mode-section-head small{color:#777e7b!important}
.v2-mode-card{padding:12px!important;border:1px solid rgba(255,255,255,.07)!important;border-radius:5px!important;background:#151717!important;box-shadow:none!important;transition:background .12s ease,border-color .12s ease!important}
.v2-mode-card:hover{border-color:rgba(255,255,255,.13)!important;background:#181b1b!important}.v2-mode-card.selected{border-color:rgba(85,185,120,.35)!important;background:#151b17!important;box-shadow:inset 3px 0 0 var(--green)!important}
.v2-mode-card.unready{border-color:rgba(210,168,93,.22)!important}.v2-mode-card.blocked{border-color:rgba(212,84,93,.25)!important;opacity:1!important}
.v2-mode-card b,.v2-mode-card p,.v2-mode-reason,.v2-record-option,.v2-mode-card button{font-family:Inter,"Pretendard","Noto Sans KR",sans-serif!important;letter-spacing:0!important}.v2-mode-card p{color:#919894!important}.v2-mode-reason{color:#7f8783!important}.v2-mode-card.unready .v2-mode-reason{color:#c9a96c!important}.v2-mode-card.blocked .v2-mode-reason{color:#d78a91!important}
.v2-mode-badge{border:0!important;border-radius:3px!important;background:#1d2020!important;color:#aeb4b1!important;font-family:ui-monospace,"SFMono-Regular",Consolas,monospace!important;box-shadow:none!important}
.v2-mode-badge.ready,.v2-mode-badge.running{background:#142019!important;color:#bfe0ca!important}.v2-mode-badge.ready::before,.v2-mode-badge.running::before{content:"●";margin-right:4px;color:var(--green)}
.v2-mode-badge.warn,.v2-mode-badge.checking{background:#211c13!important;color:#dfbf81!important}.v2-mode-badge.warn::before,.v2-mode-badge.checking::before{content:"▲";margin-right:4px;color:var(--amber)}
.v2-mode-badge.bad{background:#251719!important;color:#e6a1a7!important}.v2-mode-badge.bad::before{content:"×";margin-right:4px;color:var(--red)}
.v2-mode-card button{border:1px solid var(--line)!important;border-radius:4px!important;background:#1b1d1d!important;color:#d9dddb!important}.v2-mode-card:hover button,.v2-mode-card.selected button{border-color:rgba(255,255,255,.15)!important;color:#f2f3f2!important}.v2-mode-card.selected button{background:#1c211d!important}
.v2-record-option{border:0!important;border-radius:3px!important;background:#111313!important;color:#aab0ad!important}.v2-mode-footer a{border:1px solid var(--line)!important;border-radius:4px!important;background:#171919!important;color:#d3d7d5!important;font-family:Inter,"Pretendard","Noto Sans KR",sans-serif!important}

@media(max-width:1180px){
  header{height:auto;min-height:64px;padding:10px 14px;align-items:flex-start}.header-status{flex-wrap:wrap}.network-summary{order:0}.network-pill{min-width:82px}
  #autonomy-open{margin-left:0}
}
@media(max-width:760px){
  header{padding:9px 10px}.brand-logo{width:60px}.header-status{gap:6px}.network-summary{display:none}
  .header-action-button,.restart-button,.power-button,#v2-main-action,#v2-main-estop,#v2-main-reset{padding:7px 8px}
  #v2-main-runtime{order:20;width:100%;max-width:none;border-left:0!important;border-top:1px solid var(--line)!important;padding-left:0}
  main{padding:8px;gap:8px}.panel{border-radius:4px}.lidar-summary{grid-template-columns:1fr}.lidar-drive{width:100%;border-left:0!important;border-top:1px solid var(--line)!important}
  .v2-mode-state{grid-template-columns:1fr!important}.v2-mode-state div{border-right:0!important;border-bottom:1px solid rgba(255,255,255,.06)!important}.v2-mode-state div:last-child{border-bottom:0!important}
}
</style>
'''.encode('utf-8')

__all__ = ["OPERATOR_HMI_STYLE"]
