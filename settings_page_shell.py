'''Settings-page shell for the advanced vehicle configuration UI.'''

# Importing the terminal runtime installs its private-LAN API and injects the
# launcher into the primary vehicle dashboard only. The settings page itself
# intentionally does not render a terminal control.
import dashboard_terminal_hmi as _dashboard_terminal_hmi


SETTINGS_PAGE_SHELL = r'''
<style id="settings-page-shell-style">
.settings-back-link{display:inline-flex;align-items:center;justify-content:center;min-height:34px;padding:6px 10px;border:1px solid rgba(255,255,255,.10);border-radius:5px;background:#141717;color:#d9ddda;text-decoration:none;font-size:10px;font-weight:650;white-space:nowrap}
.settings-back-link:hover{border-color:rgba(255,255,255,.20);background:#191c1c;color:#fff}
.settings-page-context{display:flex;align-items:center;gap:10px;min-width:0}
.settings-page-context .brand{min-width:0}.settings-page-context .brand img{display:none!important}.settings-page-context .brand b{font-size:13px;letter-spacing:0;font-weight:700}.settings-page-context .brand small{font-size:9px;letter-spacing:0}
#nav{padding-left:16px;padding-right:16px;gap:6px}#nav button{border-radius:5px;font-weight:650;letter-spacing:0}
@media(max-width:650px){.top{height:auto;min-height:58px;padding:8px 10px!important;align-items:flex-start}.settings-page-context{gap:8px}.settings-back-link{min-height:32px;padding:5px 8px}.settings-page-context .brand small{display:block}.settings-page-context .brand b{font-size:12px}#nav{padding-left:10px;padding-right:10px}}
</style>
<script>
(function(){
 document.title='SWING Rover · 설정 및 데이터 관리';
 const top=document.querySelector('header .top');
 const brand=document.querySelector('header .brand');
 if(top&&brand){
   let context=top.querySelector('.settings-page-context');
   if(!context){
     context=document.createElement('div');
     context.className='settings-page-context';
     brand.parentNode.insertBefore(context,brand);
     const back=document.createElement('a');
     back.id='settings-back';
     back.className='settings-back-link';
     back.href='/';
     back.textContent='← 차량 대시보드';
     context.appendChild(back);
     context.appendChild(brand);
   }
 }
 const title=document.querySelector('.brand b');if(title)title.textContent='설정 및 데이터 관리';
 const subtitle=document.querySelector('.brand small');if(subtitle)subtitle.textContent='차량 대시보드의 세부 설정';
 const labels={data:'주행 데이터',gps:'GPS 자율주행',local:'지도 자율주행',hardware:'장치 점검',system:'연결·전원'};
 document.querySelectorAll('#nav button').forEach(button=>{const label=labels[button.dataset.view];if(label)button.textContent=label});
})();
</script>
'''.encode('utf-8')

__all__ = ["SETTINGS_PAGE_SHELL"]
