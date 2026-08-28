'''Windows-native SSH terminal launcher for the primary vehicle dashboard.'''

import server_v2_release as release


DASHBOARD_TERMINAL_HMI = r'''
<style id="dashboard-terminal-hmi-style">
#swing-terminal-quick{display:flex;align-items:center;gap:6px}
#swing-terminal-open{min-width:88px;min-height:34px;padding:6px 10px;border:1px solid #53645a;border-radius:8px;background:#172019;color:#e7eee8;font:750 10px ui-monospace,monospace;cursor:pointer}
#swing-terminal-open:hover{border-color:#8cad91;background:#1d2920}
#swing-terminal-kind{display:inline-flex;align-items:center;min-height:22px;padding:2px 6px;border:1px solid rgba(255,255,255,.09);border-radius:999px;color:#8e9991;background:#111713;font:700 8px ui-monospace,monospace;white-space:nowrap}
@media(max-width:650px){#swing-terminal-quick{gap:4px}#swing-terminal-open{min-width:72px;min-height:32px;padding:5px 8px;font-size:9px}#swing-terminal-kind{display:none}}
</style>
<script>
(function(){
 if(document.getElementById('swing-terminal-quick'))return;
 const mount=document.querySelector('header .header-status');
 if(!mount)return;

 const slot=document.createElement('div');
 slot.id='swing-terminal-quick';
 slot.innerHTML='<span id="swing-terminal-kind" title="Windows Terminal 또는 CMD에서 SSH 연결">WIN SSH</span><button id="swing-terminal-open" class="header-action-button" type="button" title="Windows Terminal에서 PI SSH 연결">PI 터미널 ↗</button>';
 mount.insertBefore(slot,mount.firstChild);

 const button=document.getElementById('swing-terminal-open');
 button.addEventListener('click',()=>{
   if(!confirm('차량이 완전히 정지되어 있고 안전하게 터미널 작업을 할 수 있는 상태입니까?'))return;
   const host=String(window.location.hostname||'192.168.137.2').trim();
   if(!/^[A-Za-z0-9.:-]+$/.test(host)){
     alert('PI 주소를 확인할 수 없습니다.');
     return;
   }

   const target=`swing-terminal://gnss@${host}/`;
   const link=document.createElement('a');
   link.href=target;
   link.style.display='none';
   document.body.appendChild(link);
   link.click();
   link.remove();

   button.textContent='SSH 여는 중…';
   setTimeout(()=>{button.textContent='PI 터미널 ↗'},1200);
 });
})();
</script>
'''.encode('utf-8')


def install_dashboard_terminal_ui():
    page = release.full.legacy.INDEX_HTML
    if not isinstance(page, (bytes, bytearray)):
        raise TypeError('Legacy INDEX_HTML must be bytes before terminal injection')
    if b'id="dashboard-terminal-hmi-style"' in page:
        return True
    release.full.legacy.INDEX_HTML = page.replace(
        b"</body>",
        DASHBOARD_TERMINAL_HMI + b"</body>",
        1,
    )
    return True


install_dashboard_terminal_ui()


__all__ = ["DASHBOARD_TERMINAL_HMI", "install_dashboard_terminal_ui"]
