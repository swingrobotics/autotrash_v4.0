#ifndef AppVersion
  #define AppVersion "0.4.0"
#endif

#define AppName "SWING Compute Worker"
#define ManagerExeName "SWING-Compute-Manager.exe"
#define WorkerExeName "SWING-Compute-Worker.exe"

[Setup]
AppId={{4C82FA97-1A6A-4C77-8B73-55E643A25A88}
AppName={#AppName}
AppVersion={#AppVersion}
AppPublisher=SWING Robotics
DefaultDirName={autopf}\SWING Robotics\Compute Worker
DefaultGroupName=SWING Robotics
UninstallDisplayIcon={app}\{#ManagerExeName}
OutputDir=..\..\installer-dist
OutputBaseFilename=SWING-Compute-Worker-Setup
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
PrivilegesRequired=admin
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
DisableProgramGroupPage=yes
CloseApplications=yes
RestartApplications=no

[Files]
Source: "..\..\dist\SWING-Compute-Worker\*"; DestDir: "{app}\worker"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "..\..\dist\SWING-Compute-Manager.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "stop_swing_worker.ps1"; DestDir: "{app}\tools"; Flags: ignoreversion

[InstallDelete]
Type: files; Name: "{userstartup}\SWING Compute Worker.lnk"
Type: files; Name: "{userstartup}\SWING Compute Worker Dev.lnk"

[Icons]
Name: "{group}\SWING Compute Worker"; Filename: "{app}\{#ManagerExeName}"
Name: "{group}\SWING Compute Worker 제거"; Filename: "{uninstallexe}"
Name: "{autodesktop}\SWING Compute Worker"; Filename: "{app}\{#ManagerExeName}"

[Run]
Filename: "{sys}\WindowsPowerShell\v1.0\powershell.exe"; Parameters: "-NoProfile -ExecutionPolicy Bypass -File ""{app}\tools\stop_swing_worker.ps1"""; Flags: runhidden waituntilterminated; StatusMsg: "기존 SWING Worker를 정리하는 중..."
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall delete rule name=""SWING Compute Worker Dev"""; Flags: runhidden waituntilterminated; StatusMsg: "개발용 방화벽 규칙을 정리하는 중..."
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall delete rule name=""SWING Compute Worker"""; Flags: runhidden waituntilterminated; StatusMsg: "기존 방화벽 규칙을 정리하는 중..."
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall add rule name=""SWING Compute Worker"" dir=in action=allow program=""{app}\worker\{#WorkerExeName}"" enable=yes profile=private protocol=TCP localport=8765"; Flags: runhidden waituntilterminated; StatusMsg: "차량 전용 LAN 방화벽 규칙을 설정하는 중..."
Filename: "{app}\{#ManagerExeName}"; Flags: nowait postinstall skipifsilent runasoriginaluser; Description: "SWING Compute Worker 앱 열기"

[UninstallRun]
Filename: "{sys}\WindowsPowerShell\v1.0\powershell.exe"; Parameters: "-NoProfile -ExecutionPolicy Bypass -File ""{app}\tools\stop_swing_worker.ps1"""; Flags: runhidden waituntilterminated
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall delete rule name=""SWING Compute Worker"""; Flags: runhidden waituntilterminated

[UninstallDelete]
Type: filesandordirs; Name: "{app}\worker"
Type: filesandordirs; Name: "{app}\tools"
