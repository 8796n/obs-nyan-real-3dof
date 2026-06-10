; Inno Setup script for obs-nyan-real-3dof.
; package.ps1 stages files at ..\dist\stage\obs-nyan-real-3dof\{bin\64bit,data,...}

#ifndef AppVersion
  #define AppVersion "0.0.0-noversion"
#endif

[Setup]
AppId={{047C23C7-BA44-4823-A188-3FF57C24C65B}
AppName=obs-nyan-real-3dof (OBS plugin)
AppVersion={#AppVersion}
AppPublisher=8796n
AppPublisherURL=https://github.com/8796n/obs-nyan-real-3dof
DefaultDirName={commonappdata}\obs-studio\plugins\obs-nyan-real-3dof
DisableDirPage=yes
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
LicenseFile=..\LICENSE
OutputDir=..\dist
OutputBaseFilename=obs-nyan-real-3dof-{#AppVersion}-windows-x64-installer
UninstallDisplayName=obs-nyan-real-3dof (OBS plugin)
WizardStyle=modern
SolidCompression=yes
ShowLanguageDialog=no

[Languages]
Name: "en"; MessagesFile: "compiler:Default.isl"
Name: "ja"; MessagesFile: "compiler:Languages\Japanese.isl"

[Files]
Source: "..\dist\stage\obs-nyan-real-3dof\*"; DestDir: "{app}"; Flags: recursesubdirs ignoreversion

[Messages]
en.WelcomeLabel2=This will install the nyan Real 3DoF plugin for OBS Studio.%n%nClose OBS Studio before continuing, then restart it after install.
ja.WelcomeLabel2=OBS Studio 用の nyan Real 3DoF プラグインをインストールします。%n%n続行する前に OBS Studio を終了し、インストール後に再起動してください。
