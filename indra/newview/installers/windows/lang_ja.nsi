; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\Japanese.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_JAPANESE} "インストーラの言語"
LangString SelectInstallerLanguage  ${LANG_JAPANESE} "インストーラの言語を選択してください"

; subtitle on license text caption
LangString LicenseSubTitleUpdate ${LANG_JAPANESE} " アップデート" 
LangString LicenseSubTitleSetup ${LANG_JAPANESE} " セットアップ" 

LangString MULTIUSER_TEXT_INSTALLMODE_TITLE ${LANG_JAPANESE} "Installation Mode"
LangString MULTIUSER_TEXT_INSTALLMODE_SUBTITLE ${LANG_JAPANESE} "Install for all users (requires Admin) or only for the current user?"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_TOP ${LANG_JAPANESE} "When you run this installer with Admin privileges, you can choose whether to install in (e.g.) c:\Program Files or the current user's AppData\Local folder."
LangString MULTIUSER_INNERTEXT_INSTALLMODE_ALLUSERS ${LANG_JAPANESE} "Install for all users"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_CURRENTUSER ${LANG_JAPANESE} "Install for current user only"

; installation directory text
LangString DirectoryChooseTitle ${LANG_JAPANESE} "インストール・ディレクトリ" 
LangString DirectoryChooseUpdate ${LANG_JAPANESE} "アップデートするセカンドライフのディレクトリを選択してください。:" 
LangString DirectoryChooseSetup ${LANG_JAPANESE} "セカンドライフをインストールするディレクトリを選択してください。: " 

LangString MUI_TEXT_DIRECTORY_TITLE ${LANG_JAPANESE} "Installation Directory"
LangString MUI_TEXT_DIRECTORY_SUBTITLE ${LANG_JAPANESE} "Select the directory into which to install Second Life:"

LangString MUI_TEXT_INSTALLING_TITLE ${LANG_JAPANESE} "Installing Second Life..."
LangString MUI_TEXT_INSTALLING_SUBTITLE ${LANG_JAPANESE} "Installing the Second Life viewer to $INSTDIR"

LangString MUI_TEXT_FINISH_TITLE ${LANG_JAPANESE} "Installing Second Life"
LangString MUI_TEXT_FINISH_SUBTITLE ${LANG_JAPANESE} "Installed the Second Life viewer to $INSTDIR."

LangString MUI_TEXT_ABORT_TITLE ${LANG_JAPANESE} "Installation Aborted"
LangString MUI_TEXT_ABORT_SUBTITLE ${LANG_JAPANESE} "Not installing the Second Life viewer to $INSTDIR."

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_JAPANESE} "プログラム名'$INSTNAME'が見つかりません。サイレント・アップデートに失敗しました。" 

; installation success dialog
LangString InstSuccesssQuestion ${LANG_JAPANESE} "直ちにセカンドライフを開始しますか？ " 

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_JAPANESE} "古いバージョン情報をチェック中です…" 

; check windows version
LangString CheckWindowsVersionDP ${LANG_JAPANESE} "ウィンドウズのバージョン情報をチェック中です..." 
LangString CheckWindowsVersionMB ${LANG_JAPANESE} "Second Life はWindows Vistaのみをサポートしています。Windows $R0をインストールする事は、データの消失やクラッシュの原因になる可能性があります。インストールを続けますか？" 
LangString CheckWindowsServPackMB ${LANG_JAPANESE} "オペレーティングシステムの最新のサービスパックで Second Life を実行することをお勧めします。$\nそうすることで、プログラムのパフォーマンスと安定性が向上します。"
LangString UseLatestServPackDP ${LANG_JAPANESE} "最新のサービスパックのインストールには、Windows Update をご利用ください。"

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_JAPANESE} "インストールのための権限をチェック中です..." 
LangString CheckAdministratorInstMB ${LANG_JAPANESE} "セカンドライフをインストールするには管理者権限が必要です。"

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_JAPANESE} "アンインストールのための権限をチェック中です..." 
LangString CheckAdministratorUnInstMB ${LANG_JAPANESE} "セカンドライフをアンインストールするには管理者権限が必要です。" 

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_JAPANESE} "セカンドライフ${VERSION_LONG} はインストール済みです。再度インストールしますか？ " 

; checkcpuflags
LangString MissingSSE2 ${LANG_JAPANESE} "このコンピュータには、Second Life ${VERSION_LONG} の実行に必要な SSE2 対応の CPU が搭載されていない可能性があります。続行しますか？"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_JAPANESE} "セカンドライフを終了中です..." 
LangString CloseSecondLifeInstMB ${LANG_JAPANESE} "セカンドライフの起動中にインストールは出来ません。直ちにセカンドライフを終了してインストールを開始する場合はOKボタンを押してください。CANCELを押すと中止します。"

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_JAPANESE} "セカンドライフを終了中です..." 
LangString CloseSecondLifeUnInstMB ${LANG_JAPANESE} "セカンドライフの起動中にアンインストールは出来ません。直ちにセカンドライフを終了してアンインストールを開始する場合はOKボタンを押してください。CANCELを押すと中止します。" 

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_JAPANESE} "ネットワークの接続を確認中..." 

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_JAPANESE} "Second Life に関連する他のすべてのファイルも削除しますか？$\n$\n別バージョンの Second Life がインストールされている場合、または最新バージョンにアップグレードするためにアンインストールする場合には、設定およびキャッシュファイルを保存しておくことをお勧めします。"

; delete program files
LangString DeleteProgramFilesMB ${LANG_JAPANESE} "セカンドライフのディレクトリには、まだファイルが残されています。$\n$INSTDIR$\nにあなたが作成、または移動させたファイルがある可能性があります。全て削除しますか？ " 

; uninstall text
LangString UninstallTextMsg ${LANG_JAPANESE} "セカンドライフ${VERSION_LONG}をアンインストールします。"

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_JAPANESE} "アプリケーションレジストリキーを削除しますか？$\n$\n別バージョンの Second Life がインストールされている場合には、レジストリキーを保存しておくことをお勧めします。"
