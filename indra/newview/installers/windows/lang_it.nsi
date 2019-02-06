; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\Italian.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_ITALIAN} "Linguaggio del programma di installazione"
LangString SelectInstallerLanguage  ${LANG_ITALIAN} "Scegliere per favore il linguaggio del programma di installazione"

; subtitle on license text caption
LangString LicenseSubTitleUpdate ${LANG_ITALIAN} " Update"
LangString LicenseSubTitleSetup ${LANG_ITALIAN} " Setup"

LangString MULTIUSER_TEXT_INSTALLMODE_TITLE ${LANG_ITALIAN} "Installation Mode"
LangString MULTIUSER_TEXT_INSTALLMODE_SUBTITLE ${LANG_ITALIAN} "Install for all users (requires Admin) or only for the current user?"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_TOP ${LANG_ITALIAN} "When you run this installer with Admin privileges, you can choose whether to install in (e.g.) c:\Program Files or the current user's AppData\Local folder."
LangString MULTIUSER_INNERTEXT_INSTALLMODE_ALLUSERS ${LANG_ITALIAN} "Install for all users"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_CURRENTUSER ${LANG_ITALIAN} "Install for current user only"

; installation directory text
LangString DirectoryChooseTitle ${LANG_ITALIAN} "Directory di installazione" 
LangString DirectoryChooseUpdate ${LANG_ITALIAN} "Scegli la directory di Second Life per l’update alla versione ${VERSION_LONG}.(XXX):"
LangString DirectoryChooseSetup ${LANG_ITALIAN} "Scegli la directory dove installare Second Life:"

LangString MUI_TEXT_DIRECTORY_TITLE ${LANG_ITALIAN} "Installation Directory"
LangString MUI_TEXT_DIRECTORY_SUBTITLE ${LANG_ITALIAN} "Select the directory into which to install Second Life:"

LangString MUI_TEXT_INSTALLING_TITLE ${LANG_ITALIAN} "Installing Second Life..."
LangString MUI_TEXT_INSTALLING_SUBTITLE ${LANG_ITALIAN} "Installing the Second Life viewer to $INSTDIR"

LangString MUI_TEXT_FINISH_TITLE ${LANG_ITALIAN} "Installing Second Life"
LangString MUI_TEXT_FINISH_SUBTITLE ${LANG_ITALIAN} "Installed the Second Life viewer to $INSTDIR."

LangString MUI_TEXT_ABORT_TITLE ${LANG_ITALIAN} "Installation Aborted"
LangString MUI_TEXT_ABORT_SUBTITLE ${LANG_ITALIAN} "Not installing the Second Life viewer to $INSTDIR."

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_ITALIAN} "Non riesco a trovare il programma '$INSTNAME'. Silent Update fallito."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_ITALIAN} "Avvia ora Second Life?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_ITALIAN} "Controllo delle precedenti versioni…"

; check windows version
LangString CheckWindowsVersionDP ${LANG_ITALIAN} "Controllo della versione di Windows…"
LangString CheckWindowsVersionMB ${LANG_ITALIAN} 'Second Life supporta solo Windows Vista.$\n$\nTentare l’installazione su Windows $R0 può provocare blocchi di sistema e perdita di dati.$\n$\n'
LangString CheckWindowsServPackMB ${LANG_ITALIAN} "Si consiglia di avviare Second Life utilizzando il service pack più recente disponibile per il vostro sistema operativo. $\n Così facendo, il programma sarà più stabile e performante."
LangString UseLatestServPackDP ${LANG_ITALIAN} "Utilizza Windows Update per installare il Service Pak più recente."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_ITALIAN} "Controllo del permesso di installazione…"
LangString CheckAdministratorInstMB ${LANG_ITALIAN} 'Stai utilizzando un account “limitato”.$\nSolo un “amministratore” può installare Second Life.'

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_ITALIAN} "Controllo del permesso di installazione…"
LangString CheckAdministratorUnInstMB ${LANG_ITALIAN} 'Stai utilizzando un account “limitato”.$\nSolo un “amministratore” può installare Second Life.'

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_ITALIAN} "Second Life ${VERSION_LONG} è stato sia già installato.$\n$\nVuoi ripetere l’installazione?"

; checkcpuflags
LangString MissingSSE2 ${LANG_ITALIAN} "Questo computer potrebbe non avere un CPU con supporto SSE2, necessario per avviare Second Life ${VERSION_LONG}. Vuoi continuare?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_ITALIAN} "In attesa che Second Life chiuda…"
LangString CloseSecondLifeInstMB ${LANG_ITALIAN} "Non è possibile installare Second Life se è già in funzione..$\n$\nTermina le operazioni in corso e scegli OK per chiudere Second Life e continuare.$\nScegli CANCELLA per annullare l’installazione."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_ITALIAN} "In attesa della chiusura di Second Life…"
LangString CloseSecondLifeUnInstMB ${LANG_ITALIAN} "Non è possibile installare Second Life se è già in funzione.$\n$\nTermina le operazioni in corso e scegli OK per chiudere Second Life e continuare.$\nScegli CANCELLA per annullare."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_ITALIAN} "Verifica connessione di rete in corso..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_ITALIAN} "Vuoi inoltre RIMUOVERE tutti gli altri file connessi a Second Life? $\n$\n Ti consigliamo di conservare le impostazioni e i file della cache se hai un'altra versione di Second Life già installata o se stai disinstallando Second Life per aggiornarlo a una versione più  recente."

; delete program files
LangString DeleteProgramFilesMB ${LANG_ITALIAN} "Sono ancora presenti dei file nella directory programmi di Second Life.$\n$\nPotrebbe trattarsi di file creati o trasferiti in:$\n$INSTDIR$\n$\nVuoi cancellarli?"

; uninstall text
LangString UninstallTextMsg ${LANG_ITALIAN} "Così facendo Second Life verrà disinstallato ${VERSION_LONG} dal tuo sistema."

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_ITALIAN} "Vuoi rimuovere le chiavi di registro dell'applicazione?$\n$\n Ti consigliamo di conservare le chiavi di registro se hai un'altra versione di Second Life installata."
