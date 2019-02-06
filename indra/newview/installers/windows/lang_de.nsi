; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\German.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_GERMAN} "Installationssprache"
LangString SelectInstallerLanguage  ${LANG_GERMAN} "Bitte wählen Sie die Installationssprache"

; subtitle on license text caption (setup new version or update current one
LangString LicenseSubTitleUpdate ${LANG_GERMAN} " Update"
LangString LicenseSubTitleSetup ${LANG_GERMAN} " Setup"

LangString MULTIUSER_TEXT_INSTALLMODE_TITLE ${LANG_GERMAN} "Installation Mode"
LangString MULTIUSER_TEXT_INSTALLMODE_SUBTITLE ${LANG_GERMAN} "Install for all users (requires Admin) or only for the current user?"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_TOP ${LANG_GERMAN} "When you run this installer with Admin privileges, you can choose whether to install in (e.g.) c:\Program Files or the current user's AppData\Local folder."
LangString MULTIUSER_INNERTEXT_INSTALLMODE_ALLUSERS ${LANG_GERMAN} "Install for all users"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_CURRENTUSER ${LANG_GERMAN} "Install for current user only"

; installation directory text
LangString DirectoryChooseTitle ${LANG_GERMAN} "Installations-Ordner"
LangString DirectoryChooseUpdate ${LANG_GERMAN} "Wählen Sie den Second Life Ordner für dieses Update:"
LangString DirectoryChooseSetup ${LANG_GERMAN} "Pfad in dem Second Life installiert werden soll:"

LangString MUI_TEXT_DIRECTORY_TITLE ${LANG_GERMAN} "Installation Directory"
LangString MUI_TEXT_DIRECTORY_SUBTITLE ${LANG_GERMAN} "Select the directory into which to install Second Life:"

LangString MUI_TEXT_INSTALLING_TITLE ${LANG_GERMAN} "Installing Second Life..."
LangString MUI_TEXT_INSTALLING_SUBTITLE ${LANG_GERMAN} "Installing the Second Life viewer to $INSTDIR"

LangString MUI_TEXT_FINISH_TITLE ${LANG_GERMAN} "Installing Second Life"
LangString MUI_TEXT_FINISH_SUBTITLE ${LANG_GERMAN} "Installed the Second Life viewer to $INSTDIR."

LangString MUI_TEXT_ABORT_TITLE ${LANG_GERMAN} "Installation Aborted"
LangString MUI_TEXT_ABORT_SUBTITLE ${LANG_GERMAN} "Not installing the Second Life viewer to $INSTDIR."

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_GERMAN} "Konnte Programm '$INSTNAME' nicht finden. Stilles Update fehlgeschlagen."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_GERMAN} "Second Life starten?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_GERMAN} "Überprüfe alte Version ..."

; check windows version
LangString CheckWindowsVersionDP ${LANG_GERMAN} "Überprüfung der Windows Version ..."
LangString CheckWindowsVersionMB ${LANG_GERMAN} 'Second Life unterstützt nur Windows Vista.$\n$\nDer Versuch es auf Windows $R0 zu installieren, könnte zu unvorhersehbaren Abstürzen und Datenverlust führen.$\n$\nTrotzdem installieren?'
LangString CheckWindowsServPackMB ${LANG_GERMAN} "Wir empfehlen, das neueste Service Pack für Ihr Betriebssystem zu installieren, um Second Life auszuführen.$\nDies unterstützt die Leistung und Stabilität des Programms."
LangString UseLatestServPackDP ${LANG_GERMAN} "Bitte verwenden Sie Windows Update, um das neueste Service Pack zu installieren."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_GERMAN} "Überprüfung der Installations-Berechtigungen ..."
LangString CheckAdministratorInstMB ${LANG_GERMAN} 'Sie besitzen ungenügende Berechtigungen.$\nSie müssen ein "administrator" sein, um Second Life installieren zu können.'

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_GERMAN} "Überprüfung der Entfernungs-Berechtigungen ..."
LangString CheckAdministratorUnInstMB ${LANG_GERMAN} 'Sie besitzen ungenügende Berechtigungen.$\nSie müssen ein "administrator" sein, um Second Life entfernen zu können..'

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_GERMAN} "Anscheinend ist Second Life ${VERSION_LONG} bereits installiert.$\n$\nWürden Sie es gerne erneut installieren?"

; checkcpuflags
LangString MissingSSE2 ${LANG_GERMAN} "Dieses Gerät verfügt möglicherweise nicht über eine CPU mit SSE2-Unterstützung, die für Second Life ${VERSION_LONG} benötigt wird. Möchten Sie fortfahren?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_GERMAN} "Warten auf die Beendigung von Second Life ..."
LangString CloseSecondLifeInstMB ${LANG_GERMAN} "Second Life kann nicht installiert oder ersetzt werden, wenn es bereits läuft.$\n$\nBeenden Sie, was Sie gerade tun und klicken Sie OK, um Second Life zu beenden.$\nKlicken Sie CANCEL, um die Installation abzubrechen."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_GERMAN} "Warten auf die Beendigung von Second Life ..."
LangString CloseSecondLifeUnInstMB ${LANG_GERMAN} "Second Life kann nicht entfernt werden, wenn es bereits läuft.$\n$\nBeenden Sie, was Sie gerade tun und klicken Sie OK, um Second Life zu beenden.$\nKlicken Sie CANCEL, um abzubrechen."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_GERMAN} "Prüfe Netzwerkverbindung..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_GERMAN} "Möchten Sie alle anderen zu Second Life gehörigen Dateien ebenfalls ENTFERNEN?$\n$\nWir empfehlen, die Einstellungen und Cache-Dateien zu behalten, wenn Sie andere Versionen von Second Life installiert haben oder eine Deinstallation durchführen, um Second Life auf eine neuere Version zu aktualisieren."

; delete program files
LangString DeleteProgramFilesMB ${LANG_GERMAN} "Es existieren weiterhin Dateien in Ihrem SecondLife Programm Ordner.$\n$\nDies sind möglicherweise Dateien, die sie modifiziert oder bewegt haben:$\n$INSTDIR$\n$\nMöchten Sie diese ebenfalls löschen?"

; uninstall text
LangString UninstallTextMsg ${LANG_GERMAN} "Dies wird Second Life ${VERSION_LONG} von Ihrem System entfernen."

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_GERMAN} "Möchten Sie die Registrierungsschlüssel der Anwendung entfernen?$\n$\nWir empfehlen, die Registrierungsschlüssel zu behalten, wenn Sie andere Versionen von Second Life installiert haben."
