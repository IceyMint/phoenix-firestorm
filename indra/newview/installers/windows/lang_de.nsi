; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\German.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_GERMAN} "Installationssprache"
LangString SelectInstallerLanguage  ${LANG_GERMAN} "Bitte wählen Sie die Installationssprache"

; subtitle on license text caption (setup new version or update current one
LangString LicenseSubTitleUpdate ${LANG_GERMAN} " Update"
LangString LicenseSubTitleSetup ${LANG_GERMAN} " Setup"

; installation directory text
LangString DirectoryChooseTitle ${LANG_GERMAN} "Installations-Ordner"
LangString DirectoryChooseUpdate ${LANG_GERMAN} "Wählen Sie den Firestorm Ordner für dieses Update:"
LangString DirectoryChooseSetup ${LANG_GERMAN} "Pfad in dem Firestorm installiert werden soll:"

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_GERMAN} "Konnte Programm '$INSTPROG' nicht finden. Stilles Update fehlgeschlagen."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_GERMAN} "Firestorm starten?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_GERMAN} "Überprüfe alte Version ..."

; check windows version
LangString CheckWindowsVersionDP ${LANG_GERMAN} "Überprüfung der Windows Version ..."
LangString CheckWindowsVersionMB ${LANG_GERMAN} 'Firestorm unterstützt nur Windows Vista mit Service Pack 2 und höher.$\nEine Installation auf diesem Betriebssystem wird nicht unterstützt. Installation wird beendet...'
LangString CheckWindowsServPackMB ${LANG_GERMAN} "Es wird empfohlen, das aktuellste Service Pack des Betriebssystems für Firestorm zu verwenden.$\nEs ist hilftreich für Performance und Stabilität des Programms."
LangString UseLatestServPackDP ${LANG_GERMAN} "Bitte Windows Update benutzen, um das aktuellste Service Pack zu installieren."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_GERMAN} "Überprüfung der Installations-Berechtigungen ..."
LangString CheckAdministratorInstMB ${LANG_GERMAN} 'Sie besitzen ungenügende Berechtigungen.$\nSie müssen ein "administrator" sein, um Firestorm installieren zu können.'

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_GERMAN} "Überprüfung der Entfernungs-Berechtigungen ..."
LangString CheckAdministratorUnInstMB ${LANG_GERMAN} 'Sie besitzen ungenügende Berechtigungen.$\nSie müssen ein "administrator" sein, um Firestorm entfernen zu können..'

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_GERMAN} "Anscheinend ist Firestorm ${VERSION_LONG} bereits installiert.$\n$\nWürden Sie es gerne erneut installieren?"

; checkcpuflags
LangString MissingSSE2 ${LANG_GERMAN} "Dieser PC besitzt möglicherweise keinen Prozessor mit SSE2-Unterstützung, die für die Ausführung von Firestorm ${VERSION_LONG} benötigt wird. Trotzdem installieren?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_GERMAN} "Warten auf die Beendigung von Firestorm ..."
LangString CloseSecondLifeInstMB ${LANG_GERMAN} "Firestorm kann nicht installiert oder ersetzt werden, wenn es bereits läuft.$\n$\nBeenden Sie, was Sie gerade tun und klicken Sie OK, um Firestorm zu beenden.$\nKlicken Sie CANCEL, um die Installation abzubrechen."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_GERMAN} "Warten auf die Beendigung von Firestorm ..."
LangString CloseSecondLifeUnInstMB ${LANG_GERMAN} "Firestorm kann nicht entfernt werden, wenn es bereits läuft.$\n$\nBeenden Sie, was Sie gerade tun und klicken Sie OK, um Firestorm zu beenden.$\nKlicken Sie CANCEL, um abzubrechen."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_GERMAN} "Prüfe Netzwerkverbindung..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_GERMAN} "Einstellungs- und Cache-Dateien in Dokumente und Einstellungen löschen?"

; delete program files
LangString DeleteProgramFilesMB ${LANG_GERMAN} "Es existieren weiterhin Dateien in Ihrem SecondLife Programm Ordner.$\n$\nDies sind möglicherweise Dateien, die sie modifiziert oder bewegt haben:$\n$INSTDIR$\n$\nMöchten Sie diese ebenfalls löschen?"

; uninstall text
LangString UninstallTextMsg ${LANG_GERMAN} "Dies wird Firestorm ${VERSION_LONG} von Ihrem System entfernen."

; <FS:Ansariel> Optional start menu entry
LangString CreateStartMenuEntry ${LANG_GERMAN} "Eintrag im Startmenü erstellen?"

LangString DeleteDocumentAndSettingsDP ${LANG_GERMAN} 'Dateien unterhalb von "Dokumente und Einstellungen werden gelöscht.'
LangString UnChatlogsNoticeMB ${LANG_GERMAN} "Diese Deinstallation löscht NICHT die Firestorm-Chatprotokolle und andere private Dateien. Sollen diese gelöscht werden, muss das Firestorm-Verzeichnis im Anwendungsdaten-Verzeichnis manuell gelöscht werden."
LangString UnRemovePasswordsDP ${LANG_GERMAN} "Lösche gespeicherte Firestorm-Passwörter."
