; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\Turkish.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_TURKISH} "Yükleyici Dili"
LangString SelectInstallerLanguage  ${LANG_TURKISH} "Lütfen yükleyicinin dilini seçin"

; subtitle on license text caption
LangString LicenseSubTitleUpdate ${LANG_TURKISH} "Güncelleştir"
LangString LicenseSubTitleSetup ${LANG_TURKISH} "Ayarlar"

; installation directory text
LangString DirectoryChooseTitle ${LANG_TURKISH} "Yükleme Dizini" 
LangString DirectoryChooseUpdate ${LANG_TURKISH} "${VERSION_LONG}.(XXX) sürümüne güncelleştirme yapmak için Firestorm dizinini seçin:"
LangString DirectoryChooseSetup ${LANG_TURKISH} "Firestorm'ın yükleneceği dizini seçin:"

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_TURKISH} "'$INSTPROG' programı bulunamadı. Sessiz güncelleştirme başarılamadı."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_TURKISH} "Firestorm şimdi başlatılsın mı?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_TURKISH} "Eski sürüm kontrol ediliyor..."

; check windows version
LangString CheckWindowsVersionDP ${LANG_TURKISH} "Windows sürümü kontrol ediliyor..."
LangString CheckWindowsVersionMB ${LANG_TURKISH} "Firestorm sadece Windows Vista SP2."
LangString CheckWindowsServPackMB ${LANG_TURKISH} "It is recomended to run Firestorm on the latest service pack for your operating system.$\nThis will help with performance and stability of the program."
LangString UseLatestServPackDP ${LANG_TURKISH} "Please use Windows Update to install the latest Service Pack."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_TURKISH} "Yükleme izni kontrol ediliyor..."
LangString CheckAdministratorInstMB ${LANG_TURKISH} "'Sınırlı' bir hesap kullanıyor görünüyorsunuz.$\nFirestorm'ı yüklemek için bir 'yönetici' olmalısınız."

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_TURKISH} "Kaldırma izni kontrol ediliyor..."
LangString CheckAdministratorUnInstMB ${LANG_TURKISH} "'Sınırlı' bir hesap kullanıyor görünüyorsunuz.$\nFirestorm'ı kaldırmak için bir 'yönetici' olmalısınız."

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_TURKISH} "Firestorm ${VERSION_LONG} zaten yüklü.$\n$\nTekrar yüklemek ister misiniz?"

; checkcpuflags
LangString MissingSSE2 ${LANG_TURKISH} "Bu makinede SSE2 desteğine sahip bir CPU bulunmayabilir, SecondLife ${VERSION_LONG} çalıştırmak için bu gereklidir. Devam etmek istiyor musunuz?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_TURKISH} "Firestorm'ın kapatılması bekleniyor..."
LangString CloseSecondLifeInstMB ${LANG_TURKISH} "Firestorm zaten çalışırken kapatılamaz.$\n$\nYaptığınız işi bitirdikten sonra Firestorm'ı kapatmak ve devam etmek için Tamam seçimini yapın.$\nYüklemeyi iptal etmek için İPTAL seçimini yapın."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_TURKISH} "Firestorm'ın kapatılması bekleniyor..."
LangString CloseSecondLifeUnInstMB ${LANG_TURKISH} "Firestorm zaten çalışırken kaldırılamaz.$\n$\nYaptığınız işi bitirdikten sonra Firestorm'ı kapatmak ve devam etmek için Tamam seçimini yapın.$\nİptal etmek için İPTAL seçimini yapın."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_TURKISH} "Ağ bağlantısı kontrol ediliyor..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_TURKISH} "Belgeler ve Ayarlar klasöründeki önbellek dosyaları siliniyor?"

; delete program files
LangString DeleteProgramFilesMB ${LANG_TURKISH} "SecondLife program dizininizde hala dosyalar var.$\n$\nBunlar muhtemelen sizin oluşturduğunuz veya şuraya taşıdığınız dosyalar:$\n$INSTDIR$\n$\nBunları kaldırmak istiyor musunuz?"

; uninstall text
LangString UninstallTextMsg ${LANG_TURKISH} "Bu adımla Firestorm ${VERSION_LONG} sisteminizden kaldırılacaktır."

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_TURKISH} "Do you want to remove application registry keys?$\n$\nIt is recomended to keep registry keys if you have other versions of Firestorm installed."

; <FS:Ansariel> Optional start menu entry
LangString CreateStartMenuEntry ${LANG_TURKISH} "Create an entry in the start menu?"

LangString DeleteDocumentAndSettingsDP ${LANG_TURKISH} "Deleting files in Documents and Settings folder."
LangString UnChatlogsNoticeMB ${LANG_TURKISH} "This uninstall will NOT delete your Firestorm chat logs and other private files. If you want to do that yourself, delete the Firestorm folder within your user Application data folder."
LangString UnRemovePasswordsDP ${LANG_TURKISH} "Removing Firestorm saved passwords."
