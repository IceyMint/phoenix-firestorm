; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\Spanish.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_SPANISH} "Lenguaje del Instalador"
LangString SelectInstallerLanguage  ${LANG_SPANISH} "Por favor seleccione el idioma de su instalador"

; subtitle on license text caption
LangString LicenseSubTitleUpdate ${LANG_SPANISH} " Actualizar"
LangString LicenseSubTitleSetup ${LANG_SPANISH} " Instalar"

; installation directory text
LangString DirectoryChooseTitle ${LANG_SPANISH} "Directorio de instalación" 
LangString DirectoryChooseUpdate ${LANG_SPANISH} "Seleccione el directorio de Second Life para actualizar el programa a la versión ${VERSION_LONG}.(XXX):"
LangString DirectoryChooseSetup ${LANG_SPANISH} "Seleccione el directorio en el que instalar Second Life:"

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_SPANISH} "No se pudo encontrar el programa '$INSTNAME'. Error al realizar la actualización desatendida."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_SPANISH} "¿Iniciar Second Life ahora?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_SPANISH} "Comprobando la versión anterior..."

; check windows version
LangString CheckWindowsVersionDP ${LANG_SPANISH} "Comprobando la versión de Windows..."
LangString CheckWindowsVersionMB ${LANG_SPANISH} 'Second Life sólo se puede ejecutar en Windows Vista.$\n$\nSi intenta instalar el programa en Windows $R0, es posible que el sistema se bloquee y se pierdan datos.$\n$\n'
LangString CheckWindowsServPackMB ${LANG_SPANISH} "Se recomienda utilizar Second Life con el último paquete de servicios para tu sistema operativo.$\nEsto ayudará al funcionamiento y la estabilidad del programa."
LangString UseLatestServPackDP ${LANG_SPANISH} "Por favor utiliza Windows Update para instalar el último paquete de servicios."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_SPANISH} "Comprobando los permisos para la instalación..."
LangString CheckAdministratorInstMB ${LANG_SPANISH} 'Parece que está usando una cuenta "limitada".$\nDebe iniciar sesión como "administrador" para instalar Second Life.'

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_SPANISH} "Comprobando los permisos para la desinstalación..."
LangString CheckAdministratorUnInstMB ${LANG_SPANISH} 'Parece que está usando una cuenta "limitada".$\nDebe iniciar sesión como "administrador" para desinstalar Second Life.'

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_SPANISH} "Parece que Second Life ${VERSION_LONG} ya está instalado.$\n$\n¿Desea volver a instalarlo?"

; checkcpuflags
LangString MissingSSE2 ${LANG_SPANISH} "Puede ser que esta máquina no tenga una CPU compatible con SSE2, esto es necesario para utilizar SecondLife ${VERSION_LONG}. ¿Deseas continuar?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_SPANISH} "Esperando que Second Life se cierre..."
LangString CloseSecondLifeInstMB ${LANG_SPANISH} "Second Life no se puede instalar mientras esté en ejecución.$\n$\nTermine lo que esté haciendo y seleccione Aceptar (OK) para cerrar Second Life y continuar el proceso.$\nSeleccione Cancelar (CANCEL) para cancelar la instalación."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_SPANISH} "Esperando que Second Life se cierre..."
LangString CloseSecondLifeUnInstMB ${LANG_SPANISH} "Second Life no se puede desinstalar mientras esté en ejecución.$\n$\nTermine lo que esté haciendo y seleccione Aceptar (OK) para cerrar Second Life y continuar el proceso.$\nSeleccione Cancelar (CANCEL) para cancelar la desinstalación."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_SPANISH} "Comprobando la conexión de red..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_SPANISH} "¿Deseas ELIMINAR todos los archivos vinculados a Second Life también?$\n$\nSe recomienda guardar los parámetros de configuración y archivos caché si tienes otras versiones de Second Life instaladas o si estás desinstalando para actualizar a una nueva versión."

; delete program files
LangString DeleteProgramFilesMB ${LANG_SPANISH} "Aún hay archivos en su directorio de programa de Second Life.$\n$\nPosiblemente son archivos que ha creado o movido a:$\n$INSTDIR$\n$\n¿Desea eliminarlos?"

; uninstall text
LangString UninstallTextMsg ${LANG_SPANISH} "Este proceso desinstalará Second Life ${VERSION_LONG} de su sistema."

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_SPANISH} "¿Deseas eliminar las claves de registro de la aplicación?$\n$\nSe recomienda guardar las claves de registro de la aplicación si tienes otras versiones de Second Life instaladas."
