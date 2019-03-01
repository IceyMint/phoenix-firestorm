; First is default
LoadLanguageFile "${NSISDIR}\Contrib\Language files\PortugueseBR.nlf"

; Language selection dialog
LangString InstallerLanguageTitle  ${LANG_PORTUGUESEBR} "Instalador Linguagem"
LangString SelectInstallerLanguage  ${LANG_PORTUGUESEBR} "Por favor seleccione a linguagem do instalador"

; subtitle on license text caption
LangString LicenseSubTitleUpdate ${LANG_PORTUGUESEBR} " Atualizar"
LangString LicenseSubTitleSetup ${LANG_PORTUGUESEBR} " Instalar"

LangString MULTIUSER_TEXT_INSTALLMODE_TITLE ${LANG_PORTUGUESEBR} "Modo de Instalação"
LangString MULTIUSER_TEXT_INSTALLMODE_SUBTITLE ${LANG_PORTUGUESEBR} "Instalar para todos os usuários (necessário Administrador) ou somente para o usuário atual?"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_TOP ${LANG_PORTUGUESEBR} "Quando operar este instalador com os privilégios de Administrador, você pode escolher instalar em (ex.) c:\Program Files ou na pasta local do usuário AppData\Local."
LangString MULTIUSER_INNERTEXT_INSTALLMODE_ALLUSERS ${LANG_PORTUGUESEBR} "Instalar para todos os usuários"
LangString MULTIUSER_INNERTEXT_INSTALLMODE_CURRENTUSER ${LANG_PORTUGUESEBR} "Instalar somente para o usuário atual"

; installation directory text
LangString DirectoryChooseTitle ${LANG_PORTUGUESEBR} "Diretório de Instalação" 
LangString DirectoryChooseUpdate ${LANG_PORTUGUESEBR} "Selecione o diretório do Second Life para atualizar para a versão ${VERSION_LONG}.(XXX):"
LangString DirectoryChooseSetup ${LANG_PORTUGUESEBR} "Selecione o diretório para a instalação do Second Life em:"

LangString MUI_TEXT_DIRECTORY_TITLE ${LANG_PORTUGUESEBR} "Diretório de Instalação"
LangString MUI_TEXT_DIRECTORY_SUBTITLE ${LANG_PORTUGUESEBR} "Selecione o diretório ao qual deseja instalar o Second Life:"

LangString MUI_TEXT_INSTALLING_TITLE ${LANG_PORTUGUESEBR} "Instalando o Second Life..."
LangString MUI_TEXT_INSTALLING_SUBTITLE ${LANG_PORTUGUESEBR} "Instalando o visualizador Second Life para $INSTDIR"

LangString MUI_TEXT_FINISH_TITLE ${LANG_PORTUGUESEBR} "Instalando o Second Life"
LangString MUI_TEXT_FINISH_SUBTITLE ${LANG_PORTUGUESEBR} "Visualizador Second Life instalado para $INSTDIR."

LangString MUI_TEXT_ABORT_TITLE ${LANG_PORTUGUESEBR} "Instalação Anulada"
LangString MUI_TEXT_ABORT_SUBTITLE ${LANG_PORTUGUESEBR} "Visualizador Second Life não instalado para $INSTDIR."

; CheckStartupParams message box
LangString CheckStartupParamsMB ${LANG_PORTUGUESEBR} "Não é possível encontrar o programa '$INSTNAME'. Ocorreu uma falha na atualização silenciosa."

; installation success dialog
LangString InstSuccesssQuestion ${LANG_PORTUGUESEBR} "Iniciar o Second Life agora?"

; remove old NSIS version
LangString RemoveOldNSISVersion ${LANG_PORTUGUESEBR} "Verificando a versão anterior..."

; check windows version
LangString CheckWindowsVersionDP ${LANG_PORTUGUESEBR} "Verificando a versão do Windows..."
LangString CheckWindowsVersionMB ${LANG_PORTUGUESEBR} 'O Second Life suporta apenas Windows Vista.$\n$\nA tentativa de instalar no Windows $R0 pode resultar em falhas e perda de dados.$\n$\n'
LangString CheckWindowsServPackMB ${LANG_PORTUGUESEBR} "Recomenda-se utilizar o pacote de serviços mais atualizado do Second Life para o seu sistema operacional.$\nIsso ajudará no desempenho e estabilidade do programa."
LangString UseLatestServPackDP ${LANG_PORTUGUESEBR} "Use o Windows Update para instalar o pacote de serviços mais atualizado."

; checkifadministrator function (install)
LangString CheckAdministratorInstDP ${LANG_PORTUGUESEBR} "Verificando a permissão para instalação..."
LangString CheckAdministratorInstMB ${LANG_PORTUGUESEBR} 'Você parece estar usando uma conta "limitada".$\nVocê deve ser um "administrador" para poder instalar o Second Life.'

; checkifadministrator function (uninstall)
LangString CheckAdministratorUnInstDP ${LANG_PORTUGUESEBR} "Verificando a permissão para desinstalação..."
LangString CheckAdministratorUnInstMB ${LANG_PORTUGUESEBR} 'Você parece estar usando uma conta "limitada".$\nVocê deve ser um "administrador" para poder desinstalar o Second Life.'

; checkifalreadycurrent
LangString CheckIfCurrentMB ${LANG_PORTUGUESEBR} "Parece que o Second Life ${VERSION_LONG} já está instalado.$\n$\nDeseja instalar novamente?"

; checkcpuflags
LangString MissingSSE2 ${LANG_PORTUGUESEBR} "Esta máquina pode não ter um CPU com suporte SSE2, que é necessário para rodar o SecondLife ${VERSION_LONG}. Deseja continuar?"

; closesecondlife function (install)
LangString CloseSecondLifeInstDP ${LANG_PORTUGUESEBR} "Esperando o encerramento do Second Life..."
LangString CloseSecondLifeInstMB ${LANG_PORTUGUESEBR} "O Second Life não pode ser instalado enquanto ainda está sendo executado.$\n$\nTermine o que estava fazendo e selecione OK para fechar o Second Life e continuar.$\nSelecione CANCELAR para cancelar a instalação."

; closesecondlife function (uninstall)
LangString CloseSecondLifeUnInstDP ${LANG_PORTUGUESEBR} "Esperando o encerramento do Second Life..."
LangString CloseSecondLifeUnInstMB ${LANG_PORTUGUESEBR} "O Second Life não pode ser desinstalado enquanto ainda está sendo executado.$\n$\nTermine o que estava fazendo e selecione OK para fechar o Second Life e continuar.$\nSelecione CANCELAR para cancelar."

; CheckNetworkConnection
LangString CheckNetworkConnectionDP ${LANG_PORTUGUESEBR} "Verificando a conexão de rede..."

; ask to remove user's data files
LangString RemoveDataFilesMB ${LANG_PORTUGUESEBR} "Deseja REMOVER todos os outros arquivos relacionados ao Second Life também?$\n$\nRecomenda-se manter as configurações e os arquivos de cache se você tiver outras versões do Second Life instalada ou estiver desinstalando para instalar uma versão mais recente."

; delete program files
LangString DeleteProgramFilesMB ${LANG_PORTUGUESEBR} "Ainda existem arquivos em seu diretório do programa Second Life.$\n$\nProvavelmente são arquivos que você criou ou moveu para:$\n$INSTDIR$\n$\nDeseja removê-los?"

; uninstall text
LangString UninstallTextMsg ${LANG_PORTUGUESEBR} "Isso desinstalará o Second Life ${VERSION_LONG} do seu sistema."

; ask to remove registry keys that still might be needed by other viewers that are installed
LangString DeleteRegistryKeysMB ${LANG_PORTUGUESEBR} "Deseja remover as chaves de registro de aplicativo?$\n$\nRecomenda-se manter a chave de registro se você tiver outras versões do Second Life instalada."
