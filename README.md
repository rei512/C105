# 雑に回すBLDCモーター (C105新刊) プログラム配布
新刊内で解説していた内容とプログラムの配布版です．
<pre>
.
├── firmware
│   ├── Core
│   │   ├── Inc
│   │   │   ├── main.h
│   │   │   ├── stm32f4xx_hal_conf.h
│   │   │   └── stm32f4xx_it.h
│   │   ├── Src
│   │   │   ├── main.c
│   │   │   ├── stm32f4xx_hal_msp.c
│   │   │   ├── stm32f4xx_it.c
│   │   │   ├── syscalls.c
│   │   │   ├── sysmem.c
│   │   │   └── system_stm32f4xx.c
│   │   └── Startup
│   │       └── startup_stm32f401retx.s
│   ├── .cproject
│   ├── firmware.ioc
│   ├── .mxproject
│   ├── .project
│   ├── .settings
│   │   ├── language.settings.xml
│   │   ├── org.eclipse.cdt.codan.core.prefs
│   │   ├── org.eclipse.cdt.core.prefs
│   │   └── stm32cubeide.project.prefs
│   ├── STM32F401RETX_FLASH.ld
│   └── STM32F401RETX_RAM.ld
├── .git
├── .gitignore
└── README.md
</pre>

./firmware/.projectをSTM32CubeIDEで開いてコンパイルしてください
ああaa