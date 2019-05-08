driver path:
tinysys\freertos\source\middleware\contexthub\MEMS_Driver\magnetometer\qmc6308
tinysys\freertos\source\middleware\contexthub\MEMS_Driver\magnetometer\lib\qmc6308

// 客制化，i2c id，layout等
tinysys\freertos\source\project\CM4_A\mt6771\customer_prj\cust\magnetometer\cust_mag.c

// qmc6308功能开关
tinysys\freertos\source\project\CM4_A\mt6771\customer_prj\ProjectConfig.mk
CFG_QMC6308_SUPPORT = yes

// 加入qmc6308源文件，库文件，编译
tinysys\freertos\source\project\CM4_A\mt6771\platform\feature_config\chre.mk

// overlay相关代码，目的在启动时只加载通讯OK的设备到RAM。
freertos\source\project\CM4_A\mt6771\customer_prj\inc\overlay_sensor.h
tinysys\freertos\source\project\CM4_A\mt6771\customer_prj\cust\overlay\overlay.c

//change SCP code size
tinysys\freertos\source\project\CM4_A\mt6771\platform\Setting.ini
