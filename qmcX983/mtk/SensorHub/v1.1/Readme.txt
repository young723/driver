driver path:
tinysys\freertos\source\middleware\contexthub\MEMS_Driver\magnetometer
tinysys\freertos\source\middleware\contexthub\MEMS_Driver\magnetometer\lib

cust_mag.c:
(tinysys\freertos\source\project\CM4_A\mt6771\e906_public_k71v1_64_bsp\cust\magnetometer)
#ifdef CFG_QMC7983_SUPPORT
    {

        .name = "qmc7983",
        .i2c_num = 1,
        .direction = 4,
        .i2c_addr = {0x2c, 0},
    },
#endif

ProjectConfig.mk:(tinysys\freertos\source\project\CM4_A\mt6771\e906_public_k71v1_64_bsp)
CFG_QMC7983_SUPPORT = yes(可关闭其他不用的设备)


chre.mk:
(tinysys\freertos\source\project\CM4_A\mt6771\platform\feature_config)
ifeq ($(CFG_QMC7983_SUPPORT),yes)
C_FILES  += $(SENDRV_DIR)/magnetometer/qmc7983.c
CFG_QST_VENDOR_SUPPORT=yes
endif

ifeq ($(CFG_QST_VENDOR_SUPPORT),yes)
INCLUDES += -I$(SENLIB_DIR)/qmc7983/
LIBFLAGS += -L$(SENLIB_DIR)/qmc7983 -lQstAlgo
endif

overlay_sensor.h:
(freertos\source\project\CM4_A\mt6771\e906_public_k71v1_64_bsp\inc)
OVERLAY_ONE_OBJECT(qmc7983, qmc7983)\
OVERLAY_LIB_OBJECT(qmc7983, libQstAlgo, ical*)

overlay.c:
(tinysys\freertos\source\project\CM4_A\mt6771\e906_public_k71v1_64_bsp\cust\overlay)
 MAG_OVERLAY_REMAP(qmc7983);
 
 
 //change SCP code size
 tinysys\freertos\source\project\CM4_A\mt6771\platform\Setting.ini
 
 Platform:******
 
