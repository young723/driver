1，acc文件夹是单加速度计功能驱动，acc-stepcounter是加速度计带计步器功能驱动。
2，此驱动适合MTK平台android L，KK。
3，带计步器功能，需要打开平台功能。
	KK版本：CUSTOM_KERNEL_STEP_COUNTER=yes	
	L版本：CONFIG_CUSTOM_KERNEL_STEP_COUNTER=y
4，计步器功能需要打开节点权限，如init.mt6735.rc
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cactive
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cdelay
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cbatch
	chmod 0660 /sys/class/misc/m_step_c_misc/step_cflush
	chown system system /sys/class/misc/m_step_c_misc/step_cactive
	chown system system /sys/class/misc/m_step_c_misc/step_cdelay
	chown system system /sys/class/misc/m_step_c_misc/step_cbatch
	chown system system /sys/class/misc/m_step_c_misc/step_cflush