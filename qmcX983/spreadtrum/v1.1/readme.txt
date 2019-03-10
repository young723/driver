kernel\arch\arm\configs
xxx_defconfig
	CONFIG_INPUT_QMCX983=y
	
kernel\arch\arm\boot\dts
xxx.dts
	qmcX983@2c{                                                                                     
		compatible = "qst,qmcX983"
		reg = <0x2c>;
		qst,direction = <1>;
	};
	
kernel\drivers\input\misc
Makefile
obj-$(CONFIG_INPUT_QMCX983)          += qmcX983.o

Kconfig
config INPUT_QMCX983
	tristate "QST magnetic device with I2C bus"
	depends on I2C
	default n
	help
	Say Y here if you have a QST magnetic device on the board and use I2C
	communication, else say N.

	To compile this driver as a module, choose M here
	   
kernel\drivers\input\misc\qmcX983.c
kernel\include\linux\i2c\qmcX983.h

more information, refer to diff_log.txt

	   
