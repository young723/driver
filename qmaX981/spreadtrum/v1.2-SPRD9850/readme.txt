1 xxx_defconfig

	CONFIG_SENSORS_QMAX981=y


2 Makefile

	obj-$(CONFIG_SENSORS_QMAX981)	+= qmaX981.o


3 Kconfig

	config SENSORS_QMAX981
	
	tristate "QST QMAX981 3-axis digital acceleromater"
	
	depends on I2C
	
	help
	  Say Y here if you want to support QST Ltd QMAX981
	  
	accelerometer connected via an I2C bus.

	  
	To compile this driver as a module, choose M here: the
	  module will be called qmaX981.



4 xxx.dtsi

	qmax981@12 {
		
	compatible = "qst,qmax981";
		
	reg = <0x12>;
		
	vdd-supply = <&pm8916_l17>;
		
	vddio-supply = <&pm8916_l6>;
		
	qst,layout = <5>;
		
	qst,poll_report = <1>;
	
	};
	
5. file_texts  

/dev/qmax981         u:object_r:sensors_device:s0


6. ueventd.common.rc

/dev/qmax981         0660     system     input