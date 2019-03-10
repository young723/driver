1，把qma6981.c，qma6981.h放入目录中。
2，在kernel/arch/arm64/configs/xxx_defconfig中添加CONFIG_SENSORS_QMA6981=y，
同时在kernel\drivers\input\misc中的Kconfig和Makefile添加相应语句。
3，kernel/arch/arm64/boot/dts/qcom中的设备树文件dtsi中添加qma6981相关内容，如：
qma6981@12 {
		
compatible = "qst,qma6981";

reg = <0x12>;

vdd-supply = <&pm8916_l17>;

vddio-supply = <&pm8916_l6>;

qst,layout = <5>;

qst,poll_report = <1>;

};
