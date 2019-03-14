
#include "./usart/bsp_usart.h"
#include "./i2c/bsp_i2c.h"
#include "./i2c/qst_sw_i2c.h"

#include "./fis210x/fis210x.h"

//#define FIS210X_USE_SW_IIC

#if defined(USE_SPI)
extern uint8_t qst_fis210x_spi_write(uint8_t Addr, uint8_t Data);
extern uint8_t qst_fis210x_spi_read(uint8_t Addr, uint8_t *pData, uint16_t Length);
#endif
extern void qst_delay(unsigned int delay);

#define FIS210X_SLAVE_ADDR		0xd4	// 0xd6		(0x6a<<1)
#define qst_printf			console_write

static uint16_t acc_lsb_div = 0;
static uint16_t gyro_lsb_div = 0;

uint8_t fis210x_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t ret=0;

#if defined(FIS210X_USE_SW_IIC)
	ret = qst_sw_writereg(FIS210X_SLAVE_ADDR, reg, value);
#else
	I2C_Bus_set_slave_addr(FIS210X_SLAVE_ADDR);
	ret = I2C_ByteWrite(reg,value);
#endif
	return ret;
}

uint8_t fis210x_read_reg(uint8_t reg, uint8_t* buf, uint16_t len)
{
	uint8_t ret=0;

#if defined(FIS210X_USE_SW_IIC)
	ret = qst_sw_readreg(FIS210X_SLAVE_ADDR, reg, buf, len);
#else
	I2C_Bus_set_slave_addr(FIS210X_SLAVE_ADDR);
	ret = I2C_BufferRead(buf, reg, len);
#endif
	return ret;
}

void fis210x_config_acc(int range, int odr)
{
	unsigned char ctl_dada;
	unsigned char range_set;

	switch (range)
	{
		case AccRange_2g:
			range_set = 0<<3;
			acc_lsb_div = (1<<14);
			break;
		case AccRange_4g:
			range_set = 1<<3;
			acc_lsb_div = (1<<13);
			break;
		case AccRange_8g:
			range_set = 2<<3;
			acc_lsb_div = (1<<12);
			break;
		default: 
			range_set = 2<<3;
			acc_lsb_div = (1<<12);
	}
	ctl_dada = (unsigned char)range_set|(unsigned char)odr;
	fis210x_write_reg(FisRegister_Ctrl2, ctl_dada);
#if 0
// set lpf enable
	read_reg(FisRegister_Ctrl5, &ctl_dada,1);
	ctl_dada &= 0xfc;
	ctl_dada |=0x02;
	write_reg(FisRegister_Ctrl5,ctl_dada);
// set lpf enable
#endif
}

void fis210x_config_gyro(int range, int odr)
{
	// Set the CTRL3 register to configure dynamic range and ODR
	unsigned char ctl_dada; 
	ctl_dada = (unsigned char)range | (unsigned char)odr;
	fis210x_write_reg(FisRegister_Ctrl3, ctl_dada);

	// Store the scale factor for use when processing raw data
	switch (range)
	{
		case GyrRange_32dps:
			gyro_lsb_div = 1024;
			break;
		case GyrRange_64dps:
			gyro_lsb_div = 512;
			break;
		case GyrRange_128dps:
			gyro_lsb_div = 256;
			break;
		case GyrRange_256dps:
			gyro_lsb_div = 128;
			break;
		case GyrRange_512dps:
			gyro_lsb_div = 64;
			break;
		case GyrRange_1024dps:
			gyro_lsb_div = 32;
			break;
		case GyrRange_2048dps:
			gyro_lsb_div = 16;
			break;
		case GyrRange_2560dps:
			//gyro_lsb_div = 8;
			break;
		default: 
			gyro_lsb_div = 32;
			break;
	}

	// Conversion from degrees/s to rad/s if necessary
#if 0
	read_reg(FisRegister_Ctrl5,&ctl_dada,1);
	ctl_dada &= 0xf3;
	ctl_dada |=0x08;
	write_reg(FisRegister_Ctrl5,ctl_dada);
#endif
}


void fis210x_read_acc_xyz(void)
{
	float			acc_xyz_tmp[3];	
	unsigned char	buf_reg[6];
	short 			raw_acc_xyz[3];

#if defined(USE_SPI)
	fis210x_read_reg(FisRegister_Ax_L, &buf_reg[0], 6); 	// 0x19, 25
#else
	fis210x_read_reg(FisRegister_Ax_L, &buf_reg[0], 1);		// 0x19, 25
	fis210x_read_reg(FisRegister_Ax_H, &buf_reg[1], 1);
	fis210x_read_reg(FisRegister_Ay_L, &buf_reg[2], 1);
	fis210x_read_reg(FisRegister_Ay_H, &buf_reg[3], 1);
	fis210x_read_reg(FisRegister_Az_L, &buf_reg[4], 1);
	fis210x_read_reg(FisRegister_Az_H, &buf_reg[5], 1);
#endif
	raw_acc_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_acc_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_acc_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));
//	if(cali_flag == FALSE)
//	{
//		raw_acc_xyz[0]+=offset[0];
//		raw_acc_xyz[1]+=offset[1];
//		raw_acc_xyz[2]+=offset[2];
//	}

	acc_xyz_tmp[0] = (raw_acc_xyz[0]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[1] = (raw_acc_xyz[1]*9.807f)/acc_lsb_div;
	acc_xyz_tmp[2] = (raw_acc_xyz[2]*9.807f)/acc_lsb_div;
//	acc_layout = 0;
//	acc_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*acc_xyz_tmp[0];
//	acc_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*acc_xyz_tmp[1];
//	acc_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*acc_xyz_tmp[2];
	qst_printf("fis210x acc:	%f	%f	%f\n", acc_xyz_tmp[0], acc_xyz_tmp[1], acc_xyz_tmp[2]);
}

void fis210x_read_gyro_xyz(void)
{
	float			gyro_xyz_tmp[3];	
	unsigned char	buf_reg[6];
	short 			raw_gyro_xyz[3];

#if defined(USE_SPI)
	fis210x_read_reg(FisRegister_Gx_L, &buf_reg[0], 6); 	// 0x19, 25
#else
	fis210x_read_reg(FisRegister_Gx_L, &buf_reg[0], 1);	
	fis210x_read_reg(FisRegister_Gx_H, &buf_reg[1], 1);
	fis210x_read_reg(FisRegister_Gy_L, &buf_reg[2], 1);
	fis210x_read_reg(FisRegister_Gy_H, &buf_reg[3], 1);
	fis210x_read_reg(FisRegister_Gz_L, &buf_reg[4], 1);
	fis210x_read_reg(FisRegister_Gz_H, &buf_reg[5], 1);
#endif
	raw_gyro_xyz[0] = (short)((buf_reg[1]<<8) |( buf_reg[0]));
	raw_gyro_xyz[1] = (short)((buf_reg[3]<<8) |( buf_reg[2]));
	raw_gyro_xyz[2] = (short)((buf_reg[5]<<8) |( buf_reg[4]));	
//	if(cali_flag == FALSE)
//	{
//		raw_gyro_xyz[0] += offset_gyro[0];
//		raw_gyro_xyz[1] += offset_gyro[1];
//		raw_gyro_xyz[2] += offset_gyro[2];
//	}

	gyro_xyz_tmp[0] = (raw_gyro_xyz[0]*1.0f)/gyro_lsb_div;
	gyro_xyz_tmp[1] = (raw_gyro_xyz[1]*1.0f)/gyro_lsb_div;
	gyro_xyz_tmp[2] = (raw_gyro_xyz[2]*1.0f)/gyro_lsb_div;
//	acc_layout = 0;
//	gyro_xyz[qst_map[acc_layout].map[0]] = qst_map[acc_layout].sign[0]*gyro_xyz_tmp[0];
//	gyro_xyz[qst_map[acc_layout].map[1]] = qst_map[acc_layout].sign[1]*gyro_xyz_tmp[1];
//	gyro_xyz[qst_map[acc_layout].map[2]] = qst_map[acc_layout].sign[2]*gyro_xyz_tmp[2];
	qst_printf("fis210x gyro:	%f	%f	%f\n", gyro_xyz_tmp[0], gyro_xyz_tmp[1], gyro_xyz_tmp[2]);
}



uint8_t fis210x_init(void)
{
	uint8_t chip_id = 0x00;

#if defined(FIS210X_USE_SW_IIC)
	i2c_CheckDevice(FIS210X_SLAVE_ADDR);
	qst_delay(50);
#endif
	fis210x_read_reg(FisRegister_WhoAmI, &chip_id, 1);
	if(chip_id == 0xfc)
	{
		fis210x_config_acc(AccRange_4g, AccOdr_128Hz);
		fis210x_config_gyro(GyrRange_1024dps, GyrOdr_250Hz);
		
		fis210x_write_reg(FisRegister_Ctrl7, 0x03);
	}
	else
	{
		chip_id = 0;
	}

	return chip_id;
}
