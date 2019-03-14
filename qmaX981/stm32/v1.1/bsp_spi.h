#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"
#include <stdio.h>


#define WIP_Flag                  0x01
#define Dummy_Byte                0xFF


/*SPI接口定义-开头****************************/
#define      QST_SPIx                       SPI1
#define      QST_SPI_APBxClock_FUN          RCC_APB2PeriphClockCmd
#define      QST_SPI_CLK                    RCC_APB2Periph_SPI1

//CS(NSS)引脚 片选选普通GPIO即可
#define      QST_SPI_CS_APBxClock_FUN       RCC_APB2PeriphClockCmd
#define      QST_SPI_CS_CLK                 RCC_APB2Periph_GPIOA    
#define      QST_SPI_CS_PORT                GPIOA
#define      QST_SPI_CS_PIN                 GPIO_Pin_4

//SCK引脚
#define      QST_SPI_SCK_APBxClock_FUN      RCC_APB2PeriphClockCmd
#define      QST_SPI_SCK_CLK                RCC_APB2Periph_GPIOA   
#define      QST_SPI_SCK_PORT               GPIOA   
#define      QST_SPI_SCK_PIN                GPIO_Pin_5
//MISO引脚
#define      QST_SPI_MISO_APBxClock_FUN     RCC_APB2PeriphClockCmd
#define      QST_SPI_MISO_CLK               RCC_APB2Periph_GPIOA    
#define      QST_SPI_MISO_PORT              GPIOA 
#define      QST_SPI_MISO_PIN               GPIO_Pin_6
//MOSI引脚
#define      QST_SPI_MOSI_APBxClock_FUN		RCC_APB2PeriphClockCmd
#define      QST_SPI_MOSI_CLK               RCC_APB2Periph_GPIOA    
#define      QST_SPI_MOSI_PORT              GPIOA 
#define      QST_SPI_MOSI_PIN               GPIO_Pin_7

#define  	 SPI_QST_CS_LOW()     			GPIO_ResetBits( QST_SPI_CS_PORT, QST_SPI_CS_PIN )
#define  	 SPI_QST_CS_HIGH()    			GPIO_SetBits( QST_SPI_CS_PORT, QST_SPI_CS_PIN )

/*SPI接口定义-结尾****************************/

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

u8 qmaX981_spi_read(u8 addr, u8* buff, u8 len);
u8 qmaX981_spi_write(u8 addr,u8 data);
void Spi_Init(void);

#endif /* __SPI_QST_H */

