#ifndef _BSP_FLASH_W25Q64_H_
#define _BSP_FLASH_W25Q64_H_

/**
鏈▼搴忕敤浜巜25q64,鍏朵粬鍐呭瓨flash閫氱敤
鏈変簺鍦版柟绋嶅仛淇敼鍗冲彲
w25q64鏈?MB鍐呭瓨
涓€椤?56涓瓧鑺?
4096瀛楄妭涔熷氨鏄紙4KB锛変负涓€涓墖鍖?
16涓墖鍖轰负涓€鍧楋紙65536瀛楄妭锛?
涓€鍏?28鍧楋紙8388608瀛楄妭锛?
This program is used for w25q64, and other memory flash is common
Some areas can be slightly modified
W25q64 has 8MB of memory
256 bytes per page
4096 bytes, which means (4KB) is a sector
16 sectors in one block (65536 bytes)
A total of 128 blocks (8388608 bytes)
**/

#include "AllHeader.h"

#define W25Q80 	0XEF13
#define W25Q16 	0XEF14
#define W25Q32 	0XEF15
#define W25Q64 	0XEF16 //ID


extern u16 SPI_FLASH_TYPE;		//Define the flash chip model we use 瀹氫箟鎴戜滑浣跨敤鐨刦lash鑺墖鍨嬪彿
extern u8 SPI_FLASH_BUF[4096];


//SPI_CS
#define CS_LOW  {DL_GPIO_clearPins(SPI_PORT,SPI_CS_PIN);}
#define CS_HIGH {DL_GPIO_setPins(SPI_PORT,SPI_CS_PIN);}//DL_GPIO_setPins DL_GPIO_clearPins



//W25X64璇诲啓
#define FLASH_ID 0XEF16

//鎸囦护琛?
#define W25X_WriteEnable		0x06  //Enable Write Enable 寮€鍚啓浣胯兘
#define W25X_WriteDisable		0x04  //Turn off write enable 鍏抽棴鍐欎娇鑳?
#define W25X_ReadStatusReg		0x05  //Read Status Register 璇荤姸鎬佸瘎瀛樺櫒
#define W25X_WriteStatusReg		0x01
#define W25X_ReadData			0x03  //READ 璇诲懡浠?
#define W25X_FastReadData		0x0B
#define W25X_FastReadDual		0x3B
#define W25X_PageProgram		0x02  //WRITE 鍐欏懡浠?
#define W25X_BlockErase			0xD8  //Block erase 64kB 鍧楁摝闄?4kB
#define W25X_SectorErase		0x20  //Sector erase 4kB 鎵囧尯鎿﹂櫎4kB
#define W25X_ChipErase			0xC7  //Erase All 鍏ㄩ儴鎿﹂櫎
#define W25X_PowerDown			0xB9  //Power saving mode 鐪佺數妯″紡
#define W25X_ReleasePowerDown	0xAB  //wake up mode 鍞ら啋妯″紡
#define W25X_DeviceID			0xAB
#define W25X_ManufactDeviceID	0x90  //Read vendor 璇诲彇鍘傚晢id
#define W25X_JedecDeviceID		0x9F


void W25Q64_Init(void);
uint8_t SPI_Read_Write_Byte(uint8_t BYTE);
u16  Spi_Flash_ReadID(void);  	    								//Read FLASH ID
u16 Spi_Flash_ReadID_two(void);
u8	 Spi_Flash_Read_SR(void);        								//Read Status Register
void Spi_Flash_Write_SR(u8 sr);  									//Write Status Register
void Spi_Flash_Write_Enable(void);  								//write enable
void Spi_Flash_Write_Disable(void);									//Write protected
void Spi_Flash_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead);    //read flash
void Spi_Flash_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite); //write flash
void Spi_Flash_Erase_Chip(void);    	  							//Chip Erase 鏁寸墖鎿﹂櫎
void Spi_Flash_Erase_Sector(u32 Dst_Addr);							//Sector Erase 鎵囧尯鎿﹂櫎
void Spi_Flash_BlockErase_Sector(u32 Dst_Addr);						//Block Erase 鍧楁摝闄?
void Spi_Flash_Wait_Busy(void);           							//Waiting for idle 绛夊緟绌洪棽
void Spi_Flash_Power_Down(void);           							//Entering power down mode 杩涘叆鎺夌數妯″紡
void Spi_Flash_WAKEUP(void);			  							//Wakeup 鍞ら啋

void Spi_Flash_Exit4ByteMode(void);
uint8_t SPI_Flash_ReadSR(uint8_t reg_num);
#endif
