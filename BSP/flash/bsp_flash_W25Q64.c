#include "bsp_flash_W25Q64.h"

#define HAL_Delay(x) delay_ms(x)

u16 SPI_FLASH_TYPE = W25Q64; // 榛樿灏辨槸25Q64 The default is 25Q64

// flash init
void W25Q64_Init(void)
{

	delay_ms(300);
    

	SPI_FLASH_TYPE = Spi_Flash_ReadID();
    
    
    
    while(SPI_FLASH_TYPE!=W25Q64 && SPI_FLASH_TYPE!=W25Q32 )
    {
        SPI_FLASH_TYPE = Spi_Flash_ReadID();
        delay_ms(300);
    }
}


//閫€鍑?瀛楄妭妯″紡锛岀敤3瀛楄妭
void Spi_Flash_Exit4ByteMode(void)
{
    CS_LOW;
    SPI_Read_Write_Byte(0xE9); // 閫€鍑?瀛楄妭鍦板潃妯″紡鍛戒护
    CS_HIGH;
    delay_us(10);
}


uint8_t SPI_Flash_ReadSR(uint8_t reg_num)
{
    uint8_t cmd = 0x05;
    if(reg_num == 2) cmd = 0x35;
    if(reg_num == 3) cmd = 0x15; // W25Q64涓撶敤鐘舵€佸瘎瀛樺櫒3
    
    uint8_t sr;
    CS_LOW;
    SPI_Read_Write_Byte(cmd);
    sr = SPI_Read_Write_Byte(0xFF);
    CS_HIGH;
    return sr;
}


/**********************
spi鍗忚鍙戦€?涓瓧鑺傚嚱鏁?
uint8_t BYTE 锛氫綘闇€瑕佸彂閫佺殑瀛楄妭鏁版嵁
SPI protocol sends 1 byte function
Uint8_ BYTE: Byte data you need to send
*********************/
uint8_t SPI_Read_Write_Byte(uint8_t BYTE)
{
	uint8_t SNED_BYTE = BYTE;
	uint8_t READ_BYTE = 0;
    
     //绛夊緟SPI鎬荤嚎绌洪棽
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    
    //鍙戦€佹暟鎹?
    DL_SPI_transmitData8(SPI_W25Q64_INST,SNED_BYTE);
    //绛夊緟SPI鎬荤嚎绌洪棽
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    //鎺ユ敹鏁版嵁
    READ_BYTE = DL_SPI_receiveData8(SPI_W25Q64_INST);
    //绛夊緟SPI鎬荤嚎绌洪棽
    while(DL_SPI_isBusy(SPI_W25Q64_INST));
    
	return READ_BYTE;
}

// Read SPI_ Status register of FLASH  璇诲彇SPI_FLASH鐨勭姸鎬佸瘎瀛樺櫒
// BIT7  6   5   4   3   2   1   0
// SPR   RV  TB BP2 BP1 BP0 WEL BUSY
// SPR:  default 0, status register protection bit, used in conjunction with WP 榛樿0,鐘舵€佸瘎瀛樺櫒淇濇姢浣?閰嶅悎WP浣跨敤
// TB,BP2,BP1,BP0:FLASH region write protection settings  FLASH鍖哄煙鍐欎繚鎶よ缃?
// WEL:Write enable lock  鍐欎娇鑳介攣瀹?
// BUSY:busy flag bit (1, busy; 0, idle) 蹇欐爣璁颁綅(1,蹇?0,绌洪棽)
// Default:0x00
u8 Spi_Flash_Read_SR(void)
{
	u8 byte = 0;
	CS_LOW;									 // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_ReadStatusReg); // Send Read Status Register Command 鍙戦€佽鍙栫姸鎬佸瘎瀛樺櫒鍛戒护
	byte = SPI_Read_Write_Byte(0Xff);		 // Read a Byte 璇诲彇涓€涓瓧鑺?
	CS_HIGH;								 // Cancel Film Selection 鍙栨秷鐗囬€?
	return byte;
}
// Write SPI_ FLASH status register 鍐橲PI_FLASH鐘舵€佸瘎瀛樺櫒
// Only SPR, TB, BP2, BP1, BP0 (bit 7,5,4,3,2) can be written!!! 鍙湁SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)鍙互鍐?!!
void Spi_Flash_Write_SR(u8 sr)
{
	CS_LOW;									  // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_WriteStatusReg); // Send Write Status Register Command 鍙戦€佸啓鍙栫姸鎬佸瘎瀛樺櫒鍛戒护
	SPI_Read_Write_Byte(sr);				  // Write a byte 鍐欏叆涓€涓瓧鑺?
	CS_HIGH;								  // Cancel Film Selection 鍙栨秷鐗囬€?
}
// SPI_ FLASH write enable SPI_FLASH鍐欎娇鑳?
// Set WEL to position 灏哤EL缃綅
void Spi_Flash_Write_Enable(void)
{
	CS_LOW;								   // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_WriteEnable); // Send Write Enable 鍙戦€佸啓浣胯兘
	CS_HIGH;							   // Cancel Film Selection 鍙栨秷鐗囬€?
}
// SPI_ FLASH write inhibit SPI_FLASH鍐欑姝?
// Reset WEL to zero 灏哤EL娓呴浂
void Spi_Flash_Write_Disable(void)
{
	CS_LOW;									// Reset WEL to zero 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_WriteDisable); // Send write inhibit command s鍙戦€佸啓绂佹鎸囦护
	CS_HIGH;								// Cancel Film Selection 鍙栨秷鐗囬€?
}

// 鍑芥暟鍔熻兘锛氳鍙朓D
// 浼犲叆鍙傛暟:鏃?
// 杩斿洖鍙傛暟锛欼D鍙?hex)
// Function function: Read ID
// Incoming parameter: None
// Return parameter: ID number (hex)
u16 Spi_Flash_ReadID(void)
{
	u16 Temp = 0;
	CS_LOW;
	SPI_Read_Write_Byte(0x90); // Send Read ID Command 鍙戦€佽鍙朓D鍛戒护
	SPI_Read_Write_Byte(0x00);
	SPI_Read_Write_Byte(0x00);
	SPI_Read_Write_Byte(0x00);
	Temp |= SPI_Read_Write_Byte(0xFF) << 8;
	Temp |= SPI_Read_Write_Byte(0xFF);
	CS_HIGH;
	return Temp;
}



u16 Spi_Flash_ReadID_two(void) {
    u16 Temp = 0;
    CS_LOW;
    SPI_Read_Write_Byte(0x9F);  // JEDEC ID 鏍囧噯鍛戒护
    Temp = SPI_Read_Write_Byte(0xFF) << 8;  // 鍒堕€犲晢ID (搴?xEF)
    Temp |= SPI_Read_Write_Byte(0xFF);      // 璁惧ID楂樺瓧鑺?(搴?x40)
    // uint8_t capacity = SPI_Read_Write_Byte(0xFF); // 瀹归噺瀛楄妭(搴?x17)
    CS_HIGH;
    return Temp;
}


// 璇诲彇SPI FLASH
// 鍦ㄦ寚瀹氬湴鍧€寮€濮嬭鍙栨寚瀹氶暱搴︾殑鏁版嵁
// pBuffer:鏁版嵁瀛樺偍鍖?
// ReadAddr:寮€濮嬭鍙栫殑鍦板潃(24bit)
// NumByteToRead:瑕佽鍙栫殑瀛楄妭鏁?鏈€澶?5535)
// Read SPI FLASH
// Starting from the specified address to read data of the specified length
// PBuffer: data storage area
// ReadAddr: Address to start reading (24bit)
// NumByteToRead: Number of bytes to read (maximum 65535)
void Spi_Flash_Read(u8 *pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
	u16 i;
	CS_LOW;										 // Enabling device
	SPI_Read_Write_Byte(W25X_ReadData);			 // Send read command
	SPI_Read_Write_Byte((u8)((ReadAddr) >> 16)); // Send a 24bit address
	SPI_Read_Write_Byte((u8)((ReadAddr) >> 8));
	SPI_Read_Write_Byte((u8)ReadAddr);
	for (i = 0; i < NumByteToRead; i++)
	{
		pBuffer[i] = SPI_Read_Write_Byte(0XFF); // Cyclic reading
	}
	CS_HIGH; // Cancel Film Selection
}

// SPI鍦ㄤ竴椤?0~65535)鍐呭啓鍏ュ皯浜?56涓瓧鑺傜殑鏁版嵁
// 鍦ㄦ寚瀹氬湴鍧€寮€濮嬪啓鍏ユ渶澶?56瀛楄妭鐨勬暟鎹?
// pBuffer:鏁版嵁瀛樺偍鍖?
// WriteAddr:寮€濮嬪啓鍏ョ殑鍦板潃(24bit)
// NumByteToWrite:瑕佸啓鍏ョ殑瀛楄妭鏁?鏈€澶?56),璇ユ暟涓嶅簲璇ヨ秴杩囪椤电殑鍓╀綑瀛楄妭鏁?!!
// SPI writes less than 256 bytes of data on a page (0-65535)
// Write up to 256 bytes of data starting at the specified address
// PBuffer: data storage area
// WriteAddr: Address to start writing (24bit)
// NumByteToWrite: The number of bytes to write (maximum 256), which should not exceed the remaining bytes on the page!!!
void Spi_Flash_Write_Page(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 i;
	Spi_Flash_Write_Enable();					  // SET WEL
	CS_LOW;										  // Enabling device
	SPI_Read_Write_Byte(W25X_PageProgram);		  // Send page write command
	SPI_Read_Write_Byte((u8)((WriteAddr) >> 16)); // Send a 24bit address
	SPI_Read_Write_Byte((u8)((WriteAddr) >> 8));
	SPI_Read_Write_Byte((u8)WriteAddr);
	for (i = 0; i < NumByteToWrite; i++)
		SPI_Read_Write_Byte(pBuffer[i]); // Number of cyclic writes
	CS_HIGH;							 // Cancel Film Selection
	Spi_Flash_Wait_Busy();				 // Waiting for write to end
}

/********************************************************
// 鏃犳楠屽啓SPI FLASH
// 蹇呴』纭繚鎵€鍐欑殑鍦板潃鑼冨洿鍐呯殑鏁版嵁鍏ㄩ儴涓?XFF,鍚﹀垯鍦ㄩ潪0XFF澶勫啓鍏ョ殑鏁版嵁灏嗗け璐?
// 鍏锋湁鑷姩鎹㈤〉鍔熻兘
// 鍦ㄦ寚瀹氬湴鍧€寮€濮嬪啓鍏ユ寚瀹氶暱搴︾殑鏁版嵁,浣嗘槸瑕佺‘淇濆湴鍧€涓嶈秺鐣?
// pBuffer:鏁版嵁瀛樺偍鍖?
// WriteAddr:寮€濮嬪啓鍏ョ殑鍦板潃(24bit)
// NumByteToWrite:瑕佸啓鍏ョ殑瀛楄妭鏁?鏈€澶?5535)
// CHECK OK
//Write SPI FLASH without verification
//It is necessary to ensure that all data within the address range written is 0XFF, otherwise data written at non 0XFF locations will fail!
//With automatic page wrapping function
//Start writing data of the specified length at the specified address, but ensure that the address does not exceed the limit!
//PBuffer: data storage area
//WriteAddr: Address to start writing (24bit)
//NumByteToWrite: Number of bytes to write (maximum 65535)
//CHECK OK
**********************************************************/
void Spi_Flash_Write_NoCheck(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u16 pageremain;
	pageremain = 256 - WriteAddr % 256; // Number of bytes remaining on a single page
	if (NumByteToWrite <= pageremain)
		pageremain = NumByteToWrite; // No more than 256 bytes 涓嶅ぇ浜?56涓瓧鑺?
	while (1)
	{
		Spi_Flash_Write_Page(pBuffer, WriteAddr, pageremain);
		// Writing completed 鍐欏叆缁撴潫浜?
		if (NumByteToWrite == pageremain)
			break;
		else // NumByteToWrite>pageremain
		{
			pBuffer += pageremain;
			WriteAddr += pageremain;

			NumByteToWrite -= pageremain; // Subtract the number of bytes already written 鍑忓幓宸茬粡鍐欏叆浜嗙殑瀛楄妭鏁?
			if (NumByteToWrite > 256)
				pageremain = 256; // Can write 256 bytes at a time 涓€娆″彲浠ュ啓鍏?56涓瓧鑺?
			else
				pageremain = NumByteToWrite; // Not enough 256 bytes left 涓嶅256涓瓧鑺備簡
		}
	};
}
// Write SPI FLASH 鍐橲PI FLASH
// Write data of the specified length starting from the specified address 鍦ㄦ寚瀹氬湴鍧€寮€濮嬪啓鍏ユ寚瀹氶暱搴︾殑鏁版嵁
// This function has an erase operation! 璇ュ嚱鏁板甫鎿﹂櫎鎿嶄綔!
// pBuffer: data storage area 鏁版嵁瀛樺偍鍖?
// WriteAddr:Address to start writing (24bit) 寮€濮嬪啓鍏ョ殑鍦板潃(24bit)
// NumByteToWrite:Number of bytes to write (maximum 65535) 瑕佸啓鍏ョ殑瀛楄妭鏁?鏈€澶?5535)
u8 SPI_FLASH_BUF[4096];
void Spi_Flash_Write(u8 *pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
	u32 secpos;
	u16 secoff;
	u16 secremain;
	u16 i;

	secpos = WriteAddr / 4096; // sector address
	secoff = WriteAddr % 4096; // Offset within sectors
	secremain = 4096 - secoff; // Size of remaining space in the sector

	// No more than 4096 bytes 涓嶅ぇ浜?096涓瓧鑺?
	if (NumByteToWrite <= secremain)
		secremain = NumByteToWrite;
	while (1)
	{
		// Read the content of the entire sector 璇诲嚭鏁翠釜鎵囧尯鐨勫唴瀹?
		Spi_Flash_Read(SPI_FLASH_BUF, secpos * 4096, 4096);
		for (i = 0; i < secremain; i++) // Verification data 鏍￠獙鏁版嵁
		{
			// Need to erase 闇€瑕佹摝闄?
			if (SPI_FLASH_BUF[secoff + i] != 0XFF)
				break;
		}
		if (i < secremain) // Need to erase 闇€瑕佹摝闄?
		{
			Spi_Flash_Erase_Sector(secpos); // Erase this sector 鎿﹂櫎杩欎釜鎵囧尯
			for (i = 0; i < secremain; i++) // Copy 澶嶅埗
			{
				SPI_FLASH_BUF[i + secoff] = pBuffer[i];
			}
			// Write entire sector 鍐欏叆鏁翠釜鎵囧尯
			Spi_Flash_Write_NoCheck(SPI_FLASH_BUF, secpos * 4096, 4096);
		}
		else // Write what has been erased and directly write to the remaining interval of the sector 鍐欏凡缁忔摝闄や簡鐨?鐩存帴鍐欏叆鎵囧尯鍓╀綑鍖洪棿.
			Spi_Flash_Write_NoCheck(pBuffer, WriteAddr, secremain);
		if (NumByteToWrite == secremain)
			break; // Writing completed 鍐欏叆缁撴潫浜?
		else	   // Write not ended 鍐欏叆鏈粨鏉?
		{
			secpos++;	// Increase sector address by 1 鎵囧尯鍦板潃澧?
			secoff = 0; // Offset position is 0 鍋忕Щ浣嶇疆涓?

			pBuffer += secremain;		 // Pointer offset 鎸囬拡鍋忕Щ
			WriteAddr += secremain;		 // Write address offset 鍐欏湴鍧€鍋忕Щ
			NumByteToWrite -= secremain; // Bytes decreasing 瀛楄妭鏁伴€掑噺
			if (NumByteToWrite > 4096)
				secremain = 4096; // Unable to finish writing the next sector 涓嬩竴涓墖鍖鸿繕鏄啓涓嶅畬
			else
				secremain = NumByteToWrite; // The next sector can be completed 涓嬩竴涓墖鍖哄彲浠ュ啓瀹屼簡
		}
	};
}

// 鎿﹂櫎鏁翠釜鑺墖
// 鏁寸墖鎿﹂櫎鏃堕棿:W25X64:40s
// Erase the entire chip
// Whole chip erase time:W25X64:40s
void Spi_Flash_Erase_Chip(void)
{
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;								 // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_ChipErase); // Send chip erase command 鍙戦€佺墖鎿﹂櫎鍛戒护
	CS_HIGH;							 // Cancel Film Selection 鍙栨秷鐗囬€?
	Spi_Flash_Wait_Busy();				 // Waiting for chip erase to end 绛夊緟鑺墖鎿﹂櫎缁撴潫
}
// 鎿﹂櫎涓€涓墖鍖?
// Dst_Addr:鎵囧尯鍦板潃 0~511(0-2047) for w25x16
// 鎿﹂櫎涓€涓北鍖虹殑鏈€灏戞椂闂?150ms
// Erase a sector
// Dst_ Addr: Sector addresses 0-511 (0-2047) for w25x16
// Minimum time to erase a mountainous area: 150ms
void Spi_Flash_Erase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 4096;
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;										 // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_SectorErase);		 // Send sector erase command 鍙戦€佹墖鍖烘摝闄ゆ寚浠?
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 16)); // Send a 24bit address 鍙戦€?4bit鍦板潃
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 8));
	SPI_Read_Write_Byte((u8)Dst_Addr);
	CS_HIGH;			   // Cancel Film Selection 鍙栨秷鐗囬€?
	Spi_Flash_Wait_Busy(); // Waiting for erase to complete 绛夊緟鎿﹂櫎瀹屾垚
}

// 鎿﹂櫎涓€涓潡
// Dst_Addr:鍧楀湴鍧€ 0~128
// 鎿﹂櫎涓€涓北鍖虹殑鏈€灏戞椂闂?150ms
// Erase a block
// Dst_ Addr: Block addresses 0~128
// Minimum time to erase a mountainous area: 150ms
void Spi_Flash_BlockErase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 65536;
	Spi_Flash_Write_Enable(); // SET WEL
	Spi_Flash_Wait_Busy();
	CS_LOW;										 // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_BlockErase);		 // Send block erase instruction 鍙戦€佸潡鎿﹂櫎鎸囦护
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 16)); // Send a 24bit address 鍙戦€?4bit鍦板潃
	SPI_Read_Write_Byte((u8)((Dst_Addr) >> 8));
	SPI_Read_Write_Byte((u8)Dst_Addr);
	CS_HIGH;			   // Cancel Film Selection 鍙栨秷鐗囬€?
	Spi_Flash_Wait_Busy(); // Waiting for erase to complete 绛夊緟鎿﹂櫎瀹屾垚
}

// Waiting for idle 绛夊緟绌洪棽
void Spi_Flash_Wait_Busy(void)
{
	while ((Spi_Flash_Read_SR() & 0x01) == 0x01)
		; // Waiting for BUSY bit to clear 绛夊緟BUSY浣嶆竻绌?
}
// Entering power down mode 杩涘叆鎺夌數妯″紡
void Spi_Flash_Power_Down(void)
{
	CS_LOW;								 // Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_PowerDown); // Send power down command 鍙戦€佹帀鐢靛懡浠?
	CS_HIGH;							 // Cancel Film Selection 鍙栨秷鐗囬€?
	HAL_Delay(1);
}
// WAKEUP 鍞ら啋
void Spi_Flash_WAKEUP(void)
{
	CS_LOW;										// Enabling device 浣胯兘鍣ㄤ欢
	SPI_Read_Write_Byte(W25X_ReleasePowerDown); // send W25X_PowerDown command 0xAB
	CS_HIGH;									// Cancel Film Selection 鍙栨秷鐗囬€?
	HAL_Delay(1);
}
