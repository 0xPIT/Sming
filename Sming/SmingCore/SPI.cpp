/****
 * Sming Framework Project - Open Source framework for high efficiency native ESP8266 development.
 * Created 2015 by Skurydin Alexey
 * http://github.com/anakod/Sming
 * All files of the Sming Core are provided under the LGPL v3 license.
 ****/

// SPIClass code based on https://github.com/Perfer/esp8266_ili9341

#include "../SmingCore/SPI.h"
#include "../SmingCore/Digital.h"

#define SPI_FLASH_CMD(i)                            (REG_SPI_BASE(i)  + 0x0)
#define SPI_FLASH_READ (BIT(31))
#define SPI_FLASH_WREN (BIT(30))
#define SPI_FLASH_WRDI (BIT(29))
#define SPI_FLASH_RDID (BIT(28))
#define SPI_FLASH_RDSR (BIT(27))
#define SPI_FLASH_WRSR (BIT(26))
#define SPI_FLASH_PP (BIT(25))
#define SPI_FLASH_SE (BIT(24))
#define SPI_FLASH_BE (BIT(23))
#define SPI_FLASH_CE (BIT(22))
#define SPI_FLASH_DP (BIT(21))
#define SPI_FLASH_RES (BIT(20))
#define SPI_FLASH_HPM (BIT(19))
#define SPI_FLASH_USR (BIT(18))

#define SPI_FLASH_ADDR(i)                           (REG_SPI_BASE(i)  + 0x4)

#define SPI_FLASH_CTRL(i)                           (REG_SPI_BASE(i)  + 0x8)
#define SPI_WR_BIT_ODER (BIT(26))
#define SPI_RD_BIT_ODER (BIT(25))
#define SPI_QIO_MODE (BIT(24))
#define SPI_DIO_MODE (BIT(23))
#define SPI_TWO_BYTE_STATUS_EN (BIT(22))
#define SPI_WP_REG (BIT(21))
#define SPI_QOUT_MODE (BIT(20))
#define SPI_SHARE_BUS (BIT(19))
#define SPI_HOLD_MODE (BIT(18))
#define SPI_ENABLE_AHB (BIT(17))
#define SPI_SST_AAI (BIT(16))
#define SPI_RESANDRES (BIT(15))
#define SPI_DOUT_MODE (BIT(14))
#define SPI_FASTRD_MODE (BIT(13))

#define SPI_FLASH_CTRL1(i)                          (REG_SPI_BASE (i) + 0xC)
#define SPI_T_CSH 0x0000000F
#define SPI_T_CSH_S 28
#define SPI_T_RES 0x00000FFF
#define SPI_T_RES_S 16
#define SPI_BUS_TIMER_LIMIT 0x0000FFFF
#define SPI_BUS_TIMER_LIMIT_S 0

#define SPI_FLASH_STATUS(i)                         (REG_SPI_BASE(i)  + 0x10)
#define SPI_STATUS_EXT 0x000000FF
#define SPI_STATUS_EXT_S 24
#define SPI_WB_MODE 0x000000FF
#define SPI_WB_MODE_S 16
#define SPI_FLASH_STATUS_PRO_FLAG (BIT(7))
#define SPI_FLASH_TOP_BOT_PRO_FLAG (BIT(5))
#define SPI_FLASH_BP2 (BIT(4))
#define SPI_FLASH_BP1 (BIT(3))
#define SPI_FLASH_BP0 (BIT(2))
#define SPI_FLASH_WRENABLE_FLAG (BIT(1))
#define SPI_FLASH_BUSY_FLAG (BIT(0))

#define SPI_FLASH_CTRL2(i)                          (REG_SPI_BASE(i)  + 0x14)
#define SPI_CS_DELAY_NUM 0x0000000F
#define SPI_CS_DELAY_NUM_S 28
#define SPI_CS_DELAY_MODE 0x00000003
#define SPI_CS_DELAY_MODE_S 26
#define SPI_MOSI_DELAY_NUM 0x00000007
#define SPI_MOSI_DELAY_NUM_S 23
#define SPI_MOSI_DELAY_MODE 0x00000003
#define SPI_MOSI_DELAY_MODE_S 21
#define SPI_MISO_DELAY_NUM 0x00000007
#define SPI_MISO_DELAY_NUM_S 18
#define SPI_MISO_DELAY_MODE 0x00000003
#define SPI_MISO_DELAY_MODE_S 16
#define SPI_CK_OUT_HIGH_MODE 0x0000000F
#define SPI_CK_OUT_HIGH_MODE_S 12
#define SPI_CK_OUT_LOW_MODE 0x0000000F
#define SPI_CK_OUT_LOW_MODE_S 8
#define SPI_HOLD_TIME 0x0000000F
#define SPI_HOLD_TIME_S 4
#define SPI_SETUP_TIME 0x0000000F
#define SPI_SETUP_TIME_S 0

#define SPI_FLASH_CLOCK(i)                          (REG_SPI_BASE(i)  + 0x18)
#define SPI_CLK_EQU_SYSCLK (BIT(31))
#define SPI_CLKDIV_PRE 0x00001FFF
#define SPI_CLKDIV_PRE_S 18
#define SPI_CLKCNT_N 0x0000003F
#define SPI_CLKCNT_N_S 12
#define SPI_CLKCNT_H 0x0000003F
#define SPI_CLKCNT_H_S 6
#define SPI_CLKCNT_L 0x0000003F
#define SPI_CLKCNT_L_S 0

#define SPI_FLASH_USER(i)                           (REG_SPI_BASE(i)  + 0x1C)
#define SPI_USR_COMMAND (BIT(31))
#define SPI_FLASH_USR_ADDR (BIT(30))
#define SPI_FLASH_USR_DUMMY (BIT(29))
#define SPI_FLASH_USR_DIN (BIT(28))
#define SPI_FLASH_DOUT (BIT(27))
#define SPI_USR_DUMMY_IDLE (BIT(26))
#define SPI_USR_DOUT_HIGHPART (BIT(25))
#define SPI_USR_DIN_HIGHPART (BIT(24))
#define SPI_USR_PREP_HOLD (BIT(23))
#define SPI_USR_CMD_HOLD (BIT(22))
#define SPI_USR_ADDR_HOLD (BIT(21))
#define SPI_USR_DUMMY_HOLD (BIT(20))
#define SPI_USR_DIN_HOLD (BIT(19))
#define SPI_USR_DOUT_HOLD (BIT(18))
#define SPI_USR_HOLD_POL (BIT(17))
#define SPI_SIO (BIT(16))
#define SPI_FWRITE_QIO (BIT(15))
#define SPI_FWRITE_DIO (BIT(14))
#define SPI_FWRITE_QUAD (BIT(13))
#define SPI_FWRITE_DUAL (BIT(12))
#define SPI_WR_BYTE_ORDER (BIT(11))
#define SPI_RD_BYTE_ORDER (BIT(10))
#define SPI_AHB_ENDIAN_MODE 0x00000003
#define SPI_AHB_ENDIAN_MODE_S 8
#define SPI_CK_OUT_EDGE (BIT(7))
#define SPI_CK_I_EDGE (BIT(6))
#define SPI_CS_SETUP (BIT(5))
#define SPI_CS_HOLD (BIT(4))
#define SPI_AHB_USR_COMMAND (BIT(3))
#define SPI_AHB_USR_COMMAND_4BYTE (BIT(1))
#define SPI_DOUTDIN (BIT(0))

#define SPI_FLASH_USER1(i)                          (REG_SPI_BASE(i) + 0x20)
#define SPI_USR_ADDR_BITLEN 0x0000003F
#define SPI_USR_ADDR_BITLEN_S 26
#define SPI_USR_OUT_BITLEN 0x000001FF
#define SPI_USR_OUT_BITLEN_S 17
#define SPI_USR_DIN_BITLEN 0x000001FF
#define SPI_USR_DIN_BITLEN_S 8
#define SPI_USR_DUMMY_CYCLELEN 0x000000FF
#define SPI_USR_DUMMY_CYCLELEN_S 0

#define SPI_FLASH_USER2(i)                          (REG_SPI_BASE(i)  + 0x24)
#define SPI_USR_COMMAND_BITLEN 0x0000000F
#define SPI_USR_COMMAND_BITLEN_S 28
#define SPI_USR_COMMAND_VALUE 0x0000FFFF
#define SPI_USR_COMMAND_VALUE_S 0

#define SPI_FLASH_USER3(i)                          (REG_SPI_BASE(i)  + 0x28)
#define SPI_FLASH_PIN(i)                            (REG_SPI_BASE(i)  + 0x2C)
#define SPI_FLASH_SLAVE(i)                          (REG_SPI_BASE(i)  + 0x30)
#define SPI_SYNC_RESET (BIT(31))
#define SPI_SLAVE_MODE (BIT(30))
#define SPI_SLV_WR_RD_BUF_EN (BIT(29))
#define SPI_SLV_WR_RD_STA_EN (BIT(28))
#define SPI_SLV_CMD_DEFINE (BIT(27))
#define SPI_TRANS_CNT 0x0000000F
#define SPI_TRANS_CNT_S 23
#define SPI_SLV_LAST_STATE 0x00000007
#define SPI_SLV_LAST_STATE_S 20
#define SPI_SLV_LAST_COMMAND 0x00000007
#define SPI_SLV_LAST_COMMAND_S 17
#define SPI_CS_I_MODE 0x00000003
#define SPI_CS_I_MODE_S 10
#define SPI_INT_EN 0x0000001F
#define SPI_INT_EN_S 5
#define SPI_TRANS_DONE (BIT(4))
#define SPI_SLV_WR_STA_DONE (BIT(3))
#define SPI_SLV_RD_STA_DONE (BIT(2))
#define SPI_SLV_WR_BUF_DONE (BIT(1))
#define SPI_SLV_RD_BUF_DONE (BIT(0))

#define SPI_FLASH_SLAVE1(i)                         (REG_SPI_BASE(i)  + 0x34)
#define SPI_SLV_STATUS_BITLEN 0x0000001F
#define SPI_SLV_STATUS_BITLEN_S 27
#define SPI_SLV_STATUS_FAST_EN (BIT(26))
#define SPI_SLV_STATUS_READBACK (BIT(25))
#define SPI_SLV_BUF_BITLEN 0x000001FF
#define SPI_SLV_BUF_BITLEN_S 16
#define SPI_SLV_RD_ADDR_BITLEN 0x0000003F
#define SPI_SLV_RD_ADDR_BITLEN_S 10
#define SPI_SLV_WR_ADDR_BITLEN 0x0000003F
#define SPI_SLV_WR_ADDR_BITLEN_S 4
#define SPI_SLV_WRSTA_DUMMY_EN (BIT(3))
#define SPI_SLV_RDSTA_DUMMY_EN (BIT(2))
#define SPI_SLV_WRBUF_DUMMY_EN (BIT(1))
#define SPI_SLV_RDBUF_DUMMY_EN (BIT(0))

#define SPI_FLASH_SLAVE2(i)                         (REG_SPI_BASE(i)  + 0x38)
#define SPI_SLV_WRBUF_DUMMY_CYCLELEN 0x000000FF
#define SPI_SLV_WRBUF_DUMMY_CYCLELEN_S 24
#define SPI_SLV_RDBUF_DUMMY_CYCLELEN 0x000000FF
#define SPI_SLV_RDBUF_DUMMY_CYCLELEN_S 16
#define SPI_SLV_WRSTA_DUMMY_CYCLELEN 0x000000FF
#define SPI_SLV_WRSTA_DUMMY_CYCLELEN_S 8
#define SPI_SLV_RDSTA_DUMMY_CYCLELEN 0x000000FF
#define SPI_SLV_RDSTA_DUMMY_CYCLELEN_S 0

#define SPI_FLASH_SLAVE3(i)                         (REG_SPI_BASE(i)  + 0x3C)
#define SPI_SLV_WRSTA_CMD_VALUE 0x000000FF
#define SPI_SLV_WRSTA_CMD_VALUE_S 24
#define SPI_SLV_RDSTA_CMD_VALUE 0x000000FF
#define SPI_SLV_RDSTA_CMD_VALUE_S 16
#define SPI_SLV_WRBUF_CMD_VALUE 0x000000FF
#define SPI_SLV_WRBUF_CMD_VALUE_S 8
#define SPI_SLV_RDBUF_CMD_VALUE 0x000000FF
#define SPI_SLV_RDBUF_CMD_VALUE_S 0

#define SPI_FLASH_C0(i) 							(REG_SPI_BASE(i) +0x40)
#define SPI_FLASH_C1(i) 							(REG_SPI_BASE(i) +0x44)
#define SPI_FLASH_C2(i) 							(REG_SPI_BASE(i) +0x48)
#define SPI_FLASH_C3(i) 							(REG_SPI_BASE(i) +0x4C)
#define SPI_FLASH_C4(i) 							(REG_SPI_BASE(i) +0x50)
#define SPI_FLASH_C5(i) 							(REG_SPI_BASE(i) +0x54)
#define SPI_FLASH_C6(i) 							(REG_SPI_BASE(i) +0x58)
#define SPI_FLASH_C7(i) 							(REG_SPI_BASE(i) +0x5C)

#define SPI_FLASH_EXT0(i)                           (REG_SPI_BASE(i)  + 0xF0)
#define SPI_T_PP_ENA (BIT(31))
#define SPI_T_PP_SHIFT 0x0000000F
#define SPI_T_PP_SHIFT_S 16
#define SPI_T_PP_TIME 0x00000FFF
#define SPI_T_PP_TIME_S 0

#define SPI_FLASH_EXT1(i)                          (REG_SPI_BASE(i)  + 0xF4)
#define SPI_T_ERASE_ENA (BIT(31))
#define SPI_T_ERASE_SHIFT 0x0000000F
#define SPI_T_ERASE_SHIFT_S 16
#define SPI_T_ERASE_TIME 0x00000FFF
#define SPI_T_ERASE_TIME_S 0

#define SPI_FLASH_EXT2(i)                           (REG_SPI_BASE(i)  + 0xF8)
#define SPI_ST 0x00000007
#define SPI_ST_S 0

#define SPI_FLASH_EXT3(i)                           (REG_SPI_BASE(i)  + 0xFC)
#define SPI_INT_HOLD_ENA 0x00000003
#define SPI_INT_HOLD_ENA_S 0


SPIClass SPI(SPI_ID_HSPI);

SPIClass::SPIClass(uint8_t spiID) : id(spiID)
{
	// Only SPI_HSPI tested on hardware for now!
}

void SPIClass::begin()
{
	//bit9 of PERIPHS_IO_MUX should be cleared when HSPI clock doesn't equal CPU clock
	//bit8 of PERIPHS_IO_MUX should be cleared when SPI clock doesn't equal CPU clock
	if (id == SPI_ID_MAIN)
	{
		WRITE_PERI_REG(PERIPHS_IO_MUX, 0x005); //clear bit9,and bit8

		PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CLK_U, 1); //configure io to spi mode
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_CMD_U, 1); //configure io to spi mode
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA0_U, 1); //configure io to spi mode
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_SD_DATA1_U, 1); //configure io to spi mode
	}
	else if (id == SPI_ID_HSPI)
	{
		WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105); //clear bit9

		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2); // HSPIQ MISO == GPIO12
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2); // HSPID MOSI == GPIO13
		PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2); // CLK		 == GPIO14
	}
	else
		SYSTEM_ERROR("UNSUPPORTED SPI id: %d", id);


	// SPI clock = CPU clock / 10 / 4
	// time length HIGHT level = (CPU clock / 10 / 2) ^ -1,
	// time length LOW level = (CPU clock / 10 / 2) ^ -1
	WRITE_PERI_REG(SPI_FLASH_CLOCK(id),
	   ((1 & SPI_CLKDIV_PRE) << SPI_CLKDIV_PRE_S) |
	   ((3 & SPI_CLKCNT_N) << SPI_CLKCNT_N_S) |
	   ((1 & SPI_CLKCNT_H) << SPI_CLKCNT_H_S) |
	   ((3 & SPI_CLKCNT_L) << SPI_CLKCNT_L_S));
}

void SPIClass::writeData(uint8_t * data, uint8_t numberByte)
{
	uint8_t i = 0;
	uint8_t shift = 0;
	uint32_t *buffer = (uint32_t *)SPI_FLASH_C0(id);
	for (i = 0; i < numberByte; ++i)
	{
		if (shift >= 32)
		{
			shift = 0;
			buffer += 1;
		}
		*buffer &= ~( 0xFFUL << shift );
		*buffer |= ((uint32_t)data[i]) << shift;
		shift += 8;
	}
}

void SPIClass::readData(uint8_t * data, uint8_t numberByte)
{
	uint8_t i = 0;
	uint8_t shift = 0;
	uint32_t buffer = 0;

	for (i = 0; i < numberByte; ++i)
	{
		if ( (i % 4 == 0) )
		{
			buffer = ( *( (uint32_t *)SPI_FLASH_C0(id) + i/4 ) );
			shift = 0;
		}
		data[i] = (buffer >> shift) & 0xFF;
		shift += 8;
	}
}

void SPIClass::end()
{
}

void SPIClass::transfer(uint8_t * data, uint8_t count)
{
	uint32_t regvalue = 0;
	uint16_t numberBit = 0;

	while (READ_PERI_REG(SPI_FLASH_CMD(id))&SPI_FLASH_USR);

	regvalue |=  SPI_FLASH_DOUT | SPI_DOUTDIN | SPI_CK_I_EDGE;
	regvalue &= ~(BIT2 | SPI_FLASH_USR_ADDR | SPI_FLASH_USR_DUMMY | SPI_FLASH_USR_DIN | SPI_USR_COMMAND); //clear bit 2 see example IoT_Demo
	WRITE_PERI_REG(SPI_FLASH_USER(id), regvalue);

	numberBit = count * 8 - 1;

	WRITE_PERI_REG(SPI_FLASH_USER1(id),
			( (numberBit & SPI_USR_OUT_BITLEN) << SPI_USR_OUT_BITLEN_S ) |
			( (numberBit & SPI_USR_DIN_BITLEN) << SPI_USR_DIN_BITLEN_S ) );

	writeData(data, count);

	SET_PERI_REG_MASK(SPI_FLASH_CMD(id), SPI_FLASH_USR);   // send

	while (READ_PERI_REG(SPI_FLASH_CMD(id)) & SPI_FLASH_USR);

	readData(data, count);
}

byte SPIClass::transfer(uint8_t data)
{
	transfer(&data, 1);
	return data;
}

/*void SPIClass::setBitOrder(uint8_t bitOrder)
{
	order = bitOrder;
}*/
