/*
 * ad4695.h
 *
 *  Created on: May 5, 2022
 *      Author: emre.ozdemir
 */

#ifndef INC_AD4695_H_
#define INC_AD4695_H_

#include "stm32h7xx_hal.h"
#include "spi.h"

#define CHIP_SELECT_0 		HAL_GPIO_WritePin(GPIOG, CS_0_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_0 	HAL_GPIO_WritePin(GPIOG, CS_0_TL_Pin, GPIO_PIN_SET);
#define CHIP_SELECT_1 		HAL_GPIO_WritePin(GPIOG, CS_1_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_1 	HAL_GPIO_WritePin(GPIOG, CS_1_TL_Pin, GPIO_PIN_SET);
#define CHIP_SELECT_2 		HAL_GPIO_WritePin(GPIOG, CS_2_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_2 	HAL_GPIO_WritePin(GPIOG, CS_2_TL_Pin, GPIO_PIN_SET);
#define CHIP_SELECT_3 		HAL_GPIO_WritePin(GPIOG, CS_3_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_3 	HAL_GPIO_WritePin(GPIOG, CS_3_TL_Pin, GPIO_PIN_SET);
#define CHIP_SELECT_4 		HAL_GPIO_WritePin(GPIOG, CS_4_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_4 	HAL_GPIO_WritePin(GPIOG, CS_4_TL_Pin, GPIO_PIN_SET);
#define CHIP_SELECT_5 		HAL_GPIO_WritePin(GPIOG, CS_5_TL_Pin, GPIO_PIN_RESET);
#define CHIP_DESELECT_5 	HAL_GPIO_WritePin(GPIOG, CS_5_TL_Pin, GPIO_PIN_SET);


#define AD4695_REG_SPI_CONFIG_A		0x000 // R/W
#define AD4695_REG_SPI_CONFIG_B		0x001 // R/W
#define AD4695_REG_DEVICE_TYPE		0x003 // R  read the 0x7 value(fixed)
//#define AD4695_REG_DEVICE_ID_L		0x004
//#define AD4695_REG_DEVICE_ID_H		0x005
#define AD4695_REG_SCRATCH_PAD		0x00A // R/W to test the SPI communications
#define AD4695_REG_VENDOR_L			0x00C // R just an ID 0x0456
#define AD4695_REG_VENDOR_H			0x00D // R just an ID
#define AD4695_REG_LOOP_MODE		0x00E // R/W
#define AD4695_REG_SPI_CONFIG_C		0x010 // R/W
#define AD4695_REG_SPI_STATUS		0x011 // R/W
#define AD4695_REG_STATUS			0x014 // R
#define AD4695_REG_ALERT_STATUS1	0x015 // R
#define AD4695_REG_ALERT_STATUS2	0x016 // R
#define AD4695_REG_ALERT_STATUS3	0x017 // R
#define AD4695_REG_ALERT_STATUS4	0x018 // R
#define AD4695_REG_CLAMP_STATUS1	0x01A // R
#define AD4695_REG_CLAMP_STATUS2	0x01B // R
#define AD4695_REG_SETUP			0x020 // R/W
#define AD4695_REG_REF_CTRL			0x021 // R/W
#define AD4695_REG_SEQ_CTRL			0x022 // R/W
#define AD4695_REG_AC_CTRL			0x023 // R/W
#define AD4695_REG_STD_SEQ_CONFIG	0x024 // R/W
#define AD4695_REG_GPIO_CTRL		0x026 // R/W
#define AD4695_REG_GP_MODE			0x027 // R/W
#define AD4695_REG_GPIO_STATE		0x028 // R/W
#define AD4695_REG_TEMP_CTRL		0x029 // R/W
#define AD4695_REG_CONFIG_IN(x)		((x & 0x0F) | 0x30)
#define AD4695_REG_UPPER_IN(x)		((x & 0x5E) | 0x40)
#define AD4695_REG_LOWER_IN(x)		((x & 0x7E) | 0x60)
#define AD4695_REG_HYST_IN(x)		((x & 0x9E) | 0x80)
#define AD4695_REG_OFFSET_IN(x)		((x & 0xBE) | 0xA0)
#define AD4695_REG_GAIN_IN(x)		((x & 0xDE) | 0xC0)
#define AD4695_REG_AS_SLOT(x)		((x & 0x7F) | 0x100)

#define AD4695_REG_REF_CTRL_MASK			(0x07 << 2 )
#define AD4695_REG_REF_CTRL_EN(x)			((x & 0x07) << 2)
/* 5-bit SDI Conversion Mode Commands */
#define AD4695_CMD_REG_CONFIG_MODE			(0x0A << 3)
#define AD4695_CMD_SEL_TEMP_SNSOR_CH		(0x0F << 3)
#define AD4695_CMD_CONFIG_CH_SEL(x)			((0x10 | (0x0F & x)) << 3)

/* AD4695_REG_SETUP */
#define AD4695_SETUP_IF_MODE_MASK			(0x01 << 2)
#define AD4695_SETUP_IF_MODE_CONV			(0x01 << 2)
#define AD4695_SETUP_IF_SDO_STATE			(0x01 << 6)
#define AD4695_SETUP_IF_SDO_STATE_MASK		(0x01 << 6)
#define AD4695_SETUP_CYC_CTRL_MASK			(0x01 << 1)
#define AD4695_SETUP_CYC_CTRL_SINGLE(x)		((x & 0x01) << 1)

/* AD4695_REG_GP_MODE */
#define AD4695_GP_MODE_BUSY_GP_EN_MASK		(0x01 << 1)
#define AD4695_GP_MODE_BUSY_GP_EN(x)		((x & 0x01) << 1)
#define AD4695_GP_MODE_BUSY_GP_SEL_MASK		(0x01 << 4)
#define AD4695_GP_MODE_BUSY_GP_SEL(x)		((x & 0x01) << 4)

/* AD4695_REG_SEQ_CTRL */
#define AD4695_SEQ_CTRL_STD_SEQ_EN_MASK		(0x01 << 7)
#define AD4695_SEQ_CTRL_STD_SEQ_EN(x)		((x & 0x01) << 7)
#define AD4695_SEQ_CTRL_NUM_SLOTS_AS_MASK	(0x7F << 0)
#define AD4695_SEQ_CTRL_NUM_SLOTS_AS(x)		((x & 0x7F) << 0)

/* AD4695_REG_TEMP_CTRL */
#define AD4695_REG_TEMP_CTRL_TEMP_EN_MASK	(0x01 << 0)
#define AD4695_REG_TEMP_CTRL_TEMP_EN(x)		((x & 0x01) << 0)

/* AD4695_REG_AS_SLOT */
#define AD4695_REG_AS_SLOT_INX(x)			((x & 0x0F) << 0)

/* AD4695_REG_SPI_CONFIG_C */
#define AD4695_REG_SPI_CONFIG_C_MB_STRICT_MASK		(0x01 << 5)
#define AD4695_REG_SPI_CONFIG_C_MB_STRICT(x)		((x & 0x01) << 5)

/* AD4695_REG_CONFIG_INn */
#define AD4695_REG_CONFIG_IN_OSR_MASK		(0x03 << 0)
#define AD4695_REG_CONFIG_IN_OSR(x)			((x & 0x03) << 0)
#define AD4695_REG_CONFIG_IN_HIZ_EN_MASK	(0x01 << 3)
#define AD4695_REG_CONFIG_IN_HIZ_EN(x)		((x & 0x01) << 3)
#define AD4695_REG_CONFIG_IN_PAIR_MASK		(0x03 << 4)
#define AD4695_REG_CONFIG_IN_PAIR(x)		((x & 0x03) << 4)
#define AD4695_REG_CONFIG_IN_MODE_MASK		(0x01 << 6)
#define AD4695_REG_CONFIG_IN_MODE(x)		((x & 0x01) << 6)
#define AD4695_REG_CONFIG_IN_TD_EN_MASK		(0x01 << 7)
#define AD4695_REG_CONFIG_IN_TD_EN(x)		((x & 0x01) << 7)
#define AD4695_CHANNEL(x)					(BIT(x) & 0xFFFF)
#define AD4695_CHANNEL_NO					16
#define AD4695_SLOTS_NO						0x80
#define AD4695_CHANNEL_TEMP					16

#define AD4695_STD_SEQ_FIRST_SIX_CHANNEL_EN 0x3F
#define AD4695_STD_SEQ_FIRST_FOUR_CHANNEL_EN 0x0F
#define AD4695_STD_SEQ_FIRST_CHANNEL_EN 	0x01
#define AD4695_STD_SEQ_F_SECOND_CHANNEL_EN  0x03
#define AD4695_STD_SEQ_4_5__CHANNEL_EN  0x18
#define AD4695_STD_SEQ_5_6__CHANNEL_EN  0x30

typedef enum{

	FALSE=0,
	TRUE=1,

}Bool;

enum ad4695_channel_sequencing {

	AD4695_single_cycle = 0,
	AD4695_two_cycle = 1,
	AD4695_standard_seq = 2,
	AD4695_advanced_seq = 3,
};

enum ad4695_ref {

	 R2V4_2V7 = 0,
	 R2V7_3V2 = 1,
	 R3V2_3V7 = 2,
	 R3V7_4V5 = 3,
	 R4V5_R5V1 = 4,
};

enum ad4695_busy_gpio_sel {

	AD4695_busy_gp0 = 0,
	AD4695_busy_gp3 = 1,
};


enum ad4695_reg_access {
	AD4695_BYTE_ACCESS = 0,
	AD4695_WORD_ACCESS = 1,
};


enum ad4695_osr_ratios { /* oversampling */

	AD4695_OSR_1 = 0,
	AD4695_OSR_4 = 1,
	AD4695_OSR_16 = 2,
	AD4695_OSR_64 = 3,
};

typedef struct{

	Bool StractPad;
	Bool RegisterAccessMode;
	Bool BusyState;
	Bool STD_Sq_Mode_OSR;
	Bool Set_STD_Mode;
	Bool SetRef;
	Bool STD_En_Channels;
	Bool EnterConservationMode;
	Bool SDO_State;
}AD4695_Init_Error_Struct;
AD4695_Init_Error_Struct AD4695_Init_Error;

uint8_t WBuf[3];
uint8_t WBuf2[3];
uint8_t RBuf[3];
uint8_t TestData;

uint8_t TestRegisterAccessModeCTR;
uint8_t TestBusyState;
uint8_t TestSTD_Sq_Mode_OSR;
uint8_t TestSet_STD_Mode;
uint8_t TestSetRef;
uint8_t TestSTD_En_Channels;
uint8_t TestEnterConservationMode;
uint8_t TestSDO_State;
uint32_t l;

void AD4695_Test(void);
void SPI_Write_Reg_AD4695(uint16_t reg_addr,uint8_t reg_data);
void SPI_Read_Reg_AD4695(uint16_t reg_addr,uint8_t *reg_data);
void SPI_Read_Mask(uint16_t reg_addr, uint8_t mask, uint8_t *data);
void SPI_Write_Mask(uint16_t reg_addr, uint8_t mask, uint8_t data);
void AD5695_Register_Access_Mode( enum ad4695_reg_access access);
void AD4695_Init();
void AD4695_Set_Busy_State(/*Enum ad4695_busy_gpio_sel gp_sel*/);
void AD4695_Standart_Seq_MODEandOSR(enum ad4695_osr_ratios ratio);
void AD4695_Enter_Conservation_Mode(void);
void AD4695_Exit_Conservation_Mode(void);
void AD4695_Standart_MODE_SET(void);
void AD4695_STD_SEQ_EN_Channels(uint16_t reg_addr);
void AllConfigForAD4695();




#endif /* INC_AD4695_H_ */
