/*
 * ad4695.c
 *
 *  Created on: May 5, 2022
 *      Author: emre.ozdemir
 */

#include "ad4695.h"


uint8_t TEST_DATA = 0x1f; //can be changed


void AD4695_Test(){ //test the SPI com

	while(TEST_DATA != TestData){

		SPI_Write_Reg_AD4695(AD4695_REG_SCRATCH_PAD, TEST_DATA);
		SPI_Read_Reg_AD4695(AD4695_REG_SCRATCH_PAD, &TestData);

		HAL_Delay(100);
	}
}

void SPI_Write_Reg_AD4695(uint16_t reg_addr,uint8_t reg_data){

	WBuf[0] = ((reg_addr >> 8) & 0x7F);
	WBuf[1] = 0xFF & reg_addr;
	WBuf[2] = reg_data;

	CHIP_SELECT_5;
	HAL_SPI_Transmit(&hspi4, WBuf,3,HAL_MAX_DELAY);
	CHIP_DESELECT_5;
}

void SPI_Read_Reg_AD4695(uint16_t reg_addr,uint8_t *reg_data){

	WBuf2[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	WBuf2[1] = 0xFF & reg_addr;
	WBuf2[2] = 0xFF;

	CHIP_SELECT_5;
	HAL_SPI_TransmitReceive(&hspi4, WBuf2,RBuf, 3, HAL_MAX_DELAY);
	CHIP_DESELECT_5;

	*reg_data = RBuf[2];
}

void SPI_Read_Mask(uint16_t reg_addr, uint8_t mask, uint8_t *data){

	uint8_t Reg_Data[3];
	SPI_Read_Reg_AD4695(reg_addr,Reg_Data);
	*data = (Reg_Data[2] & mask);
}

void SPI_Write_Mask(uint16_t reg_addr, uint8_t mask, uint8_t data){

	uint8_t Reg_Data;
	SPI_Read_Reg_AD4695(reg_addr,&Reg_Data);
	Reg_Data &= ~mask;
	Reg_Data |= data;
	SPI_Write_Reg_AD4695(reg_addr,Reg_Data);
}

void AD5695_Register_Access_Mode( enum ad4695_reg_access access){

	SPI_Write_Mask(AD4695_REG_SPI_CONFIG_C,AD4695_REG_SPI_CONFIG_C_MB_STRICT_MASK,AD4695_REG_SPI_CONFIG_C_MB_STRICT(access));
	SPI_Read_Reg_AD4695(AD4695_REG_SPI_CONFIG_C,&TestRegisterAccessModeCTR);

	if(TestRegisterAccessModeCTR != WBuf[2]){

		AD4695_Init_Error.RegisterAccessMode = TRUE;


	}
}

void AD4695_Set_Busy_State(/*enum ad4695_busy_gpio_sel gp_sel*/){

	SPI_Write_Mask(AD4695_REG_GP_MODE, AD4695_GP_MODE_BUSY_GP_EN_MASK, AD4695_GP_MODE_BUSY_GP_EN(1));
	//SPI_Write_Mask(AD4695_REG_GP_MODE, AD4695_GP_MODE_BUSY_GP_SEL_MASK, AD4695_GP_MODE_BUSY_GP_SEL(gp_sel));
	SPI_Read_Reg_AD4695(AD4695_REG_GP_MODE,&TestBusyState);

	if(TestBusyState != WBuf[2]){

		AD4695_Init_Error.BusyState = TRUE;

	}

}

void AD4695_Standart_Seq_MODEandOSR(enum ad4695_osr_ratios ratio){/*over sampling ratio in standard sequencer mode*/

	SPI_Write_Mask(AD4695_REG_CONFIG_IN(0), AD4695_REG_CONFIG_IN_OSR_MASK, AD4695_REG_CONFIG_IN_OSR(ratio));
	SPI_Read_Reg_AD4695(AD4695_REG_CONFIG_IN(0),&TestSTD_Sq_Mode_OSR);
	if(TestSTD_Sq_Mode_OSR != WBuf[2]){

		AD4695_Init_Error.STD_Sq_Mode_OSR = TRUE;


	}
}

void AD4695_Standart_MODE_SET(){/*Standard Sequencer Enable Bit*/

	SPI_Write_Mask(AD4695_REG_SEQ_CTRL, AD4695_SEQ_CTRL_STD_SEQ_EN_MASK, AD4695_SEQ_CTRL_STD_SEQ_EN(1));
	SPI_Read_Reg_AD4695(AD4695_REG_SEQ_CTRL,&TestSet_STD_Mode);
	if(TestSet_STD_Mode != WBuf[2]){

		AD4695_Init_Error.Set_STD_Mode = TRUE;

	}
}

void AD4695_RefControl(enum ad4695_ref REF){/*Reference Input Range Control  0x0: 2.4 V ≤ VREF ≤ 2.75 V.
														 	 	 	 	 	 0x1: 2.75 V < VREF ≤ 3.25 V.
														 	 	 	 	 	 0x2: 3.25 V < VREF ≤ 3.75 V.
														 	 	 	 	 	 0x3: 3.75 V < VREF ≤ 4.50 V.
														 	 	 	 	 	 0x4: 4.5 V < VREF ≤ 5.10 V.*/
	//SPI_Write_Mask(AD4695_REG_REF_CTRL, AD4695_REG_REF_CTRL_MASK, AD4695_REG_REF_CTRL_EN(REF));
	SPI_Write_Reg_AD4695(AD4695_REG_REF_CTRL, 0x06);
	SPI_Read_Reg_AD4695(AD4695_REG_REF_CTRL,&TestSetRef);
	if(TestSetRef != WBuf[2]){

		AD4695_Init_Error.SetRef = TRUE;

	}
}

void AD4695_STD_SEQ_EN_Channels(uint16_t reg_addr){

	//uint8_t Reg_Data;
	SPI_Write_Reg_AD4695(reg_addr,/*AD4695_STD_SEQ_FIRST_FOUR_CHANNEL_EN*//*AD4695_STD_SEQ_5_6__CHANNEL_EN*//*AD4695_STD_SEQ_4_5__CHANNEL_EN*//*AD4695_STD_SEQ_F_SECOND_CHANNEL_EN */AD4695_STD_SEQ_FIRST_CHANNEL_EN/*AD4695_STD_SEQ_FIRST_SIX_CHANNEL_EN*/);
	SPI_Read_Reg_AD4695(reg_addr,&TestSTD_En_Channels);

	if(TestSTD_En_Channels != WBuf[2]){

		AD4695_Init_Error.STD_En_Channels = TRUE;


				}
}

void AD4695_Autocycle_MODE_SET(uint16_t reg_addr){

	uint8_t Reg_Data;
	SPI_Write_Reg_AD4695(reg_addr, 0x01);

	SPI_Read_Reg_AD4695(reg_addr,&Reg_Data);;
}

void AD4695_Init(){

	//This field disables the CRC
	AD5695_Register_Access_Mode(AD4695_BYTE_ACCESS); //individual bytes in multibyte registers are read from or written to in individual data phases
	AD4695_Set_Busy_State();
	AD4695_Standart_Seq_MODEandOSR(AD4695_OSR_1); //16 bit not 17,18 or 19
	AD4695_Standart_MODE_SET(); //Standard mode is set
	AD4695_RefControl(R2V7_3V2); /*Setting the reference voltage */
	AD4695_STD_SEQ_EN_Channels(AD4695_REG_STD_SEQ_CONFIG); /*Enables selected channels*/
	AD4695_Enter_Conservation_Mode(); /*Enters conservation mode*/
	HAL_Delay(100);


}

void AD4695_Enter_Conservation_Mode(){/*Enter conversion mode*/

	SPI_Write_Mask(AD4695_REG_SETUP,AD4695_SETUP_IF_SDO_STATE_MASK,AD4695_SETUP_IF_SDO_STATE );

	SPI_Read_Reg_AD4695(AD4695_REG_SETUP,&TestSDO_State);
		if(TestSDO_State != WBuf[2]){

			AD4695_Init_Error.SDO_State = TRUE;

		}

	SPI_Write_Mask(AD4695_REG_SETUP, AD4695_SETUP_IF_MODE_MASK, AD4695_SETUP_IF_MODE_CONV);
}

void AD4695_Exit_Conservation_Mode(){ /*To pass from conservaiton mode to register configuration mode */

	uint8_t commands_data[1];
	commands_data[0] = AD4695_CMD_REG_CONFIG_MODE ;
	CHIP_SELECT_5;
	HAL_SPI_Transmit(&hspi4,commands_data,1,HAL_MAX_DELAY);
	CHIP_DESELECT_5;
}
