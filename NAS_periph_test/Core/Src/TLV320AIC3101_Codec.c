#include "main.h"
#include "stm32f4xx_it.h"
#include "TLV320AIC3101_Codec.h"

HAL_StatusTypeDef Codec_Init(Codec *codec, I2C_HandleTypeDef *I2Chandle){

	//assign I2C handler
	codec->I2Chandle = I2Chandle;

	HAL_StatusTypeDef status;

	//Wait power on reset for the CODEC
	HAL_Delay(RESET_TIME);
	//(1) software reset
	status = Codec_WriteRegister(codec, 0x01, 0b10000000);
	HAL_Delay(10);

	//(2) codec ADC/DAC sample rate
	status = Codec_WriteRegister(codec, 0x02, 0b00000000);

	//(3) with PLL disabled: We want fs=48kHz, from datasheet: fs = MCLK / (Q*128) since MCLK=96k*256, for fs=48kHz -> Q=4
	//status = Codec_WriteRegister(codec, 0x03, 0b00100000);
	status = Codec_WriteRegister(codec, 0x03, 0b00100000);

	//(7) fs=48kHz, ADC-DAC dual rate ON for 96kHz, left-DAC data path plays left-channel input data, right-DAC data path plays right-channel input data
	status = Codec_WriteRegister(codec, 0x07, 0b01101010);
	//status = Codec_WriteRegister(codec, 0x07, 0x08); //PHIL'S VERSION

	//(14) high-power outputs ac-coupled driver configuration
	status = Codec_WriteRegister(codec, 0x0e, 0b10000000);

	//(15) un-mute left ADC PGA
	status = Codec_WriteRegister(codec, 0x0f, 0b00000000);

	//(16) un-mute right ADC PGA
	status = Codec_WriteRegister(codec, 0x10, 0b00000000);

	//LET'S USE JACK CH2 FOR THE TESTS!
	//(17) MIC2L connected to LEFT ADC (0dB), MIC2R not connected to LEFT ADC
	status = Codec_WriteRegister(codec, 0x11, 0b00001111);
	//(18) MIC2R connected to RIGTH ADC (0dB), MIC2L not connected to RIGHT ADC
	status = Codec_WriteRegister(codec, 0x12, 0b11110000);

	//(19) Turn ON LEFT ADC
	status = Codec_WriteRegister(codec, 0x13, 0b01111100);

	//(22) Turn ON RIGHT ADC
	status = Codec_WriteRegister(codec, 0x16, 0b01111100);

	//(37) Turn ON RIGHT and LEFT DACs, HPLCOM set as independent single-ended output
	status = Codec_WriteRegister(codec, 0x25, 0b11000000);

	//(38) HPRCOM set as independent single-ended output, short circuit protection activated
	status = Codec_WriteRegister(codec, 0x26, 0b00000110);

	//(40) output common-mode voltage = 1.65 V
	status = Codec_WriteRegister(codec, 0x28, 0b10000000);

	//(41) set DAC path, DAC_L2 to left high power, DAC_R2 to right high power, right-DAC volume follows left-DAC volume
	status = Codec_WriteRegister(codec, 0x29, 0b10100000);

	//(43) un-mute left DAC
	status = Codec_WriteRegister(codec, 0x2B, 0b00000000);

	//(44) un-mute right DAC
	status = Codec_WriteRegister(codec, 0x2C, 0b00000000);

	//(51) un-mute HPLOUT, high impedance when powered down, HPLOUT fully powered
	status = Codec_WriteRegister(codec, 0x33, 0b00001101);

	//(65) un-mute HPROUT, high impedance when powered down, HPROUT fully powered
	status = Codec_WriteRegister(codec, 0x41, 0b00001111);

	//(101) CLK source selection, CLKDIV_OUT
	status = Codec_WriteRegister(codec, 0x65, 0b00000001);

	//(46)
	//status = Codec_WriteRegister(codec, 0x2e, 0b10000000);

	//(63)
	//status = Codec_WriteRegister(codec, 0x3f, 0b10000000);

	return status;
}

HAL_StatusTypeDef Codec_WriteRegister(Codec *codec, uint8_t reg_addr, uint8_t val){
	uint8_t data[2]={reg_addr,val};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(codec->I2Chandle, CODEC_I2C_ADDR, data, 2, 100);
	//HAL_StatusTypeDef status = HAL_I2C_Mem_Write(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, &val, sizeof(val), 100);
	return status;
}


HAL_StatusTypeDef Codec_ReadRegister(Codec *codec, uint8_t reg_addr, uint8_t *val){
	//HAL_I2C_Master_Transmit(codec->I2Chandle, CODEC_I2C_ADDR, &reg_addr, sizeof(reg_addr), 100);
	//HAL_StatusTypeDef status = HAL_I2C_Master_Receive(codec->I2Chandle, CODEC_I2C_ADDR, val, I2C_MEMADD_SIZE, 100);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, val, sizeof(val), 100);
	return status;
}
