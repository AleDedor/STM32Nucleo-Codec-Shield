#include "main.h"
#include "stm32f4xx_it.h"
#include "TLV320AIC3101_Codec.h"

HAL_StatusTypeDef Codec_Init(Codec *codec, I2C_HandleTypeDef *I2Chandle){

	//assign I2C handler
	codec->I2Chandle = I2Chandle;

	HAL_StatusTypeDef status;

	//software reset
	status = Codec_WriteRegister(codec, 0x01, 0b10000000);
	HAL_Delay(10);

	//with PLL disabled: We want fs=48kHz, from datasheet: fs = MCLK / (Q*128) since MCLK=96k*256, for fs=48kHz -> Q=4
	status = Codec_WriteRegister(codec, 0x03, 0b00100000);

	//fs=48kHz, ADC-DAC dual rate ON for 96kHz, left-DAC data path plays left-channel input data, right-DAC data path plays right-channel input data
	status = Codec_WriteRegister(codec, 0x07, 0b01101010);

	//high-power outputs ac-coupled driver configuration
	status = Codec_WriteRegister(codec, 0x0e, 0b10000000);

	//un-mute left ADC PGA
	status = Codec_WriteRegister(codec, 0x0f, 0b00000000);

	//un-mute rigth ADC PGA
	status = Codec_WriteRegister(codec, 0x10, 0b00000000);

	//DA FINIRE
	return status;
}

HAL_StatusTypeDef Codec_WriteRegister(Codec *codec, uint8_t reg_addr, uint8_t val){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, &val, sizeof(val), 100);
	return status;
}


HAL_StatusTypeDef Codec_ReadRegister(Codec *codec, uint8_t reg_addr, uint8_t *val){
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, val, sizeof(val), 100);
	return status;
}
