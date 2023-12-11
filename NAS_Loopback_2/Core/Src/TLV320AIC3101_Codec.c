#include "main.h"
#include "stm32f4xx_it.h"
#include "TLV320AIC3101_Codec.h"

HAL_StatusTypeDef Codec_Init(Codec *codec, I2C_HandleTypeDef *I2Chandle){

	//assign I2C handler
	codec->I2Chandle = I2Chandle;

	HAL_StatusTypeDef status;

	//Wait power on reset for the CODEC
	HAL_Delay(RESET_TIME);

	uint8_t reg_add_val[]={
			0x02,	0x00, //(2) codec ADC/DAC sample rate
			//0x03, 0b00100000, //(3) with PLL disabled: We want fs=48kHz, from datasheet: fs = MCLK / (Q*128) since MCLK=96k*256, for fs=48kHz -> Q=4
			0x04,	0x04,
			0x05,	0x00,
			0x06,	0x00,
			0x07,	0x0A, // 48 kHz fs, Left-DAC plays left channel input data, right-DAC plays right channel input data
			0x08,	0x00,
			0x09,	0x00,
			0x0A,	0x00,
			0x0B,	0x01,
			0x0C,	0xF0, // Enable high pass filter for Left ADC and Right ADC
			0x0D,	0x00,
			0x0E,	0x80, // AC coupled high power output, pseudo differential output
			0x0F,	0x00, // Left ADC PGA gain of 0 dB
			0x10,	0x3F, // Right ADC PGA gain of ~30 dB
			0x11,	0xFF, // Disconnect MIC2L/R from left ADC
			0x12,	0xFF, // Disconnect MIC2L/R from right ADC
			0x13,	0x78, // Left ADC powered down. LINE1L not connected to left-ADC PGA. We only have input on the right ADC
			0x14,   0x78, // Left Channel Analog Inputs to CM connection
			0x15,	0xF8, // MIC1RP to Left-ADC Control Register. MIC1RP differential mode. LINE1R is not connected to left ADC.
			0x16,	0x84, // MIC1RP to Right-ADC Control Register. Right ADC powered up, differential microphone mode, -0 DB volume
			0x17,   0x78, // Right Channel Analog Inputs to CM Connection
			0x18,	0xF8, // LINE1L not connected to right ADC
			0x19,	0x00, // MIC bias selection. Disable bias = 0x00, AVDD is 0xC0, 0x40 is 2.0V, 0x80 is 2.5V
			0x1A,	0x00, // Left-AGC control register
			0x1B,	0x7F,
			0x1C,	0x00,
			0x1D,	0x00,
			0x1E,	0x7F,
			0x1F,	0x00,
			0x20,	0x00,
			0x21,	0x00,
			0x22,	0x00,
			0x23,	0x00,
			0x24,	0x00,
			0x25,	0xE0, // Left DAC powered up, Right DAC powered up, HPLCOM is independent single-ended output
			0x26,	0x14, // HPRCOM is independent single-ended output, Short-Circuit Protection is enabled
			0x28,	0x00,
			0x29,	0xA0, // Left and right DAC output path selects path 2
			0x2A,	0x38, // Output driver pop reduction
			0x2B,	0x10, // Left-DAC digital volume control register
			0x2C,	0x10, // Right-DAC Digital Volume Control Register
			0x2E,	0x00, // PGA_R is NOT routed to HPLCOM
			0x2F,	0x00, // DAC_L1 is NOT routed to HPROUT
			0x31,	0x00,
			0x32,	0x00,
			0x33,	0x0D, // HPLOUT output level control to 0 dB.
			0x35,	0x00,
			0x36,	0x00,
			0x38,	0x00,
			0x39,	0x00,
			0x3A,	0x0C, // HPLCOM Output level control register
			0x3C,	0x00,
			0x3D,	0x00,
			0x3F,	0x00,
			0x40,	0x00,
			0x41,	0x0D, // HPROUT output level control to 0 dB.
			0x43,	0x00, // PGA_L is NOT routed to HPROUT
			0x44,	0x00,
			0x46,	0x00,
			0x47,	0x00,
			0x48,	0x0C, // HPRCOM Output level control register
			0x51,	0x00,
			0x52,	0x00,
			0x54,	0x00,
			0x55,	0x00,
			0x56,	0x00,
			0x58,	0x00,
			0x59,	0x00,
			0x5B,	0x00,
			0x5C,	0x00,
			0x5D,	0x02,
			0x5E,	0x00,
			0x5F,	0x00,
			0x65,	0x01,
			0x66,	0x02, // MCLK
			0x67,	0x00,
			0x68,	0x00,
			0x69,	0x00,
			0x6A,	0x00,
			0x6B,	0x00,
			0x6C,	0x00,
			0x6D,	0x00
	};

	uint8_t len_array_reg = sizeof(reg_add_val)/sizeof(reg_add_val[0]);

	status = Codec_WriteRegister(codec, reg_add_val, len_array_reg);
	/*
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

	//(12) Input HP filter disabled, DAC filters bypassed
	status = Codec_WriteRegister(codec, 0x0c, 0b00000000);

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

	//(19) Turn ON LEFT ADC - PGA soft stepping disabled (?)
	status = Codec_WriteRegister(codec, 0x13, 0b01111111);

	//(22) Turn ON RIGHT ADC - PGA soft stepping disabled (?)
	status = Codec_WriteRegister(codec, 0x16, 0b01111111);

	//(37) Turn ON RIGHT and LEFT DACs, HPLCOM set as independent single-ended output
	status = Codec_WriteRegister(codec, 0x25, 0b11100000);

	//(38) HPRCOM set as independent single-ended output, short circuit protection activated with curr limit
	status = Codec_WriteRegister(codec, 0x26, 0b00010100);

	//(40) output common-mode voltage = 1.65 V - output soft stepping disabled
	status = Codec_WriteRegister(codec, 0x28, 0b10000010);

	//(41) set DAC path, DAC_L2 to left high power, DAC_R2 to right high power, right-DAC volume follows left-DAC volume
	status = Codec_WriteRegister(codec, 0x29, 0b10100000);

	//(42) Pop-reduction register, band gap reference
	status = Codec_WriteRegister(codec, 0x2a, 0b00000010);

	//(43) un-mute left DAC
	status = Codec_WriteRegister(codec, 0x2b, 0b00000000);

	//(44) un-mute right DAC
	status = Codec_WriteRegister(codec, 0x2c, 0b00000000);

	//(46) PGA_L to HPLOUT OFF, volume control 0dB
	status = Codec_WriteRegister(codec, 0x2e, 0b00000000);

	//(51) un-mute HPLOUT, high impedance when powered down, HPLOUT fully powered
	status = Codec_WriteRegister(codec, 0x33, 0b00001001);

	//(58) un-mute HPLCOM, high impedance when powered down, HPLCOM fully powered
	status = Codec_WriteRegister(codec, 0x3A, 0b00001101);

	//(63) PGA_R to HPROUT OFF, volume control 0dB
	status = Codec_WriteRegister(codec, 0x3f, 0b00000000);

	//(65) un-mute HPROUT, high impedance when powered down, HPROUT fully powered
	status = Codec_WriteRegister(codec, 0x41, 0b00001001);

	//(72) un-mute HPRCOM, high impedance when powered down, HPRCOM fully powered
	status = Codec_WriteRegister(codec, 0x48, 0b00001101);

	//(101) CLK source selection, CLKDIV_OUT
	status = Codec_WriteRegister(codec, 0x65, 0b00000001);

	//(101) CLK source selection, MCLK
	status = Codec_WriteRegister(codec, 0x66, 0b00000001);



	//(63)
	//status = Codec_WriteRegister(codec, 0x3f, 0b10000000);
	*/
	return status;
}

HAL_StatusTypeDef Codec_WriteRegister(Codec *codec, uint8_t* reg_add_val, uint8_t len_array_reg){
	uint8_t data[2] = {0,0};
	int16_t counter = 0;
	HAL_StatusTypeDef status;
		while(counter < len_array_reg){
			data[0]= reg_add_val[counter];
			data[1]= reg_add_val[counter+1];
			counter += 2;
			status = HAL_I2C_Master_Transmit(codec->I2Chandle, CODEC_I2C_ADDR, data, 2, 100);
			HAL_Delay(10);
		}
	return status;
}
/*
HAL_StatusTypeDef Codec_WriteRegister(Codec *codec, uint8_t reg_addr, uint8_t val){
	uint8_t data[2]={reg_addr,val};
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(codec->I2Chandle, CODEC_I2C_ADDR, data, 2, 100);
	//HAL_StatusTypeDef status = HAL_I2C_Mem_Write(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, &val, sizeof(val), 100);
	return status;
}
*/

HAL_StatusTypeDef Codec_ReadRegister(Codec *codec, uint8_t reg_addr, uint8_t *val){
	//HAL_I2C_Master_Transmit(codec->I2Chandle, CODEC_I2C_ADDR, &reg_addr, sizeof(reg_addr), 100);
	//HAL_StatusTypeDef status = HAL_I2C_Master_Receive(codec->I2Chandle, CODEC_I2C_ADDR, val, I2C_MEMADD_SIZE, 100);
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(codec->I2Chandle, CODEC_I2C_ADDR, reg_addr, I2C_MEMADD_SIZE, val, sizeof(val), 100);
	return status;
}
