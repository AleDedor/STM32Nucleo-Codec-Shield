#ifndef SRC_TLV320AIC3101_CODEC_H_
#define SRC_TLV320AIC3101_CODEC_H_

#define CODEC_I2C_ADDR 0b00110000
#define I2C_MEMADD_SIZE 1
#define RESET_TIME 50

//magari si può fare il define dei led qua
// e aggiungere i led nello struct che definisce il codec
typedef struct{
	I2C_HandleTypeDef *I2Chandle;
}Codec;

HAL_StatusTypeDef Codec_Init(Codec *codec, I2C_HandleTypeDef *I2Chandle);
HAL_StatusTypeDef Codec_WriteRegister(Codec *codec, uint8_t reg_addr, uint8_t val);
HAL_StatusTypeDef Codec_ReadRegister(Codec *codec, uint8_t reg_addr, uint8_t *val);

#endif
