#ifndef TLV320AIC3101_H
#define TLV320AIC3101_H

#include "miosix.h"

class TLV320AIC3101
{
public:
	/**
	 * \return a Singleton instance of the class so that only 1 instance is possible
	 */
	static TLV320AIC3101& instance();

    /**
	 * Setup the Codec so that is ready to work:
     * 1. Set codec registers via I2C
     * 2. Allocate memory for sound data manipulation
     * 3. Set I2S registers inside the STM32
     * 4. Enable I2S interrupts
	 */
    void TLV320AIC3101::setup();

    void I2C_Read(unsigned char regAddress, char data);

private:
    TLV320AIC3101(); //constructor
    TLV320AIC3101(const TLV320AIC3101&);
	TLV320AIC3101& operator= (const TLV320AIC3101&);

    mutable miosix::Mutex mutex; // a mutex in order to protect codec setup

    uint8_t I2C_address = 0b00110000; // codec I2C address

    static void I2C_Send(unsigned char regAddress, char data);

};

#endif