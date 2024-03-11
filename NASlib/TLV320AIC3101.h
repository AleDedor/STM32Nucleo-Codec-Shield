#include "miosix.h"

#ifndef TLV320AIC3101_H
#define TLV320AIC3101_H

class TLV320AIC3101
{
public:
	/**
	 * \return a Singleton instance of the class so that only 1 instance is possible
     * static method that returns the only instance of the class
	 */
	static TLV320AIC3101& instance();

    /**
	 * Setup the Codec so that is ready to work:
     * 1. Set codec registers via I2C
     * 2. Allocate memory for sound data manipulation
     * 3. Set I2S registers inside the STM32
     * 4. Enable I2S interrupts
	 */
    void setup();

    /* ??? */
    void ok();

    /**
	 * Start I2S communication (RX) with DMA
	 */
    static bool I2S_startRx();


    /**
	 * Start I2S communication (TX) with DMA
	 */
    static bool I2S_startTx();

    static const unsigned short * getReadableBuff();

    /**
	 * \return a byte coming from the Codec specified reg. address
	 */
    unsigned char I2C_Receive(unsigned char regAddress);

    bool I2C_Send(unsigned char regAddress, char data);


private:

    /** Singleton is accomplished by:
     *  Declaring all constructors of the class to be private, which prevents it from being instantiated by other objects
     *  Providing a static method that returns a reference to the instance
     *  The instance is usually stored as a private static variable; 
     *  the instance is created when the variable is initialized, at some point before when the static method is first called.
     */

    TLV320AIC3101(); //constructor is private
    TLV320AIC3101(const TLV320AIC3101&);
	TLV320AIC3101& operator= (const TLV320AIC3101&);

    /** C++ has the ability to provide the operators with a special meaning for a data type, this ability is known as operator overloading. 
     *  Operator overloading is a compile-time polymorphism. For example, we can overload an operator ‘+’ in a class like String 
     *  so that we can concatenate two strings by just using +. 
     *  operator= () defines new meaning for "=" sign when used with the class
     */

    mutable miosix::Mutex mutex; // a mutex in order to protect codec setup

    /* place here inside the class or with global scope? Logically better inside classe so only class can use it*/
    uint8_t I2C_address = 0b00110000; // codec I2C address

};

#endif