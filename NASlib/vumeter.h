// include guards
#ifndef VUMETER_IMPL_H
#define VUMETER_IMPL_H

#include "miosix.h"
#include "interfaces/gpio.h"

using namespace miosix;

// a namespace for all the related constants
namespace VUCONST{
    const uint8_t NUM_LEDS = 5;
    const uint8_t NUM_BIT_ADC = 16;
    const int MAX_LEVEL = 31;
    const int MIN_LEVEL = 0;
}

class Vumeter // fixed @6 leds
{
public:
    /**  Main idea of the VU-Meter: must be able to display the sampled
     *   sound value in a log scale. Let's use the constructor to initialize
     *   the passed Led pins (set them as outputs), define some simple functions
     *   to turn on / off all the leds (debug purposes) and finally display the value.
     */

    /**
     * Constructor, initializes the vumeter pins @ OUTPUT
     * \param Rled1 a Gpio class specifying the GPIO connected to the Rled1
     * \param Rled2 a Gpio class specifying the GPIO connected to the Rled2
     * \param Yled1 a Gpio class specifying the GPIO connected to the Yled1
     * \param Yled2 a Gpio class specifying the GPIO connected to the Yled2
     * \param Gled1 a Gpio class specifying the GPIO connected to the Gled1
     * \param Gled2 a Gpio class specifying the GPIO connected to the Gled2
     */
    Vumeter(GpioPin Rled1, GpioPin Rled2, GpioPin Yled1, GpioPin Yled2, GpioPin Gled1);
    
    /**
     * turn off all the leds
     */
    void clear();

    /**
     * turn on all the leds
     */
    void setHigh();

    /**
     * Display the sound value with the VU-Meter
     * \param sound_val a 16-bit integer coming from Codec ADC -> we use an int (32-bit) value
     * because max ADC #bits is 32. 
     * The value is displayed in a logarithmic scale.
     */
    void showVal(unsigned int sound_val);

private:

    // all the pins needed for the class
    GpioPin R1;
    GpioPin R2;
    GpioPin Y1;
    GpioPin Y2;
    GpioPin G1; 

};

// here the definition of the contructor
Vumeter::Vumeter(GpioPin Rled1, GpioPin Rled2, GpioPin Yled1, GpioPin Yled2, GpioPin Gled1) 
: R1(Rled1), R2(Rled2), Y1(Yled1), Y2(Yled2), G1(Gled1) //init list, assign passed parameters to class variables
{
    R1.mode(Mode::OUTPUT);
    R2.mode(Mode::OUTPUT);
    Y1.mode(Mode::OUTPUT);
    Y2.mode(Mode::OUTPUT);
    G1.mode(Mode::OUTPUT);
}

//Reset all LEDs
void Vumeter::clear(){
    R1.low();
    R2.low();
    Y1.low();
    Y2.low();
    G1.low();
}

//Set all LEDs
void Vumeter::setHigh(){
    R1.high();
    R2.high();
    Y1.high();
    Y2.high();
    G1.high();
}

// Display the sound value
void Vumeter::showVal(unsigned int sound_val){

    unsigned int threshold = 0;
    unsigned int value = 0;
    
    for(int i=0; i<VUCONST::NUM_LEDS; i++){
        if((sound_val>>(VUCONST::NUM_BIT_ADC - VUCONST::NUM_LEDS)) > threshold){
            value = (value << 1) | 1;
        }
        threshold = (threshold << 1) | 1;
    }

    //clear();
    if(value & 0x00000001) G1.high();
    else                   G1.low();

    if(value & 0x00000002) Y2.high();
    else                   Y2.low();

    if(value & 0x00000004) Y1.high();
    else                   Y1.low();

    if(value & 0x00000008) R2.high();
    else                   R2.low();

    if(value & 0x00000010) R1.high();
    else                   R1.low();

}
#endif //VUMETER_IMPL_H