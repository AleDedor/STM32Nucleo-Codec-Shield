#include <cstdio>
#include "miosix.h"
#include "util/software_i2c.h"

using namespace std;
using namespace miosix;

//Gpios for I2C
typedef Gpio<GPIOB_BASE,7> sda;
typedef Gpio<GPIOB_BASE,6> scl;
typedef SoftwareI2C<sda,scl> i2c;

//Gpios for I2S
typedef Gpio<GPIOC_BASE,6> mclk;
typedef Gpio<GPIOB_BASE,10> sclk;
typedef Gpio<GPIOB_BASE,12> lrclk;
typedef Gpio<GPIOC_BASE,2> sdin;
typedef Gpio<GPIOC_BASE,3> sdout;

static const int bufferSize = 256;
static BufferQueue<unsigned short, bufferSize> *bq1;
static BufferQueue<unsigned short, bufferSize> *bq2;

//------------------------Codec instance, Singleton pattern ------------------------------------
/* Singleton is a creational design pattern that lets you ensure that a class has only one instance, 
 * while providing a global access point to this instance. 
 */
TLV320AIC3101& TLV320AIC3101::instance()
{
	static TLV320AIC3101 singleton;
	return singleton;
}


//---------------------------I2C Codec communication function---------------------------------------
static void TLV320AIC3101::I2C_Send(unsigned char regAddress, char data){
    i2c::sendStart();
    i2c::send(TLV320AIC3101::I2C_address); // codec I2C address
    i2c::send(regAddress);
    i2c::send(data);
    i2c::sendStop();
}

static void TLV320AIC3101::I2C_Read(unsigned char regAddress, char data){
    i2c::sendStart();
    i2c::send(TLV320AIC3101::I2C_address); // codec I2C address
    i2c::send(regAddress);
    i2c::recvWithNack();
    i2c::sendStop();
}


//------------------------Codec initialization and setup function------------------------------------
void TLV320AIC3101::setup()
{
    Lock<Mutex> l(mutex);

    //allocation of memory for 2 buffer queues
    bq1 = new BufferQueue<unsigned short, bufferSize>();
    bq2 = new BufferQueue<unsigned short, bufferSize>();

    {
        FastInterruptDisableLock dLock;
        //Enable DMA1 and I2S2 clocks on AHB and APB buses
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        RCC_SYNC();
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        RCC_SYNC();

        //GPIO configuration
        i2c::init();
        mclk::mode(Mode::ALTERNATE);
        mclk::alternateFunction(6);
        sclk::mode(Mode::ALTERNATE);
        sclk::alternateFunction(6);
        sdin::mode(Mode::ALTERNATE);
        sdin::alternateFunction(6);
        lrclk::mode(Mode::ALTERNATE);
        lrclk::alternateFunction(6);

        //enabling PLL for I2S and starting clock
        //PLLI2SR=3, PLLI2SN=258 see datasheet pag.595
        RCC->PLLI2SCFGR=(3<<28) | (258<<6);
        RCC->CR |= RCC_CR_PLLI2SON;
    }

    //wait for PLL to lock
    while((RCC->CR & RCC_CR_PLLI2SRDY)==0);

    //Send TLV320AIC3101 configuration registers with I2C
    TLV320AIC3101::I2C_Send(0x01,0b10000000);
    TLV320AIC3101::I2C_Send(0x02,0b00000000);
    TLV320AIC3101::I2C_Send(0x01,0b00010001);
    TLV320AIC3101::I2C_Send(0x07,0b00001010);
    TLV320AIC3101::I2C_Send(0x0C,0b00000000);
    TLV320AIC3101::I2C_Send(0x0E,0b10001000);
    TLV320AIC3101::I2C_Send(0x0F,0b00001100);
    TLV320AIC3101::I2C_Send(0x10,0b00001100);
    TLV320AIC3101::I2C_Send(0x11,0b00001111);
    TLV320AIC3101::I2C_Send(0x12,0b11110000);
    TLV320AIC3101::I2C_Send(0x13,0b01111111);
    TLV320AIC3101::I2C_Send(0x16,0b01111111);
    TLV320AIC3101::I2C_Send(0x25,0b11010000);
    TLV320AIC3101::I2C_Send(0x26,0b00001100);
    TLV320AIC3101::I2C_Send(0x28,0b10000010);
    TLV320AIC3101::I2C_Send(0x29,0b10100000);
    TLV320AIC3101::I2C_Send(0x2a,0b00010100);
    TLV320AIC3101::I2C_Send(0x2b,0b00000000);
    TLV320AIC3101::I2C_Send(0x2c,0b00000000);
    TLV320AIC3101::I2C_Send(0x33,0b10011111);
    TLV320AIC3101::I2C_Send(0x3f,0b00000000);
    TLV320AIC3101::I2C_Send(0x41,0b10011111);
    TLV320AIC3101::I2C_Send(0x65,0b00000001);
    TLV320AIC3101::I2C_Send(0x66,0b00000010);

    //enable DMA on I2S, RX mode
    SPI2->CR2=SPI_CR2_RXDMAEN;
    //I2S prescaler register, see pag.595
    SPI2->I2SPR=  SPI_I2SPR_MCKOE //mclk enable
                | SPI_I2SPR_ODD   //ODD = 1
                | 3;              //I2SDIV = 3
    
    SPI2->I2SCFGR=SPI_I2SCFGR_I2SMOD    //I2S mode selected
                | SPI_I2SCFGR_I2SE      //I2S Enabled
                | SPI_I2SCFGR_I2SCFG;   //Master receive

    //set DMA interrupt priority
    NVIC_SetPriority(DMA1_Stream3_IRQn,2); //high prio
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);     //enable interrupt

}
//----------------------------------------------------------------------------------------------------
