#include <cstdio>
#include "miosix.h"
#include "util/software_i2c.h"
#include "miosix/kernel/scheduler/scheduler.h"
#include "TLV320AIC3101.h"

using namespace std;
using namespace miosix;

//Gpios for I2C
typedef Gpio<GPIOB_BASE,7> sda;
typedef Gpio<GPIOB_BASE,6> scl;
typedef SoftwareI2C<sda,scl> I2C;

//Gpios for I2S
typedef Gpio<GPIOC_BASE,6> mclk;
typedef Gpio<GPIOB_BASE,10> sclk;
typedef Gpio<GPIOB_BASE,12> lrclk;
typedef Gpio<GPIOC_BASE,2> sdin;
typedef Gpio<GPIOC_BASE,3> sdout;

static const int bufferSize = 128;
unsigned int size;
static Thread *waiting;
BufferQueue<unsigned short, bufferSize> *bq;
//BufferQueue<unsigned short, bufferSize, 3> bq; for version with also TX

//------------------------Codec instance, Singleton pattern ------------------------------------
/* Singleton is a creational design pattern that lets you ensure that a class has only one instance, 
 * while providing a global access point to this instance. 
 */
TLV320AIC3101& TLV320AIC3101::instance()
{
	static TLV320AIC3101 singleton;
	return singleton;
}

TLV320AIC3101::TLV320AIC3101(){}

//---------------------------I2C Codec communication function----------------------------------------
static void TLV320AIC3101::I2C_Send(unsigned char regAddress, char data)
{
    I2C::sendStart();
    I2C::send(TLV320AIC3101::I2C_address);
    I2C::send(regAddress);
    I2C::send(data);
    I2C::sendStop();
}

unsigned char TLV320AIC3101::I2C_Receive(unsigned char regAddress){
    unsigned char data = 0;
    I2C::sendStart();
    I2C::send(TLV320AIC3101::I2C_address);
    I2C::send(regAddress);
    data = I2C::recvWithNack();
    I2C::sendStop();
    return data;
}

//---------------------------Try to get a writable buffer--------------------------------------------
const unsigned short *getReadableBuff()
{
    FastInterruptDisableLock dLock;
    const unsigned short *readableBuff;

    //try to find the writable buffer among the 2
    while(bq->tryGetReadableBuffer(readableBuff,size)==false){

        //sleep until a buffer is marked as writable
        waiting->IRQwait();
		{
			FastInterruptEnableLock eLock(dLock);
			Thread::yield();
		} 
    }
    return readableBuff;
}

//-------------------------------IRQ handler function------------------------------------------------
void __attribute__((naked)) DMA1_Stream3_IRQHandler()
{
    saveContext();
	asm volatile("bl _Z17I2SdmaHandlerImplv");
	restoreContext();
}

void __attribute__((used)) I2SdmaHandlerImpl() //actual function implementation
{
    //clear DMA1 interrupt flags
	DMA1->HIFCR=DMA_HIFCR_CTCIF5  | //clear transfer complete flag 
                DMA_HIFCR_CTEIF5  | //clear transfer error flag
                DMA_HIFCR_CDMEIF5 | //clear direct mode error flag
                DMA_HIFCR_CFEIF5;   //clear fifo error interrupt flag

	bq->bufferFilled(size);
	waiting->IRQwakeup();
	if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
		Scheduler::IRQfindNextThread();
}

//--------------------Process the bq which is not read or written------------------------------------

/*void TLV320::processBuffer(){


}*/

//--------------------------Function for starting the I2S DMA RX-----------------------------------------
static void startRxDMA(){ //needed to make sure that the lock reaches the scopes at the end of the startRX()
    
    unsigned short *buffer;

    if(bq->tryGetWritableBuffer(buffer) == false){
        return;
    }

    //Start DMA
    DMA1_Stream3->CR=0; //reset configuration register to 0
    DMA1_Stream3->PAR = reinterpret_cast<unsigned int>(&SPI2->DR); //pheripheral address set to SPI2
    DMA1_Stream3->M0AR = reinterpret_cast<unsigned int>(buffer);   //set buffer as destination
    DMA1_Stream3->NDTR = bufferSize;                               //size of buffer to fulfill
    DMA1_Stream3->CR= DMA_SxCR_PL_1    | //High priority DMA stream
                      DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
					  DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
				      DMA_SxCR_MINC    | //Increment RAM pointer after each transfer
			          DMA_SxCR_TCIE    | //Interrupt on completion
			  	      DMA_SxCR_EN;       //Start the DMA
}

void TLV320AIC3101::I2S_startRx()
{
    FastInterruptDisableLock dLock;
    startRxDMA();
}

//------------------------Codec initialization and setup method------------------------------------
void TLV320AIC3101::setup()
{
    Lock<Mutex> l(mutex);

    //allocation of memory for 2 buffer queues
    bq = new BufferQueue<unsigned short, bufferSize>();

    {
        FastInterruptDisableLock dLock;
        //Enable DMA1 and I2S2 clocks on AHB and APB buses
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
        RCC_SYNC();
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        RCC_SYNC();

        //GPIO configuration
        I2C::init();
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
    delayMs(10);
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
