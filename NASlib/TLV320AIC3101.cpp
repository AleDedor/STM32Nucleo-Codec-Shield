#include "miosix/kernel/scheduler/scheduler.h"
#include "util/software_i2c.h"
#include "TLV320AIC3101.h"

#define DMA_SxCR_CHSEL_3    ((uint32_t)0x06000000)

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
//typedef Gpio<GPIOC_BASE,3> sdout;

const int bufferSize = 128;
unsigned int size = 128;
static Thread *waiting;
BufferQueue<unsigned short, bufferSize> *bq;
//BufferQueue<unsigned short, bufferSize, 3> bq; for version with also TX

//uint16_t bufferw[256] = {0};
bool DMA_working = false;

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

//---------------------------I2C Codec communication functions---------------------------------------
bool TLV320AIC3101::I2C_Send(unsigned char regAddress, char data)
{
    bool commWorked = false;
    delayMs(1);
    I2C::sendStart();
    I2C::send(TLV320AIC3101::I2C_address);
    I2C::send(regAddress);
    commWorked = I2C::send(data);
    I2C::sendStop();
    return commWorked;
}

unsigned char TLV320AIC3101::I2C_Receive(unsigned char regAddress)
{
    unsigned char data = 0;
    I2C::sendStart();
    I2C::send(TLV320AIC3101::I2C_address);
    I2C::send(regAddress);
    I2C::sendRepeatedStart();
    I2C::send(TLV320AIC3101::I2C_address | 0b00000001); //read bit (LSB) to 1   
    data = I2C::recvWithNack();
    I2C::sendStop();
    return data;
}

//-------------------------------IRQ handler function------------------------------------------------
void __attribute__((weak)) DMA1_Stream3_IRQHandler()
{
    saveContext();
	asm volatile("bl _Z17I2SdmaHandlerImplv"); 
	restoreContext();
    TLV320AIC3101::IRQ_entrato = true;
}

void __attribute__((used)) I2SdmaHandlerImpl() //actual function implementation
{
    //clear DMA1 interrupt flags
    DMA1->LIFCR=DMA_LIFCR_CTCIF3  | //clear transfer complete flag 
                DMA_LIFCR_CTEIF3  | //clear transfer error flag
                DMA_LIFCR_CDMEIF3 | //clear direct mode error flag
                DMA_LIFCR_CHTIF3  | 
                DMA_LIFCR_CFEIF3;   //clear fifo error interrupt flag

    DMA_working = false;
    //mark the buffer as readable
    bq->bufferFilled(size);
    waiting->IRQwakeup();
    if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
}


//--------------------------------get a readable buffer----------------------------------------------
const unsigned short * TLV320AIC3101::getReadableBuff()
{
    FastInterruptDisableLock dLock;
    const unsigned short *readableBuff;

    //try to find the readable buffer among the 2
    while(bq->tryGetReadableBuffer(readableBuff,size)==false){

        //sleep until a buffer is marked as readable
        waiting->IRQwait();
		{
			FastInterruptEnableLock eLock(dLock);
			Thread::yield();
		} 
    }
    return readableBuff;
}

//--------------------Process the bq which is not read or written------------------------------------

bool TLV320AIC3101::test(){
        //Start DMA
        DMA1_Stream3->CR = 0; //reset configuration register to 0
        DMA1_Stream3->PAR = reinterpret_cast<unsigned int>(&I2S2ext->DR); //pheripheral address set to SPI2
        DMA1_Stream3->M0AR = reinterpret_cast<unsigned int>(bufferw);   //set buffer as destination
        DMA1_Stream3->NDTR = 256;                               //size of buffer to fulfill
        DMA1_Stream3->CR = //DMA_SxCR_CHSEL_3 | //dma1 stream 3 channel 3
                        DMA_SxCR_PL_1    | //High priority DMA stream
                        DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
                        DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
                        DMA_SxCR_MINC    | //Increment RAM pointer after each transfer
                        DMA_SxCR_TEIE    | //Interrupt on transfer error
                        DMA_SxCR_DMEIE   | //Interrupt on direct mode error
                        DMA_SxCR_TCIE    | //Interrupt on completion
                        DMA_SxCR_EN;       //Start the DMA
                        //DMA_SxCR_CIRC  | //circular mode
        TLV320AIC3101::IRQ_entrato = false;

    return true;
}

void TLV320AIC3101::ok()
{
    bq->bufferEmptied();
}

//--------------------------Function for starting the I2S DMA RX-----------------------------------------
bool startRxDMA() //needed to make sure that the lock reaches the scopes at the end of the I2S_startRX()
{ 
    
    unsigned short *buffer;

    if((bq->tryGetWritableBuffer(buffer) == false) || DMA_working){
        return false;
    }

    //Start DMA
    DMA1_Stream3->CR = 0; //reset configuration register to 0
    DMA1_Stream3->PAR = reinterpret_cast<unsigned int>(&I2S2ext->DR); //pheripheral address set to SPI2
    DMA1_Stream3->M0AR = reinterpret_cast<unsigned int>(buffer);   //set buffer as destination
    DMA1_Stream3->NDTR = size;                               //size of buffer to fulfill
    DMA1_Stream3->CR = DMA_SxCR_CHSEL_3 | //dma1 stream 3 channel 3
                       DMA_SxCR_PL_1    | //High priority DMA stream
                       DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
					   DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
				       DMA_SxCR_MINC    | //Increment RAM pointer after each transfer
                       //DMA_SxCR_TEIE    | //Interrupt on transfer error
                       DMA_SxCR_DMEIE   | //Interrupt on direct mode error
			           DMA_SxCR_TCIE    | //Interrupt on completion
			  	       DMA_SxCR_EN;       //Start the DMA
                       //DMA_SxCR_CIRC  | //circular mode
    DMA_working = true;
    return true;
}

bool TLV320AIC3101::I2S_startRx()
{
    bool startedDMA = false;
    {
    FastInterruptDisableLock dLock;
    startedDMA = startRxDMA();
    }
    return startedDMA;
}

//------------------------Codec initialization and setup method------------------------------------
void TLV320AIC3101::setup()
{
    Lock<Mutex> l(mutex);

    waiting=Thread::getCurrentThread();
    //allocation of memory for 2 buffer queues
    bq = new BufferQueue<unsigned short, bufferSize>();

    {
        FastInterruptDisableLock dLock;
        //Enable DMA1 and I2S2 clocks on AHB and APB buses
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
        RCC_SYNC();
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        RCC_SYNC();

        //GPIO configuration - alternate function #5 is associated with SPI2
        I2C::init();
        mclk::mode(Mode::ALTERNATE);
        mclk::alternateFunction(5);
        sclk::mode(Mode::ALTERNATE);
        sclk::alternateFunction(5);
        sdin::mode(Mode::ALTERNATE);
        sdin::alternateFunction(5);
        lrclk::mode(Mode::ALTERNATE);
        lrclk::alternateFunction(5);

        //enabling PLL for I2S and starting clock
        //PLLM = 16 (default), PLLI2SR = 3, PLLI2SN = 258 see datasheet pag.595
        //Actually by trying on the cubeIDE, these values distort the sound more than the one set on the olde project (? dunno why)
        RCC->PLLI2SCFGR=(3<<28) | (258<<6); //values suggested by datasheet
        //RCC->PLLI2SCFGR=(5<<28) | (123<<6); //values we found
        RCC->CR |= RCC_CR_PLLI2SON;
    }

    //wait for PLL to lock
    while((RCC->CR & RCC_CR_PLLI2SRDY)==0);

    //Send TLV320AIC3101 configuration registers with I2C
    delayMs(10);
    TLV320AIC3101::I2C_Send(0x01,0b10000000);
    TLV320AIC3101::I2C_Send(0x02,0b00000000);
    TLV320AIC3101::I2C_Send(0x03,0b00010001);
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
    TLV320AIC3101::I2C_Send(0x2A,0b00010100);
    TLV320AIC3101::I2C_Send(0x2B,0b00000000);
    TLV320AIC3101::I2C_Send(0x2C,0b00000000);
    TLV320AIC3101::I2C_Send(0x33,0b10011111);
    TLV320AIC3101::I2C_Send(0x3F,0b00000000);
    TLV320AIC3101::I2C_Send(0x41,0b10011111);
    TLV320AIC3101::I2C_Send(0x65,0b00000001);
    TLV320AIC3101::I2C_Send(0x66,0b00000010);

    //enable DMA on I2S, RX mode
    SPI2->CR2= SPI_CR2_RXDMAEN;
    //I2S prescaler register, see pag.595. fi2s = 16M*PLLI2SN/(PLLM*PLLI2SR)=86MHz -> fs=fi2s/[32*(2*I2SDIV+ODD)*8]=47991Hz
    SPI2->I2SPR=  SPI_I2SPR_MCKOE       //mclk enable
                | SPI_I2SPR_ODD         //ODD = 1
                | (uint32_t)0x00000003; //I2SDIV = 3
    
    SPI2->I2SCFGR= SPI_I2SCFGR_I2SMOD    //I2S mode selected
                 | SPI_I2SCFGR_I2SE      //I2S Enabled
                 | SPI_I2SCFGR_I2SCFG;   //Master receive

    //set DMA interrupt priority
    NVIC_SetPriority(DMA1_Stream3_IRQn,2);   //high prio
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);     //enable interrupt

    DMA1_Stream3->CR = 0; //reset configuration register to 0
}
