#include "miosix/kernel/scheduler/scheduler.h"
#include "util/software_i2c.h"
#include "TLV320AIC3101.h"
#include <thread>
#include "e20/e20.h"
#include <cstring>

#define DMA_SxCR_CHSEL_3    ((uint32_t)0x06000000)

using namespace std;
using namespace miosix;

FixedEventQueue<100,12> queue;

typedef Gpio<GPIOC_BASE,9> mco2;

//Gpios for I2C
typedef Gpio<GPIOB_BASE,7> sda;
typedef Gpio<GPIOB_BASE,6> scl;
typedef SoftwareI2C<sda,scl> I2C;

//Gpios for I2S
typedef Gpio<GPIOC_BASE,6> mclk;
typedef Gpio<GPIOB_BASE,10> sclk;
typedef Gpio<GPIOB_BASE,12> lrclk;  // word select
typedef Gpio<GPIOC_BASE,2> sdin;    // I2S_ext -> MISO
typedef Gpio<GPIOC_BASE,3> sdout;   // I2S -> MOSI

const int bufferSize = 128;
unsigned int size = 128;
static Thread *waiting;
BufferQueue<unsigned short, bufferSize> *bq;


//BufferQueue<unsigned short, bufferSize, 3> bq; for version with also TX

//------------------------Codec instance, Singleton pattern ------------------------------------
/* Singleton is a creational design pattern that lets you ensure that a class has only one instance, 
 * while providing a global access point to this instance. 
 */
TLV320AIC3101& TLV320AIC3101::instance(){
	static TLV320AIC3101 singleton;
	return singleton;
}

TLV320AIC3101::TLV320AIC3101(){}

//---------------------------I2C Codec communication functions---------------------------------------
bool TLV320AIC3101::I2C_Send(unsigned char regAddress, char data){
    bool commWorked = false;
    delayMs(1);
    I2C::sendStart();
    I2C::send(TLV320AIC3101::I2C_address);
    I2C::send(regAddress);
    commWorked = I2C::send(data);
    I2C::sendStop();
    return commWorked;
}

unsigned char TLV320AIC3101::I2C_Receive(unsigned char regAddress){
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

//--------------------------------get a readable buffer----------------------------------------------
const unsigned short * TLV320AIC3101::getReadableBuff(){
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

void TLV320AIC3101::ok(){
    bq->bufferEmptied();
}

//--------------------------Function for starting the I2S DMA RX-----------------------------------------
//nested function call needed to make sure that the lock reaches the scopes at the end of the I2S_startRX()
bool startRxDMA(){ 

    unsigned short *buffer_rx;
    if((bq->tryGetWritableBuffer(buffer_rx) == false)){
        return false;
    }
    
    //Start DMA1 with I2S2_ext, peripheral to memory
    DMA1_Stream3->PAR = reinterpret_cast<unsigned int>(&I2S2ext->DR); //source address, peripheral, I2Sext data register 
    DMA1_Stream3->M0AR = reinterpret_cast<unsigned int>(buffer_rx);   //destination address, memory, buffer
    DMA1_Stream3->NDTR = size; //size of buffer to be filled
    DMA1_Stream3->CR |= DMA_SxCR_EN; //Start the DMA

    //Now enable the I2Sext
    I2S2ext->CR2 |= SPI_CR2_RXDMAEN;


    if((I2S2ext->I2SCFGR & SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE ){
        I2S2ext->I2SCFGR |= SPI_I2SCFGR_I2SE;
    }
    /*
    //for debug purpose, print relevant registers value
    queue.IRQpost([=]{
        iprintf("----------I2S registers-----------\n");
        iprintf("I2S2ext CR2 =%x\n", I2S2ext->CR2);
        iprintf("SPI2 CR2 =%x\n", SPI2->CR2);
        iprintf("I2S2ext SR =%x\n", I2S2ext->SR);
        iprintf("SPI2 SR =%x\n", SPI2->SR);
        iprintf("I2S2ext CFGR =%x\n", I2S2ext->I2SCFGR);
        iprintf("SPI2 CFGR =%x\n", SPI2->I2SCFGR);
        iprintf("I2S2ext I2SPR =%x\n", I2S2ext->I2SPR);
        iprintf("SPI2 I2SPR =%x\n", SPI2->I2SPR);
        iprintf("----------DMA registers-----------\n");
        iprintf("Stream4 PAR =%x\n",DMA1_Stream4->PAR );
        iprintf("Stream4 M0AR =%x\n",DMA1_Stream4->M0AR);
        iprintf("Stream4 CR=%x\n",DMA1_Stream4->CR);
        iprintf("Stream3 PAR =%x\n",DMA1_Stream3->PAR);
        iprintf("Stream3 M0AR =%x\n",DMA1_Stream3->M0AR);
        iprintf("Stream3 CR=%x\n",DMA1_Stream3->CR);
        
    });*/

    return true;
}

bool TLV320AIC3101::I2S_startRx(){
    bool startedDMA = false;
    {
        FastInterruptDisableLock dLock;
        startedDMA = startRxDMA();
    }
    return startedDMA;
}


//--------------------------Function for starting the I2S DMA TX-----------------------------------------
void startTxDMA(const unsigned short *buffer_tx){ 

    
}

void TLV320AIC3101::I2S_startTx(const unsigned short *buffer_tx){
    {
        FastInterruptDisableLock dLock;

        //start also transmission DMA 
        //Start DMA1 with SPI2, memory to peripheral
        DMA1_Stream4->PAR = reinterpret_cast<unsigned int>(&SPI2->DR); //destination address, SPI2 data reg
        DMA1_Stream4->M0AR = reinterpret_cast<unsigned int>(buffer_tx);  //source address, peripheral, memory data register
        DMA1_Stream4->NDTR = size; //size of buffer being sent
        DMA1_Stream4->CR |= DMA_SxCR_EN; //Start the DMA

        //Now enable the I2S
        SPI2->CR2 |= SPI_CR2_TXDMAEN;

        if((SPI2->I2SCFGR & SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE ){
            SPI2->I2SCFGR |= SPI_I2SCFGR_I2SE;
        }
    }
    return; 
}

//-------------------------------IRQ handler functions------------------------------------------------

/********************* DMA1_STREAM3 => I2S2_ext (RX, MISO) ***********************/
void __attribute__((naked)) DMA1_Stream3_IRQHandler(){
    saveContext();
	asm volatile("bl _Z17I2SdmaHandlerImplv"); //_Z, num caratteri nome fun. , nome, v = void 
	restoreContext();
}

//actual function implementation
void __attribute__((used)) I2SdmaHandlerImpl(){ 
    //clear DMA1 interrupt flags
    DMA1->LIFCR=DMA_LIFCR_CTCIF3  | //clear transfer complete flag 
                DMA_LIFCR_CTEIF3  | //clear transfer error flag
                DMA_LIFCR_CDMEIF3 | //clear direct mode error flag
                DMA_LIFCR_CHTIF3  | 
                DMA_LIFCR_CFEIF3;   //clear fifo error interrupt flag
    //mark the buffer as readable
    bq->bufferFilled(size);
    //startRxDMA();
    waiting->IRQwakeup();
    if(waiting->IRQgetPriority()>Thread::IRQgetCurrentThread()->IRQgetPriority())
        Scheduler::IRQfindNextThread();
}

/********************* DMA1_STREAM4 => SPI2 (TX, MOSI) *************************/
void __attribute__((weak)) DMA1_Stream4_IRQHandler(){
    saveContext();
	asm volatile("bl _Z18I2SdmaHandlerImpl2v"); 
	restoreContext();
}

//actual function implementation
void __attribute__((used)) I2SdmaHandlerImpl2(){
    //clear DMA1 interrupt flags
    DMA1->HIFCR=DMA_HIFCR_CTCIF4  | //clear transfer complete flag 
                DMA_HIFCR_CTEIF4  | //clear transfer error flag
                DMA_HIFCR_CDMEIF4 | //clear direct mode error flag
                DMA_HIFCR_CHTIF4  | 
                DMA_HIFCR_CFEIF4; 
    bq->bufferEmptied();
}

//------------------------Codec initialization and STM32 setup method------------------------------------
void TLV320AIC3101::setup(){
    Lock<Mutex> l(mutex);

    thread t([&]{ queue.run(); });
    t.detach();

    //allocation of memory for 2 buffer queues
    bq = new BufferQueue<unsigned short, bufferSize>();

    {
        FastInterruptDisableLock dLock;

        /**************** I2S2, DMA1 INIT *********************/
        //Enable DMA1 and I2S2 clocks on AHB and APB buses
        RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN; 
        RCC_SYNC();
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; //only SPI2 clock must be enabled, shared with 12S2ext
        RCC_SYNC();

        /*
        //debug for output of I2S MCLK
        mco2::mode(Mode::ALTERNATE);
        mco2::alternateFunction(0);
        mco2::speed(Speed::_100MHz);
        RCC ->CFGR |= RCC_CFGR_MCO2_0;
        RCC ->CFGR |= RCC_CFGR_MCO2PRE_2;
        RCC ->CFGR |= RCC_CFGR_MCO2PRE_1;
        */

        //GPIO configuration - alternate function #5, #6 are associated with SPI2/I2S2
        I2C::init();

        mclk::mode(Mode::ALTERNATE);
        mclk::alternateFunction(5);

        sclk::mode(Mode::ALTERNATE);
        sclk::alternateFunction(5);

        sdin::mode(Mode::ALTERNATE);
        sdin::alternateFunction(6);     // I2S_ext associated with AF_6

        sdout::mode(Mode::ALTERNATE);
        sdout::alternateFunction(5);    // SPI2 associated with AF_5

        lrclk::mode(Mode::ALTERNATE);
        lrclk::alternateFunction(5);

        //enabling PLL for I2S and starting clock
        //PLLM = 16 (default), PLLI2SR = 3, PLLI2SN = 258 see datasheet pag.595
        //Actually by trying on the cubeIDE, these values distort the sound more than the one set on the older project (? dunno why)
        RCC->PLLI2SCFGR = (3<<28) | (258<<6); //values suggested by datasheet
        //RCC->PLLI2SCFGR = (10<<28) | (123<<6); //values found by us (works, tested)
        RCC->CR |= RCC_CR_PLLI2SON;
    }

    //wait for PLL to lock
    while((RCC->CR & RCC_CR_PLLI2SRDY)==0);

    /******************* CODEC SETTINGS ************************/
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
    
    /*********************************************** I2S SETTINGS **********************************************************************/
    //I2S + I2Sext must be used in full duplex mode (in half duplex, only I2S can be used, not the _ext)
    //Using Full duplex mode, I2Sext is the MISO (input)
    /********** CONFIGURE BOTH SPI2 + I2S2_EXT REGISTERS ***********/

    /********************************************** SPI2 CONFIGURATION ****************************************************************/
    //enable DMA on I2S, TX mode, no interrupts enabled by SPI2 peripheral, only DMA!
    // VCO @ 1MHz 
    //I2S prescaler register, see pag.595. fi2s = 16M*PLLI2SN/(PLLM*PLLI2SR)=86MHz -> fs=fi2s/[32*(2*I2SDIV+ODD)*8]=47991Hz
    SPI2->I2SPR =   SPI_I2SPR_MCKOE       //mclk enable
                | SPI_I2SPR_ODD         //ODD = 1 
                | (uint32_t)0x00000003; //I2SDIV = 3

    // Default settings: I2S Philips std, CKPOL low, 16 bit DATLEN, CHLEN 16 bit
    SPI2->I2SCFGR = SPI_I2SCFGR_I2SMOD      //I2S mode selected
                  | SPI_I2SCFGR_I2SCFG_1; //Master transmit , this bit should be config. when I2S disabled
                //| SPI_I2SCFGR_I2SCFG; //Master receive , this bit should be config. when I2S disabled

    /********************************************** I2S2_EXT CONFIGURATION ****************************************************************/
    //enable DMA on I2S2ext, RX mode
    // Default settings: I2S Philips std, CKPOL low, 16 bit DATLEN, CHLEN 16 bit
    I2S2ext->I2SCFGR = SPI_I2SCFGR_I2SMOD       //I2S mode selected    
                     | SPI_I2SCFGR_I2SCFG_0;    //Slave Receive
                                              
    /********************************************** DMA1_Stream3 (RX, I2S2_EXT) CONFIGURATION ****************************************************************/
    DMA1_Stream3->CR = 0;                 //reset configuration register to 0
    DMA1_Stream3->CR = DMA_SxCR_CHSEL_3 | //dma1 stream 3 channel 3
                       DMA_SxCR_PL_1    | //High priority DMA stream
                       DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
					   DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
				       DMA_SxCR_MINC    | //Increment RAM pointer after each transfer
                       DMA_SxCR_TEIE    | //Interrupt on transfer error
                       DMA_SxCR_DMEIE   | //Interrupt on direct mode error
			           DMA_SxCR_TCIE;     //Interrupt on completion

    /********************************************** DMA1_Stream4 (TX, SPI2) CONFIGURATION ****************************************************************/
    DMA1_Stream4->CR = 0;                 //reset configuration register to 0
    DMA1_Stream4->CR = //DMA_SxCR_CHSEL_0 | //dma1 stream 4 channel 0
                       DMA_SxCR_PL_1    | //High priority DMA stream
                       DMA_SxCR_MSIZE_0 | //Read  16bit at a time from RAM
                       DMA_SxCR_PSIZE_0 | //Write 16bit at a time to SPI
                       DMA_SxCR_MINC    | //Increment RAM pointer after each transfer
                       DMA_SxCR_DIR_0   | //Mem to periph
                       DMA_SxCR_TEIE    | //Interrupt on transfer error
                       DMA_SxCR_DMEIE   | //Interrupt on direct mode error
                       DMA_SxCR_TCIE;     //Interrupt on completion

    /********************* ENABLE DMA_IT AND SET PRIORITY ****************/
    // DMA1_Stream3 => I2S_ext_RX
    NVIC_SetPriority(DMA1_Stream3_IRQn,2);   //high prio
    NVIC_EnableIRQ(DMA1_Stream3_IRQn);       //enable interrupt

    // DMA1_Stream4 => SPI2_TX
    NVIC_SetPriority(DMA1_Stream4_IRQn,2);   
    NVIC_EnableIRQ(DMA1_Stream4_IRQn);     

    waiting=Thread::getCurrentThread();
}
