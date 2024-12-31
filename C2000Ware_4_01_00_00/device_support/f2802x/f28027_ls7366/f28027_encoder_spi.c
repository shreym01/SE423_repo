//#############################################################################
// FILE:   f28027_main.c
//
// TITLE:  Lab Starter
//#############################################################################

//
// Included Files
//
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "DSP28x_Project.h"
#include "f28027Serial.h"

#include "fft.h"
#include "fft_hamming_Q31.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

#define FFT_SIZE    512

uint16_t temp=0;

uint16_t XG_OFFSET_H        = 0x1300;
uint16_t XG_OFFSET_L        = 0x1400;
uint16_t YG_OFFSET_H        = 0x1500;
uint16_t YG_OFFSET_L        = 0x1600;
uint16_t ZG_OFFSET_H        = 0x1700;
uint16_t ZG_OFFSET_L        = 0x1800;

uint16_t SMPLRT_DIV         = 0x1900;

uint16_t CONFIG             = 0x1A00;
uint16_t GYRO_CONFIG        = 0x1B00;
uint16_t ACCEL_CONFIG       = 0x1C00;
uint16_t ACCEL_CONFIG_2     = 0x1D00;

uint16_t LP_ACCEL_ODR       = 0x1E00;
uint16_t WOM_THR            = 0x1F00;
uint16_t FIFO_EN            = 0x2300;

uint16_t I2C_MST_CTRL       = 0x2400;
uint16_t I2C_SLV0_ADDR      = 0x2500;
uint16_t I2C_SLV0_REG       = 0x2600;
uint16_t I2C_SLV0_CTRL      = 0x2700;
uint16_t I2C_SLV1_ADDR      = 0x2800;
uint16_t I2C_SLV1_REG       = 0x2900;
uint16_t I2C_SLV1_CTRL      = 0x2A00;

uint16_t INT_ENABLE         = 0x3800;
uint16_t INT_STATUS         = 0x3A00;

uint16_t I2C_SLV1_DO        = 0x6400;
uint16_t I2C_MST_DELAY_CTRL = 0x6700;

uint16_t USER_CTRL          = 0x6A00;
uint16_t PWR_MGMT_1         = 0x6B00;
uint16_t WHO_AM_I           = 0x7500;

uint16_t XA_OFFSET_H        = 0x7700;
uint16_t XA_OFFSET_L        = 0x7800;
uint16_t YA_OFFSET_H        = 0x7A00;
uint16_t YA_OFFSET_L        = 0x7B00;
uint16_t ZA_OFFSET_H        = 0x7D00;
uint16_t ZA_OFFSET_L        = 0x7E00;
//
// Function Prototypes
//
__interrupt void adc_isr(void);
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void ADCB_ISR(void);

void serialRXA(serial_t *s, char data);
void setupSpia(void);
void start_SPI(void);
__interrupt void SPI_RXint(void);
uint16_t SPIenc_state = 0;
uint16_t UARTPrint = 0;
uint16_t numRXA = 0;
int16_t Timer0Count = 0;
int16_t ADC0raw = 0;
int16_t ADC2raw = 0;
int32_t microphone = 0;

uint32_t ADCcount = 0;
int16_t fftsamplecount = 0;
int16_t pingpong = 0; // ping = 0, pong = 1
uint16_t runpingfftcount = 0;
int16_t RunPingFFT = 0;
int16_t RunPongFFT = 0;

int32_t SPIenc1_reading = 0;
int32_t SPIenc2_reading = 0;

int32_t SPIbyte1 = 0;
int32_t SPIbyte2 = 0;
int32_t SPIbyte3 = 0;
int32_t SPIbyte4 = 0;
int32_t SPIbyte5 = 0;

#ifndef __cplusplus
#pragma DATA_SECTION(ipcb, "FFTipcb");
#else
#pragma DATA_SECTION("FFTipcb");
#endif
int32_t ipcb[FFT_SIZE+2];

#ifndef __cplusplus
#pragma DATA_SECTION(ipcbsrcping, "FFTipcbsrc");
#else
#pragma DATA_SECTION("FFTipcbsrc");
#endif
#ifndef __cplusplus
#pragma DATA_SECTION(ipcbsrcpong, "FFTipcbsrc");
#else
#pragma DATA_SECTION("FFTipcbsrc");
#endif
int32_t ipcbsrcping[FFT_SIZE];
int32_t ipcbsrcpong[FFT_SIZE];

// Declare and initialize the structure object.
// Use the RFFT32_<n>P_DEFUALTS in the FFT header file if
// unsure as to what values to program the object with.
RFFT32  rfft = RFFT32_512P_DEFAULTS;

// Define window Co-efficient Array
// Note: Windowing is not used in this example
const long win[FFT_SIZE/2]=HAMMING512;

int32_t maxpwr = 0;
int16_t maxpwrindex = 0;
float maxpwr_f = 0;
//
// Main
//
void main(void)
{
    uint32_t i = 0;
    //
    // WARNING: Always ensure you call memcpy before running any functions from
    // RAM InitSysCtrl includes a call to a RAM based function and without a 
    // call to memcpy first, the processor will go "into the weeds"
    //
#ifdef _FLASH
    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
#endif

    //
    // Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the f2802x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the f2802x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in f2802x_DefaultIsr.c.
    // This function is found in f2802x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;            // This is needed to write to EALLOW protected registers
    PieVectTable.ADCINT1 = &adc_isr;
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.TINT1 = &cpu_timer1_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;
	PieVectTable.SCIRXINTA = &RXAINT_recv_ready;
    PieVectTable.SCITXINTA = &TXAINT_data_sent;
    PieVectTable.SPIRXINTA = &SPI_RXint;
    EDIS;      // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in f2802x_CpuTimers.c
    //
    InitCpuTimers();        // For this example, only initialize the Cpu Timers


    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 60MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 60, 100000);
    ConfigCpuTimer(&CpuTimer1, 60, 1000);
    ConfigCpuTimer(&CpuTimer2, 60, 1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the 
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in f2802x_CpuTimers.h), the
    // below settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0
    CpuTimer1Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0
    CpuTimer2Regs.TCR.all = 0x4001; //write-only instruction to set TSS bit = 0

    //
    // User specific code, enable interrupts
    //
	init_serial(&SerialA,115200,serialRXA);

	InitSpiGpio(); // Just Setup SPI pins
	setupSpia();
	InitAdc();
	InitAdcAio();

    //
    // Configure ADC
    EALLOW;
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcRegs.INTSEL1N2.bit.INT1E     = 1;    // Enabled ADCINT1
    AdcRegs.INTSEL1N2.bit.INT1CONT  = 0;    // Disable ADCINT1 Continuous mode
    AdcRegs.INTSEL1N2.bit.INT1SEL   = 1;
    AdcRegs.ADCSOC0CTL.bit.CHSEL    = 0;
    AdcRegs.ADCSOC1CTL.bit.CHSEL    = 2;
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0xB;  //EPWM4A
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0xB;
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 40;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 40;
    EDIS;


    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (â€œpulseâ is the same as â€œtriggerâ€)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1  50Mhz Clock
    EPwm4Regs.TBPRD = 30000/10;  //10000Hz,  Input clock is 30MHz.  60Mhz/2
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode



    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A

    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B
    EDIS;

    EPwm2Regs.TBCTL.bit.CTRMODE = 0;      //set epwm2 to upcount mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm2Regs.TBCTL.bit.CLKDIV = 4;  // Divide by 16
    EPwm2Regs.TBPRD = 37500; //set epwm2 counter  50Hz
    EPwm2Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
    EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
    EPwm2Regs.CMPA.half.CMPA = 3000;
    EPwm2Regs.CMPB = 3000;

    EALLOW;
//    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
//    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;   // Configure GPIO4 as EPWM3A
//    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
//    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
//    GpioCtrlRegs.GPBPUD.bit.GPIO4 = 1;

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B
    EDIS;
//
//    EPwm3Regs.TBCTL.bit.CTRMODE = 0;      //set epwm3 to upcount mode
//    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
//    EPwm3Regs.TBCTL.bit.CLKDIV = 4;   // Divide by 16
//    EPwm3Regs.TBPRD = 37500; //set epwm3 counter  50Hz
//    EPwm3Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
//    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
//    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
//    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
//    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
//    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
//    EPwm3Regs.CMPA.half.CMPA = 3000;
//    EPwm3Regs.CMPB = 3000;
//



        //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:
    //
    IER |= M_INT1;
    IER |= M_INT13;
    IER |= M_INT14;
	IER |= M_INT9;  // SCIA
	IER |= M_INT6; // spia


//    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 ADCA1
    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    EALLOW;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 1;
    EDIS;


    //
    // Enable global Interrupts and higher priority real-time debug events
    //
    EINT;           // Enable Global interrupt INTM
    ERTM;           // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (RunPingFFT == 1) {
//            GpioDataRegs.GPASET.bit.GPIO4 = 1;
            RunPingFFT = 0;
            //Clean up input/output buffer
            for(i=0; i < (FFT_SIZE+2); i=i+2){
                ipcb[i]   = 0;
                ipcb[i+1] = 0;
            }
            RFFT32_brev(ipcbsrcping, ipcb, FFT_SIZE); // real FFT bit reversing

            rfft.ipcbptr = ipcb;                  // FFT computation buffer
            rfft.magptr  = ipcbsrcping;               // Magnitude output buffer
            rfft.winptr  = (long *)win;           // Window coefficient array
            rfft.init(&rfft);                     // Twiddle factor pointer initialization

            rfft.calc(&rfft);                     // Compute the FFT
            rfft.split(&rfft);                    // Post processing to get the correct spectrum
            rfft.mag(&rfft);                      // Q31 format (abs(ipcbsrc)/2^16).^2
            maxpwr = 0;
            maxpwrindex = 0;

            for (i=5;i<(FFT_SIZE/2);i++) {
                if (ipcbsrcping[i]>maxpwr) {
                    maxpwr = ipcbsrcping[i];
                    maxpwrindex = i;
                }
            }
            maxpwr_f = maxpwr/2147483648.0;
//            GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
        }
        else if (RunPongFFT == 1) {
//            GpioDataRegs.GPASET.bit.GPIO4 = 1;
            RunPongFFT = 0;
            //Clean up input/output buffer
            for(i=0; i < (FFT_SIZE+2); i=i+2){
                ipcb[i]   = 0;
                ipcb[i+1] = 0;
            }
            RFFT32_brev(ipcbsrcpong, ipcb, FFT_SIZE); // real FFT bit reversing

            rfft.ipcbptr = ipcb;                  // FFT computation buffer
            rfft.magptr  = ipcbsrcpong;               // Magnitude output buffer
            rfft.winptr  = (long *)win;           // Window coefficient array
            rfft.init(&rfft);                     // Twiddle factor pointer initialization

            rfft.calc(&rfft);                     // Compute the FFT
            rfft.split(&rfft);                    // Post processing to get the correct spectrum
            rfft.mag(&rfft);                      // Q31 format (abs(ipcbsrc)/2^16).^2
            maxpwr = 0;
            maxpwrindex = 0;

            for (i=5;i<(FFT_SIZE/2);i++) {
                if (ipcbsrcpong[i]>maxpwr) {
                    maxpwr = ipcbsrcpong[i];
                    maxpwrindex = i;
                }
            }
            maxpwr_f = maxpwr/2147483648.0;
//            GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
        }

        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Pwr=%.3f,I=%d \r\n",maxpwr_f,maxpwrindex);
            UARTPrint = 0;
        }
    }
}

void setupSpia(void)        //for mpu9250
{
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (SIMO)
    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;   // Configure GPIO16 as SIMO

    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (SOMI)
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;   // Configure GPIO17 as SOMI

    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;    // Enable pull-up on GPIO18 (CLK)
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;   // Configure GPIO18 as CLK

    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 1;

    EDIS;

    EALLOW;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;

    SpiaRegs.SPICCR.bit.SPISWRESET   = 0;    // Put SPI in Reset

    SpiaRegs.SPICTL.bit.CLK_PHASE    = 1;
    SpiaRegs.SPICCR.bit.CLKPOLARITY  = 0;

    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;    // SPI master
    SpiaRegs.SPICCR.bit.SPICHAR      = 0x7;  // Set to transmit 16 bits
    SpiaRegs.SPICTL.bit.TALK         = 1;    // Enables transmission for the 4-pin option
    SpiaRegs.SPIPRI.bit.FREE         = 1;    // Free run, continue SPI operation
    SpiaRegs.SPICTL.bit.SPIINTENA    = 0;    // Disables the SPI interrupt

    SpiaRegs.SPIBRR = LSPCLK_HZ/1000000L - 1;

    SpiaRegs.SPISTS.all              = 0x0000;

    SpiaRegs.SPIFFTX.bit.SPIRST      = 1;    // SPI FIFO can resume transmit or receive.
    SpiaRegs.SPIFFTX.bit.SPIFFENA    = 1;    // SPI FIFO enhancements are enabled
    SpiaRegs.SPIFFTX.bit.TXFIFO      = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR  = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag

    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFOVF]
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpiaRegs.SPIFFRX.bit.RXFFIENA    = 1;    // RX FIFO interrupt based on RXFFIL match

    SpiaRegs.SPIFFCT.bit.TXDLY       = 0; // The next word in the TX FIFO buffer is transferred to SPITXBUF immediately upon completion of transmission of the previous word.

    SpiaRegs.SPICCR.bit.SPISWRESET   = 1;    // Pull the SPI out of reset

    SpiaRegs.SPIFFTX.bit.TXFIFO      = 1;    // Release transmit FIFO from reset.
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpiaRegs.SPICTL.bit.SPIINTENA    = 1;    // Enables the SPI interrupt.
    SpiaRegs.SPIFFRX.bit.RXFFIL      = 5;    // A RX FIFO interrupt request is generated when there are 1 or more words in the RX buffer.

    //-----------------------------------------------------------------------------------------------------------------

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1300         | 0x00); // 0x1300 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1400 (0x0000),       0x1500 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1600 (0x0000),       0x1700 (0x0000)
    SpiaRegs.SPITXBUF = (0x0000         | 0x13); // 0x1800 (0x0000),       0x1900 (0x0013)

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1A00         | 0x02); //0x1A00 (0x0002),

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);
    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (0x1B00         | 0x00); // 0x1B00 (0x0000), 250 dps
    SpiaRegs.SPITXBUF = (0x0800         | 0x06); // 0x1C00 (0x0008), +-4g, 0x1D00 (0x0006), 5Hz
    SpiaRegs.SPITXBUF = (0x0000         | 0x00); // 0x1E00 (0x0000),       0x1F00 (0x0000)

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=3);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (FIFO_EN        | 0x0000); // 0x2300 (0x0000)
    SpiaRegs.SPITXBUF = (0x4000         | 0x008C); // 0x2400 (0x0040),       0x2500 (0x008C)
    SpiaRegs.SPITXBUF = (0x0200         | 0x0088); // 0x2600 (0x0002),       0x2700 (0x0088)
    SpiaRegs.SPITXBUF = (0x0C00         | 0x000A); // 0x2800 (0x000C),       0x2900 (0x000A)
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_SLV1_CTRL | 0x0081);  // 0x2A00 (0x0081)
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (INT_ENABLE | 0x0001);  // 0x3800
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (INT_STATUS | 0x0001);  // 0x3A00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_SLV1_DO | 0x0001);  // 0x6400
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (I2C_MST_DELAY_CTRL | 0x0003); // 0x6700
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (USER_CTRL | 0x0020);  // 0x6A00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (PWR_MGMT_1 | 0x0001); // 0x6B00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (WHO_AM_I | 0x0071);  // 0x7500
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (XA_OFFSET_H | 0x00EB); // 0x7700
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (XA_OFFSET_L | 0x0012); // 0x7800
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (YA_OFFSET_H | 0x10F1); // 0x7A00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (YA_OFFSET_L | 0x00E0); // 0x7B00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (ZA_OFFSET_H | 0x0021); // 0x7D00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
    SpiaRegs.SPITXBUF = (ZA_OFFSET_L | 0x0050); // 0x7E00
    while(SpiaRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;
    temp = SpiaRegs.SPIRXBUF;

    while(SpiaRegs.SPIFFRX.bit.RXFFST !=0){
        temp = SpiaRegs.SPIRXBUF;
    }


    DELAY_US(50);
    EALLOW;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;

    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;
    GpioDataRegs.GPASET.bit.GPIO6 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;
    EDIS;

    DELAY_US(1000);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    SpiaRegs.SPITXBUF = ((unsigned)0x20)<<8;  // CLR COUNT all four chips
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 1) {}
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

    temp = SpiaRegs.SPIRXBUF;


    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
    SpiaRegs.SPITXBUF = ((unsigned)0x88)<<8;  // WR to MDR0
    SpiaRegs.SPITXBUF = ((unsigned)0x83)<<8;
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
    SpiaRegs.SPITXBUF = ((unsigned)0x90)<<8;  // WR MDR1
    SpiaRegs.SPITXBUF = 0x01<<8;
    while (SpiaRegs.SPIFFRX.bit.RXFFST != 2) {}
    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;
    temp = SpiaRegs.SPIRXBUF;

    SpiaRegs.SPICTL.bit.SPIINTENA = 1;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt
}

void start_SPI(void) {
//    SpiaRegs.SPICCR.bit.SPICHAR      = 0x7;  // Set to transmit 8 bits
    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    SpiaRegs.SPITXBUF = ((unsigned)0xE8)<<8; // Latch All ENCs
    SPIenc_state = 1;
}


__interrupt void adc_isr(void)
{
    ADC0raw = AdcResult.ADCRESULT0;
    microphone = ((float)((ADC0raw)/4096.0))*2147483648;
    if (pingpong == 0) {
        ipcbsrcping[fftsamplecount] = microphone;
        fftsamplecount++;
        if (fftsamplecount == FFT_SIZE) {
            fftsamplecount = 0;
            runpingfftcount = 0;
            RunPingFFT = 1;
            pingpong = 1;
        }
    } else {
        ipcbsrcpong[fftsamplecount] = microphone;
        fftsamplecount++;
        if (fftsamplecount == FFT_SIZE) {

            fftsamplecount = 0;
            runpingfftcount = 0;
            RunPongFFT = 1;
            pingpong = 0;
        }
    }



    ADCcount++;
    if ((ADCcount%500)==0) {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
    //
    // Clear ADCINT1 flag reinitialize for next SOC
    //
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    //
    // Acknowledge interrupt to PIE
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

void SPI_RXint(void) {


    GpioDataRegs.GPASET.bit.GPIO4 = 1;
    GpioDataRegs.GPASET.bit.GPIO6 = 1;
    switch (SPIenc_state) {
        case 1:
            temp = SpiaRegs.SPIRXBUF;

            SpiaRegs.SPIFFRX.bit.RXFFIL = 4;
            SPIenc_state = 2;
            GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            break;
        case 2:

            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
//            SPIenc1_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
            SPIenc1_reading =  ((SPIbyte2<<16) | (SPIbyte3<<8) | SPIbyte4)<<8;
            SPIenc1_reading = SPIenc1_reading >>8;
            SpiaRegs.SPIFFRX.bit.RXFFIL = 4;
            SPIenc_state = 3;
            GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
            SpiaRegs.SPITXBUF = ((unsigned)0x68)<<8;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            SpiaRegs.SPITXBUF = 0;
            break;

        case 3:
            SPIenc_state = 30;
            SPIbyte1 = SpiaRegs.SPIRXBUF;
            SPIbyte2 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte3 = SpiaRegs.SPIRXBUF & 0xFF;
            SPIbyte4 = SpiaRegs.SPIRXBUF & 0xFF;
//            SPIenc2_reading = (SPIbyte2<<24) | (SPIbyte3<<16) | (SPIbyte4<<8) | SPIbyte5;
            SPIenc2_reading = ((SPIbyte2<<16) | (SPIbyte3<<8) | SPIbyte4)<<8;
            SPIenc2_reading = SPIenc2_reading>>8;
//            SWI_post(&SWI_control);
            break;
    }
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE

}

//
// cpu_timer0_isr - 
//
__interrupt void cpu_timer0_isr(void)
{
    Timer0Count++;
    if (Timer0Count > 32000) { // Rolls over at 32767 so catch before that point
        Timer0Count = 0;
    }
   // GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
	UARTPrint = 1;
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - 
//
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
    start_SPI();
    //
    // The CPU acknowledges the interrupt
    //
    EDIS;
}

//
// cpu_timer2_isr - 
//
__interrupt void cpu_timer2_isr(void)
{  
    EALLOW;
    CpuTimer2.InterruptCount++;
    
    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}

//
// End of File
//
