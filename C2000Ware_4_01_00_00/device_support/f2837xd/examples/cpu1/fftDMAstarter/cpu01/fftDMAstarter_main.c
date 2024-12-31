//#############################################################################
// FILE:   fftDMAstarter_main.c
//
// TITLE:  FFT DMA Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200

// To Move FFT DMA code to your Robot or Homework Projects look for the !!!!!!! comments
// For blocks of code to copy into your other project.  This will give you code to take
// FFT of the microphone connected to ADCINB4
//!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code to your global variable declarations
//*****************************************************************************
// the defines for FFT
//*****************************************************************************
#define RFFT_STAGES     10
#define RFFT_SIZE       (1 << RFFT_STAGES)

//*****************************************************************************
// the globals
//*****************************************************************************
#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(pwrSpec, "FFT_buffer_2")
#endif
float pwrSpec[(RFFT_SIZE/2)+1];
float maxpwr = 0;
int16_t maxpwrindex = 0;

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(test_output, "FFT_buffer_2")
#endif
float test_output[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_1")
#else
#pragma DATA_SECTION(fft_input, "FFT_buffer_1")
#endif
float fft_input[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(RFFTF32Coef,"FFT_buffer_2")
#endif //__cplusplus
//! \brief Twiddle Factors
//!
float RFFTF32Coef[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPingBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPingBufRaw[RFFT_SIZE];

#ifdef __cplusplus
#pragma DATA_SECTION("FFT_buffer_2")
#else
#pragma DATA_SECTION(AdcPongBufRaw, "FFT_buffer_2")
#endif
uint16_t AdcPongBufRaw[RFFT_SIZE];


//! \brief Object of the structure RFFT_F32_STRUCT
//!
RFFT_F32_STRUCT rfft;

//! \brief Handle to the RFFT_F32_STRUCT object
//!
RFFT_F32_STRUCT_Handle hnd_rfft = &rfft;
uint16_t pingFFT = 0;
uint16_t pongFFT = 0;
uint16_t pingpongFFT = 1;
uint16_t iPingPong = 0;
int16_t DMAcount = 0;
__interrupt void DMA_ISR(void);
void InitDma(void);
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);


// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;


void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LS7366#1 CS
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LS7366#2 CS	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LS7366#3 CS
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LS7366#4 CS
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// WIZNET RST
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	//PushButton 1
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_INPUT, GPIO_PULLUP);

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //F28027 CS
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

     // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
//!!!!!!!!!!!!!!!!!!!!!!  Copy the Assignmnt of the DMA interrupt service routine
    PieVectTable.DMA_CH1_INT = &DMA_ISR;
//!!!!!!!!!!!!!!!!!!!!!!  End of Block
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
//    init_serialSCIC(&SerialC,115200);
//    init_serialSCID(&SerialD,115200);


//!!!!!!!!!!!!!!!!!!!!!! DMAFFT Copy this block of code after your init_serial functions
    EALLOW;
    EPwm7Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm7Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm7Regs.ETSEL.bit.SOCASEL = 0x2; // Select Event when counter equal to PRD
    EPwm7Regs.ETPS.bit.SOCAPRD = 0x1; // Generate pulse on 1st event
    EPwm7Regs.TBCTR = 0x0; // Clear counter
    EPwm7Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm7Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm7Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm7Regs.TBPRD = 5000; // Set Period to 0.1ms sample. Input clock is 50MHz.
    EPwm7Regs.ETSEL.bit.SOCAEN = 1; // Disable SOC on A group
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal EPwm7Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm7Regs.TBCTL.bit.CTRMODE = 0x00; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1; //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1; //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //ADCB
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4; //SOC0 will convert Channel you choose Does not have to be B0
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x11; // EPWM7 ADCSOCA or another trigger you choose will trigger SOC0
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag, originally = 1 (need to set to 1 for non-DMA)
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // Interrupt pulses regardless of flag state
    EDIS;
    // Initialize DMA
    InitDma();
    //!!!!!!!!!!!!!!!!!!!!!!  End of Block

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

//!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code right before your EINT; line of code
    int16_t i = 0;
    float samplePeriod = 0.0001;

    // Clear input buffers:
    for(i=0; i < RFFT_SIZE; i++){
        fft_input[i] = 0.0f;
    }

    for (i=0;i<RFFT_SIZE;i++) {
        fft_input[i] = sin(125*2*PI*i*samplePeriod)+2*sin(2400*2*PI*i*samplePeriod);
    }
    hnd_rfft->FFTSize   = RFFT_SIZE;
    hnd_rfft->FFTStages = RFFT_STAGES;
    hnd_rfft->InBuf     = &fft_input[0];  //Input buffer
    hnd_rfft->OutBuf    = &test_output[0];  //Output buffer
    hnd_rfft->MagBuf    = &pwrSpec[0];  //Magnitude buffer
    
    hnd_rfft->CosSinBuf = &RFFTF32Coef[0];  //Twiddle factor buffer
    RFFT_f32_sincostable(hnd_rfft);         //Calculate twiddle factor

    for (i=0; i < RFFT_SIZE; i++){
          test_output[i] = 0;               //Clean up output buffer
    }

    for (i=0; i <= RFFT_SIZE/2; i++){
         pwrSpec[i] = 0;                //Clean up magnitude buffer
    }


    int16_t tries = 0;
    while(tries < 10*0) {  // Get ride of the 0 in 10*0 if you want to run this while loop and test out the FFT function with these sin waves
        RFFT_f32(hnd_rfft);                     //Calculate real FFT
        
    #ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
            // properties
            RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
    #else
            RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
    #endif
        maxpwr = 0;
        maxpwrindex = 0;

        for (i=0;i<(RFFT_SIZE/2);i++) {
            if (pwrSpec[i]>maxpwr) {
                maxpwr = pwrSpec[i];
                maxpwrindex = i;
            }
        }

        tries++;
        for (i=0;i<RFFT_SIZE;i++) {
            fft_input[i] = sin((125 + tries*125)*2*PI*i*samplePeriod)+2*sin((2400-tries*200)*2*PI*i*samplePeriod);
        }
    }
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			// Normally on the Robot Car we only use the below UART_printfLine functions to write to the
			// on board LCD screen.  This below serial_printf is only used in lab 1 to print to a serial 
			// terminal over a USB cable like you will for your Homeworks.
			//serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
			
			//IMPORTANT!! %ld is for an int32_t.  To print an int16_t use %d
			UART_printfLine(1,"Max Pwr %.3f",maxpwr);
            UART_printfLine(2,"Freq %.0f",maxpwrindex*10000.0/1024.0);
            UARTPrint = 0;
        }
//!!!!!!!!!!!!!!!!!!!!!!  Copy this block of code after your UARTPrint == 1 while loop as above
        if ( (pingFFT == 1) || (pongFFT == 1) ) {
            if (pingFFT == 1) {
                pingFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC results:
                    fft_input[i] = AdcPingBufRaw[i]*3.0/4095.0;  // ping data
                }
            } else if (pongFFT == 1) {
                pongFFT = 0;
                // Raw ADC data
                for(i=0; i<RFFT_SIZE; i++) {
                    //--- Read the ADC result:
                    fft_input[i] = AdcPongBufRaw[i]*3.0/4095.0;  // pong data
                }
            }
            
            RFFT_f32(hnd_rfft);

            #ifdef __TMS320C28XX_TMU__ //defined when --tmu_support=tmu0 in the project
                        // properties
                    RFFT_f32_mag_TMU0(hnd_rfft);            //Calculate magnitude
            #else
                    RFFT_f32_mag(hnd_rfft);                 //Calculate magnitude
            #endif

            maxpwr = 0;
            maxpwrindex = 0;

            for (i=5;i<(RFFT_SIZE/2);i++) {
                if (pwrSpec[i]>maxpwr) {
                    maxpwr = pwrSpec[i];
                    maxpwrindex = i;
                }
            }

            UARTPrint = 1;
        }
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%5) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;
	
//	if ((CpuTimer2.InterruptCount % 10) == 0) {
//		UARTPrint = 1;
//	}
}

//!!!!!!!!!!!!!!!!!!!!!!  Copy these two function to the end of your C file
interrupt void DMA_ISR(void)                    // PIE7.1 @ 0x000DA0  DMA channel 1 interrupt
{
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP7;         // Must acknowledge the PIE group

    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;

    //--- Process the ADC data
    if(iPingPong == 0)  // Ping buffer filling, process Pong bugger
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPongBufRaw;      // Adjust DST start address for Pong buffer
        EDIS;                                                       // Disable EALLOW protected register access

        // Don't run ping FFT first time around
        if (DMAcount > 0) {
            pingFFT = 1;
        }
        iPingPong = 1;
    }
    else    // Pong buffer filling, process Ping buffer
    {
        // Manage the DMA registers
        EALLOW;                                                     // Enable EALLOW protected register access
        DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // Adjust DST start address for Ping buffer
        EDIS;                                                       // Disable EALLOW protected register access

        pongFFT = 1;
        iPingPong = 0;
    }

    DMAcount += 1;

}

void InitDma(void)
{
    EALLOW;

    //---------------------------------------------------------------------
    //--- Overall DMA setup
    //---------------------------------------------------------------------
    DmaRegs.DMACTRL.bit.HARDRESET = 1;          // Reset entire DMA module
    asm(" NOP");                                // 1 cycle delay for HARDRESET to take effect

    DmaRegs.DEBUGCTRL.bit.FREE = 1;             // 1 = DMA unaffected by emulation halt
    DmaRegs.PRIORITYCTRL1.bit.CH1PRIORITY = 0;  // Not using CH1 Priority mode

    //---------------------------------------------------------------------
    //--- Configure DMA channel 1 to read the ADC results
    //---------------------------------------------------------------------
    DmaRegs.CH1.MODE.all = 0x8901; //good
    // bit 15        1:      CHINTE, 0=interrupt disabled, 1=interrupt enabled
    // bit 14        0:      DATASIZE, 0=16-bit, 1=32-bit
    // bit 13-12     00:     reserved
    // bit 11        1:      CONTINUOUS, 0=stop, 1=re-init after transfer complete
    // bit 10        0:      ONESHOT, 0=one burst on trigger, 1=all bursts on trigger
    // bit 9         0:      CHINTMODE, 0=start of transfer, 1=end of transfer
    // bit 8         1:      PERINTE, peripheral interrupt trigger enable, 0=disabled, 1=enabled
    // bit 7         0:      OVRINTE, overflow interrupt enable, 0=disabled, 1=enabled
    // bit 6-5       00:     reserved
    // bit 4-0       00001:  Set to channel number

    //--- Select DMA interrupt source                 /******** TRIGGER SOURCE FOR EACH DMA CHANNEL (unlisted numbers are reserved) ********/
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH1 = 6;    // 0=none       6=ADCBINT1  12=ADCCINT2  18=ADCDINT3  32=XINT4      40=EPWM3SOCA  46=EPWM6SOCA  52=EPWM9SOCA   58=EPWM12SOCA  72=MREVTA    98=SD1FLT4    110=SPIRXDMAA     132=USBA_EPx_TX1
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH2 = 0;    // 1=ADCAINT1   7=ADCBINT2  13=ADCCINT3  19=ADCDINT4  33=XINT5      41=EPWM3SOCB  47=EPWM6SOCB  53=EPWM9SOCB   59=EPWM12SOCB  73=MXEVTB    99=SD2FLT1    111=SPITXDMAB     133=USBA_EPx_RX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH3 = 0;    // 2=ADCAINT2   8=ADCBINT3  14=ADCCINT4  20=ADCDEVT   36=EPWM1SOCA  42=EPWM4SOCA  48=EPWM7SOCA  54=EPWM10SOCA  68=TINT0       74=MREVTB   100=SD2FLT2    112=SPIRXDMAB     134=USBA_EPx_TX2
    DmaClaSrcSelRegs.DMACHSRCSEL1.bit.CH4 = 0;    // 3=ADCAINT3   9=ADCBINT4  15=ADCCEVT   29=XINT1     37=EPWM1SOCB  43=EPWM4SOCB  49=EPWM7SOCB  55=EPWM10SOCB  69=TINT1       95=SD1FLT1  101=SD2FLT3    113=SPITXDMAC     135=USBA_EPx_RX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH5 = 0;    // 4=ADCAINT4  10=ADCBEVT   16=ADCDINT1  30=XINT2     38=EPWM2SOCA  44=EPWM5SOCA  50=EPWM8SOCA  56=EPWM11SOCA  70=TINT2       96=SD1FLT2  102=SD2FLT4    114=SPIRXDMAC     136=USBA_EPx_TX3
    DmaClaSrcSelRegs.DMACHSRCSEL2.bit.CH6 = 0;    // 5=ADCAEVT   11=ADCCINT1  17=ADCDINT2  31=XINT3     39=EPWM2SOCB  45=EPWM5SOCB  51=EPWM8SOCB  57=EPWM11SOCB  71=MXEVTA      97=SD1FLT3  109=SPITXDMAA  131=USBA_EPx_RX1

    //--- DMA trigger source lock
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL1 = 0;              // Write a 1 to lock (cannot be cleared once set)
    DmaClaSrcSelRegs.DMACHSRCSELLOCK.bit.DMACHSRCSEL2 = 0;              // Write a 1 to lock (cannot be cleared once set)


    DmaRegs.CH1.BURST_SIZE.bit.BURSTSIZE = 0;                           // 0 means 1 word per burst
    DmaRegs.CH1.TRANSFER_SIZE = RFFT_SIZE-1;                          // RFFT_SIZE bursts per transfer

    DmaRegs.CH1.SRC_TRANSFER_STEP = 0;                                  // 0 means add 0 to pointer each burst in a transfer
    DmaRegs.CH1.SRC_ADDR_SHADOW = (Uint32)&AdcbResultRegs.ADCRESULT0;   // SRC start address

    DmaRegs.CH1.DST_TRANSFER_STEP = 1;                                  // 1 = add 1 to pointer each burst in a transfer
    DmaRegs.CH1.DST_ADDR_SHADOW = (Uint32)AdcPingBufRaw;                    // DST start address Ping buffer


    DmaRegs.CH1.CONTROL.all = 0x0091; //good
    // bit 15        0:      reserved
    // bit 14        0:      OVRFLG, overflow flag, read-only
    // bit 13        0:      RUNSTS, run status, read-only
    // bit 12        0;      BURSTSTS, burst status, read-only
    // bit 11        0:      TRANSFERSTS, transfer status, read-only
    // bit 10-9      00:     reserved
    // bit 8         0:      PERINTFLG, read-only
    // bit 7         1:      ERRCLR, error clear, 0=no action, 1=clear SYNCERR bit
    // bit 6-5       00:     reserved
    // bit 4         1:      PERINTCLR, periph event clear, 0=no action, 1=clear periph event
    // bit 3         0:      PERINTFRC, periph event force, 0=no action, 1=force periph event
    // bit 2         0:      SOFTRESET, 0=no action, 1=soft reset the channel
    // bit 1         0:      HALT, 0=no action, 1=halt the channel
    // bit 0         1:      RUN, 0=no action, 1=enable the channel

    //--- Finish up
    EDIS;

    //--- Enable the DMA interrupt
    PieCtrlRegs.PIEIER7.bit.INTx1 = 1;  // Enable DINTCH1 in PIE group 7
    IER |= M_INT7;                      // Enable INT7 in IER to enable PIE group


} // end InitDma()
//!!!!!!!!!!!!!!!!!!!!!!  End of Block

