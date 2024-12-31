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

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
//
// Function Prototypes
//
__interrupt void adc_isr(void);
__interrupt void cpu_timer0_isr(void);
//__interrupt void cpu_timer1_isr(void);  //Just have timer1 trigger ADC
__interrupt void cpu_timer2_isr(void);
__interrupt void SPI_RXint(void);

void serialRXA(serial_t *s, char data);

uint16_t UARTPrint = 0;
uint16_t numRXA = 0;
int16_t Timer0Count = 0;
int16_t ADC0raw = 0;
int16_t ADC2raw = 0;
int16_t Pwm1A = 1500;
int16_t Pwm1B = 1500;
int16_t RCServo1 = 3000;
int16_t RCServo2 = 3000;
int16_t RCServo3 = 3000;
int16_t RCServo4 = 3000;

uint32_t ADCcount = 0;

int16_t receive_SPI = 0;
int16_t insideCommand = 0;
int16_t sendindex = 0;
int16_t sendvalues[3] = {0,0,0};
int16_t recvvalues[3] = {0,0,0};

uint32_t NoPWMSentTimeout = 0;
//
// Main
//
void main(void)
{
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
//    PieVectTable.TINT1 = &cpu_timer1_isr; //Just have timer1 trigger ADC
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
    ConfigCpuTimer(&CpuTimer1, 60, 500);  // Trigger ADC every .5ms
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
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;  // Put SPI in reset
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 1;
    SpiaRegs.SPICCR.bit.SPICHAR = 0xF;   // set to transmitt 16 bits
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 0;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;
    SpiaRegs.SPISTS.all=0x0000;
    SpiaRegs.SPIFFTX.bit.SPIRST = 1;
    SpiaRegs.SPIFFTX.bit.SPIFFENA = 1;
    SpiaRegs.SPIFFTX.bit.TXFIFO = 0;
    SpiaRegs.SPIFFTX.bit.TXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET = 0;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIENA = 1;
    SpiaRegs.SPIFFCT.all=0x00;
    SpiaRegs.SPIPRI.bit.FREE = 1;
    SpiaRegs.SPIPRI.bit.SOFT = 0;
    SpiaRegs.SPICCR.bit.SPISWRESET = 1;  // Pull the SPI out of reset
    SpiaRegs.SPIFFTX.bit.TXFIFO=1;
    SpiaRegs.SPIFFRX.bit.RXFIFORESET=1;
    SpiaRegs.SPICTL.bit.SPIINTENA = 1;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR = 1;
    SpiaRegs.SPIFFRX.bit.RXFFIL = 1;
    SpiaRegs.SPITXBUF = 0;  // Ready for first garbage send back.

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
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL  = 0x2;  //CPUTimer1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL  = 0x2;  //CPUTimer1
    AdcRegs.ADCSOC0CTL.bit.ACQPS    = 40;
    AdcRegs.ADCSOC1CTL.bit.ACQPS    = 40;
    EDIS;

    EPwm1Regs.TBCTL.bit.CTRMODE = 3;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;  // Divide by 1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  //I am thinking this will change Main clk to PWM to 60Mhz
    EPwm1Regs.TBPRD = 3000; //set epwm1 counter  20KHz
    EPwm1Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm1Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm1Regs.CMPA.half.CMPA = 1500;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm1Regs.CMPB = 1500;

    // Not using Epwm2 in ME461 F28027 but leaving it setup
    EPwm2Regs.TBCTL.bit.CTRMODE = 3;
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;  // Divide by 1
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;  //I am thinking this will change Main clk to PWM to 60Mhz
    EPwm2Regs.TBPRD = 3000; //set epwm2 counter  20KHz
    EPwm2Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm2Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm2Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm2Regs.CMPA.half.CMPA = 1500;

    EPwm1Regs.TBCTL.bit.CTRMODE = 0;      //set epwm1 to upcount mode  start pwm
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;      //set epwm2 to upcount mode

    EPwm3Regs.TBCTL.bit.CTRMODE = 0;      //set epwm3 to upcount mode
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm3Regs.TBCTL.bit.CLKDIV = 4;   // Divide by 16
    EPwm3Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm3Regs.TBPRD = 37500; //set epwm3 counter  50Hz for RCserve
    EPwm3Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm3Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm3Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
    EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
    EPwm3Regs.CMPA.half.CMPA = 3000;  // 8% duty cycle
    EPwm3Regs.CMPB = 3000;  // 8% duty cycle

    EPwm4Regs.TBCTL.bit.CTRMODE = 0;      //set epwm4 to upcount mode
    EPwm4Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm4Regs.TBCTL.bit.CLKDIV = 4;   // Divide by 16
    EPwm4Regs.TBCTL.bit.HSPCLKDIV = 1;
    EPwm4Regs.TBPRD = 37500; //set epwm4 counter  50Hz for RCserve
    EPwm4Regs.TBCTL.bit.PHSEN = 0;    // Disable phase loading
    EPwm4Regs.TBCTR = 0x0000;                  // Clear counter
    EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm4Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0
    EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareB
    EPwm4Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
    EPwm4Regs.CMPA.half.CMPA = 3000;  // 8% duty cycle
    EPwm4Regs.CMPB = 3000;  // 8% duty cycle

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM4A)
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM4A

    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4B
    EDIS;


    //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:
    //
    IER |= M_INT1;
    //IER |= M_INT13;  Do Not have CPU Timer 1 cause an interrupt.  Just Trigger ADC
    IER |= M_INT14;
    IER |= M_INT9;  // SCIA
    IER |= M_INT6;  // SPIA


    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; // Enable INT 1.1 ADCA1
    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE SPIA
    PieCtrlRegs.PIEIER6.bit.INTx1 = 1;  //Enable PIE 6.1 interrupt SPIA

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
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"A0=%d A2=%d SV1:%d,SV2:%d\r\n",ADC0raw,ADC2raw,Pwm1,Pwm2);  //Compiled printf minimal so only %d %x %u %c and maybe %s
            UARTPrint = 0;
        }
    }
}

__interrupt void adc_isr(void)
{

    ADC0raw = AdcResult.ADCRESULT0;
    ADC2raw = AdcResult.ADCRESULT1;

    ADCcount++;
    if ((ADCcount%500)==0) {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    NoPWMSentTimeout++;
    if (NoPWMSentTimeout > 200) {  // if NoPWMSent able to count up this far then No Receive for 100ms so set to zero
        EPwm1Regs.CMPA.half.CMPA = 1500;
        EPwm1Regs.CMPB = 1500;

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

// Using CPU Timer 1 to trigger ADC every 1ms
//
// cpu_timer1_isr - 
//
//__interrupt void cpu_timer1_isr(void)
//{
//    CpuTimer1.InterruptCount++;
//
//    //
//    // The CPU acknowledges the interrupt
//    //
//
//}

//
// cpu_timer2_isr - 
//
__interrupt void cpu_timer2_isr(void)
{  

    CpuTimer2.InterruptCount++;
    
    //
    // The CPU acknowledges the interrupt.
    //

}

// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;

}

//
// End of File
//

//Master will send    0x00DA;
//                    PWM1Value  0 to 3000
//                    PWM2Value  0 to 3000
//                    PWM3AValue  1500 to 4500
//                    PWM3BValue  1500 to 4500
//                    PWM4AValue  1500 to 4500
//                    PWM4BValue  1500 to 4500
//Master will recieve Garbage
//                    ADC0raw  0 to 4096
//                    ADC2raw  0 to 4096
//Master is first going to read the IMU then after IMU is read then go into second mode and read DAN28027
//Master will need two SPI states for interrupt reading IMU and reading DAN28027
__interrupt void SPI_RXint(void) {

#warn add RCserv1,2,3,4 later
    //debug
    //GpioDataRegs.GPASET.bit.GPIO7 = 1;
    receive_SPI = SpiaRegs.SPIRXBUF;

    //newprint = 1;
    if (insideCommand == 0) {

        if (receive_SPI == 0x00DA) {  // New command started

            insideCommand = 1;
            sendindex = 0;
            sendvalues[0] = ADC0raw;
            SpiaRegs.SPITXBUF = sendvalues[0];
            sendvalues[1] = ADC2raw;
            sendvalues[2] = 0;  // nothing
            sendindex++;

        } else {
            sendindex = 0;
            sendvalues[0] = 0;
            sendvalues[1] = 0;
            sendvalues[2] = 0;
        }

    } else {
        SpiaRegs.SPITXBUF = sendvalues[sendindex];
        recvvalues[sendindex-1] = receive_SPI;

        //            while (!(IFG2 & UCB0TXIFG));

        sendindex++;

        if (sendindex >= 3) {
            insideCommand = 0;
            Pwm1A = recvvalues[0];
            Pwm1B = recvvalues[1];
            if (Pwm1A > 3000) {
                Pwm1A = 3000;
            }
            if (Pwm1A < 0) {
                Pwm1A = 0;
            }
            if (Pwm1B > 3000) {
                Pwm1B = 3000;
            }
            if (Pwm1B < 0) {
                Pwm1B = 0;
            }
            EPwm1Regs.CMPA.half.CMPA = Pwm1A;
            EPwm1Regs.CMPB = Pwm1B;
            NoPWMSentTimeout = 0;  // reset timeout to zero every time we receive new Pwm values.
        }
    }

    //debug
    //GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
    SpiaRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpiaRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;   // Acknowledge interrupt to PIE


}
