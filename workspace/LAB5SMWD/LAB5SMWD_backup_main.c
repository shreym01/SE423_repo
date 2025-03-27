//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
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


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

void setEPWM1A(float controleffort);
void setEPWM2A(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);

__interrupt void ADCC_ISR (void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;

int16_t adcc0result = 0;
int16_t adcc1result = 0;
int16_t adcc2result = 0;
int16_t adcc3result = 0;

float scaledc0 = 0;
float scaledc1 = 0;
float scaledc2 = 0;
float scaledc3 = 0;

float z4 = 0.0;
float x4 = 0.0;
float Z = 0.0;
float X = 0.0;

float sum4Z = 0.0;
float sum4X = 0.0;
float sumZ = 0.0;
float sumX = 0.0;

int32_t adccCount = 0;


float LeftWheel = 0;
float RightWheel = 0;
float left_feet = 0;
float right_feet = 0;
float uLeft = 0;
float uRight= 0;
float enc = 0;
float p_old1 = 0;
float p_current1= 0;
float p_old2 = 0;
float p_current2= 0;
float v1= 0;
float v2= 0;
float Vpos = 2.3;
float Vneg = 2.2;
float Cpos = 2.116;
float Cneg = -2.116;

float Vref = 1.0;
float turn = 0.0;
float Ki = 15.0;
float Kp = 3.0;
float Kp_turn = 3.0;

float errKLeft = 0;
float IKLeft = 0;
float IKLeft_old = 0;
float errLeft = 0;
float errLeft_old = 0;

float errKRight = 0;
float IKRight = 0;
float IKRight_old = 0;
float errRight = 0;
float errRight_old = 0;

float int4Z = 0.0;
float intZ = 0.0;

float int4Z_old = 0.0;
float intZ_old = 0.0;

float z4_old = 0.0;
float Z_old = 0.0;

float errSteer = 0.0;

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
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5);     //SMWD change to EPwm12A
    //    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
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


    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO2 = 1;

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

    PieVectTable.ADCC1_INT = &ADCC_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers



    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
    //    init_serialSCIC(&SerialC,115200);
    //    init_serialSCID(&SerialD,115200);

    EALLOW;                                  //SMWD Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;     //SMWD For EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;      //SMWD For EPWMA2
    EDIS;
    EALLOW;
    //SMWD PWM2
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;   //SMWD Count mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT= 2;  //SMWD Free Soft mode to Free run
    EPwm2Regs.TBCTL.bit.CLKDIV= 0;      //SMWD set clock divide 1
    EPwm2Regs.TBCTL.bit.PHSEN = 0;      //SMWD disable Phase loading

    EPwm2Regs.TBCTR = 0;                //SMWD set timer to 0
    EPwm2Regs.TBPRD = 2500;             //SMWD Carrier Frequency
    EPwm2Regs.CMPA.bit.CMPA = 1250;      //SMWD duty Cycle 50%
    EPwm2Regs.AQCTLA.bit.CAU=1;         //SMWD PWM clear signla
    EPwm2Regs.AQCTLA.bit.ZRO = 2;       //SMWD Have pin be set wen register is zro
    EPwm2Regs.TBPHS.bit.TBPHS = 0;      //SMWD ignore phase

    //SMWDPWM1
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;   //SMWD Count mode
    EPwm1Regs.TBCTL.bit.FREE_SOFT= 2;  //SMWD Free Soft mode to Free run
    EPwm1Regs.TBCTL.bit.CLKDIV= 0;      //SMWD set clock divide 1
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      //SMWD disable Phase loading

    EPwm1Regs.TBCTR = 0;                //SMWD set timer to 0
    EPwm1Regs.TBPRD = 2500;             //SMWD Carrier Frequency
    EPwm1Regs.CMPA.bit.CMPA = 1250;      //SMWD duty Cycle 50%
    EPwm1Regs.AQCTLA.bit.CAU=1;         //SMWD PWM clear signla
    EPwm1Regs.AQCTLA.bit.ZRO = 2;       //SMWD Have pin be set wen register is zro
    EPwm1Regs.TBPHS.bit.TBPHS = 0;      //SMWD ignore phase



    EPwm4Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm4Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm4Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm4Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm4Regs.TBCTR = 0x0; // Clear counter
    EPwm4Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm4Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm4Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm4Regs.TBPRD = 50000; // Set Period to 1ms sample. Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm4Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm4Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //ADCC
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2; //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3; //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4; //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5; //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 0x0B; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;
    init_eQEPs();
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
    // Enable TINT0 in the PIE: Group 1 interrupt 3
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;

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
//            UART_printfLine(1,"x4 %.1f, z4 %.1f", x4, z4);
//            UART_printfLine(2," X %.1f, Z %.1f", X, Z);
//            UART_printfLine(1,"x4 %.1f, z4 %.1f", x4, z4);
            UART_printfLine(2," 4Z %.2f, Z %.2f", int4Z, intZ);
//            UART_printfLine(1,"v1 %.2f,v2 %.2f",v1, v2);
//            UART_printfLine(2,"Vref %.2f,turn %.2f ",Vref, turn);
            UARTPrint = 0;
        }
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

//    if ((CpuTimer2.InterruptCount % 10) == 0) {
//        UARTPrint = 1;
//    }
}
__interrupt void ADCC_ISR (void) {

    LeftWheel = readEncLeft();
    RightWheel = readEncRight();
    left_feet  = LeftWheel / 9.95;
    right_feet = RightWheel / 9.95;


    adcc0result = AdccResultRegs.ADCRESULT0;
    adcc1result = AdccResultRegs.ADCRESULT1;
    adcc2result = AdccResultRegs.ADCRESULT2;
    adcc3result = AdccResultRegs.ADCRESULT3;

    scaledc0 = adcc0result * 3.0/4096.0;
    scaledc1 = adcc1result * 3.0/4096.0;
    scaledc2 = adcc2result * 3.0/4096.0;
    scaledc3 = adcc3result * 3.0/4096.0;

    //    z4 = (scaledc2 - 1.23) * 100;
    //    x4 = (scaledc1 - 1.23) * 100;
    //    Z = (scaledc3 - 1.23) * 400;
    //    X = (scaledc0 - 1.23) * 400;
    // Print xk and yk’s voltage value to LCD every 100ms
    //    if ((adccCount % 250) == 0) {
    //        UARTPrint = 1;
    //    }

    if ((adccCount < 2000)) {

    } else if(adccCount < 4000){
        sum4Z += scaledc2;
        sum4X += scaledc1;
        sumZ += scaledc3;
        sumX += scaledc0;
    } else if ((adccCount == 4000)) {
        sum4Z /= 2000;
        sum4X /= 2000;
        sumZ /= 2000;
        sumX /= 2000;

    } else {
        scaledc2 -= sum4Z;
        scaledc1 -= sum4X;
        scaledc0 -= sumX;
        scaledc3 -= sumZ;
        z4 = (scaledc2 ) * 100;
        x4 = (scaledc1) * 100;
        Z = (scaledc3 ) * 400;
        X = (scaledc0 ) * 400;

//        uLeft = readEncWheel();
//        uRight = readEncWheel();
        turn = readEncWheel()/4;

        p_current1 = left_feet;
        p_current2 = right_feet;
        v1 = (p_current1-p_old1) / 0.001;
        v2 = (p_current2-p_old2) / 0.001;
        errSteer = Kp_turn * (v2 - v1 + turn);
        errLeft =  Vref - v1 + errSteer;
        errRight =  Vref - v2 - errSteer;
        IKLeft = IKLeft_old + ((errLeft + errLeft_old)*0.5) * 0.001;
        IKRight = IKRight_old + ((errRight + errRight_old)*0.5) * 0.001;
        uLeft = Kp*errKLeft + Ki*IKLeft;
        uRight = Kp*errKRight + Ki*IKRight;

        int4Z = int4Z_old + ((z4 + z4_old)*0.5)* 0.001;
        intZ = intZ_old + ((Z + Z_old)*0.5)* 0.001;

        if (v1 > 0.0) {
            uLeft = uLeft +0.6 * Vpos * v1 + 0.6* Cpos;
        } else {
            uLeft = uLeft + 0.6* Vneg * v1 + 0.6*Cneg;
        }
        if (v2 > 0.0) {
            uRight = uRight + 0.6* Vpos * v2 + 0.6*Cpos;
        } else {
            uRight = uRight + 0.6*Vneg * v2  + 0.6*Cneg;
        }

        if (fabs(uLeft) > 10){
            IKLeft = IKLeft_old*.99;
        }
        if (fabs(uRight) > 10){
            IKRight = IKRight_old*.99;
        }


        setEPWM1A(uLeft);
        setEPWM2A(-uRight);

        errLeft_old = errLeft;
        errRight_old = errRight;

        IKLeft_old = IKLeft;
        IKRight_old = IKRight;

        p_old1 = p_current1;
        p_old2 = p_current2;

        z4_old = z4;
        Z_old = Z;

        int4Z_old = int4Z;
        intZ_old = intZ;
    }


    if ((adccCount % 200) == 0) {
        UARTPrint = 1;
    }


    adccCount++;
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

void setEPWM1A(float controleffort){
    if(controleffort <= -10) {
        controleffort = -10;
    }
    if(controleffort >= 10) {
        controleffort = 10;
    }
    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD * (0.05 * controleffort + 0.5) ;

}
void setEPWM2A(float controleffort){
    if(controleffort <= -10) {
        controleffort = -10;
    }
    if(controleffort >= 10) {
        controleffort = 10;
    }
    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.TBPRD * (0.05 * controleffort + 0.5) ;
}

void init_eQEPs(void) {
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1; // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1; // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (-raw*(2*PI/40000.0));
}
float readEncRight(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (raw*(2*PI/40000.0));
}
float readEncWheel(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep3Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue; // I don't think this is needed and never true
    return (-raw*(2*PI/4000.0));
}
