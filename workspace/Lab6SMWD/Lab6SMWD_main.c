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
#include "F2837xD_SWPrioritizedIsrLevels.h"
#define FEETINONEMETER 3.28083989501312

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);


__interrupt void SWI1_HighestPriority(void);
__interrupt void SWI2_MiddlePriority(void);
__interrupt void SWI3_LowestPriority(void);

void setEPWM1A(float controleffort);
void setEPWM2A(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);

__interrupt void ADCC_ISR (void);

//SM ADDED
__interrupt void SPIB_isr(void);
void setupSpib(void);

void PostSWI1(void);
void PostSWI3(void);

uint32_t timecount = 0;

extern datapts ladar_data[228]; //distance data from LADAR

extern float printLV1;
extern float printLV2;

extern float LADARrightfront;
extern float LADARfront;

extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern char G_command[]; //command for getting distance -120 to 120 degree
extern uint16_t G_len; //length of command
extern xy ladar_pts[228]; //xy data

extern uint16_t LADARpingpong;
extern float LADARxoffset;
extern float LADARyoffset;

extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

extern uint16_t NewLVData;

uint16_t LADARi = 0;
pose ROBOTps = {0,0,0}; //robot position
pose LADARps = {3.5/12.0,0,1};  // 3.5/12 for front mounting, theta is not used in this current code
float printLinux1 = 0;
float printLinux2 = 0;


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

float sumgZ = 0.0;


int32_t adccCount = 0;
int32_t spibCount = 0;


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

float vref = 1.0;
float turn = 0.0;


float ref_right_wall = 1.6;
float left_turn_Start_threshold = 2.0;
float left_turn_Stop_threshold = 4.5;
float Kp_right_wall = -1;
float Kp_front_wall = -0.5;
float front_turn_velocity = 0.4;
float forward_velocity = 0.0;
float turn_command_saturation = 1.0;
int16_t right_wall_follow_state = 2;

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

uint16_t songIndex = 0;
int16_t gyroz_raw = 0;
int32_t ISRcount = 0;

int16_t spivalue1 = 0;
int16_t spivalue2 = 0;
int16_t spivalue3 = 0;
int16_t spivalue4 = 0;
int16_t spivalue5 = 0;
int16_t spivalue6 = 0;
int16_t spivalue7 = 0;
int16_t spivalue8 = 0;

float accelx = 0;
float accely = 0;
float accelz = 0;
//float temp = 0;
float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

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

    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

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

    //    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI1_HighestPriority;
    PieVectTable.RAM_CORRECTABLE_ERROR_INT = &SWI2_MiddlePriority;
    PieVectTable.FLASH_CORRECTABLE_ERROR_INT = &SWI3_LowestPriority;

    PieVectTable.SPIB_RX_INT = &SPIB_isr; //SM ADDED
    EDIS;    // This is needed to disable write to EALLOW protected registers



    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 100000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    DELAY_US(1000000);  // Delay 1 second giving LADAR Time to power on after system power on

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,19200);
    init_serialSCIC(&SerialC,19200);
    init_serialSCID(&SerialD,2083332);

    for (LADARi = 0; LADARi < 228; LADARi++) {
        ladar_data[LADARi].angle = ((3*LADARi+44)*0.3515625-135)*0.01745329; //0.017453292519943 is pi/180, multiplication is faster; 0.3515625 is 360/1024
    }

    EALLOW;                                  //SMWD Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;     //SMWD For EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;      //SMWD For EPWMA2
    EDIS;

    setupSpib();

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
    IER |= M_INT6;//SM IER 6

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    // Enable TINT0 in the PIE: Group 1 interrupt 3
    PieCtrlRegs.PIEIER1.bit.INTx3 = 1;
    //SM group 1 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    //    PieCtrlRegs.PIEIER12.bit.INTx9 = 1; //SWI1
    PieCtrlRegs.PIEIER12.bit.INTx10 = 1; //SWI2
    PieCtrlRegs.PIEIER12.bit.INTx11 = 1; //SWI3  Lowest priority

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    char S_command[19] = "S1152000124000\n";//this change the baud rate to 115200
    uint16_t S_len = 19;
    serial_sendSCIC(&SerialC, S_command, S_len);


    DELAY_US(1000000);  // Delay letting LADAR change its Baud rate
    init_serialSCIC(&SerialC,115200);
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
            //            UART_printfLine(2," 4Z %.2f, Z %.2f", int4Z, intZ);
            //            UART_printfLine(1,"v1 %.2f,v2 %.2f",v1, v2);
            //            UART_printfLine(2,"Vref %.2f,turn %.2f ",Vref, turn);
            UART_printfLine(1," gyroZ %.2f", gyroz);
            UARTPrint = 0;
        }
    }
}

//
// Connected to PIEIER12_9 (use MINT12 and MG12_9 masks):
//
__interrupt void SWI1_HighestPriority(void)     // EMIF_ERROR
{
    GpioDataRegs.GPBSET.bit.GPIO61 = 1;
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_9;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    uint16_t i = 0;//for loop
    if ((timecount < 2000)) {
    } else if(timecount < 4000){
        sum4Z += scaledc2;
        sum4X += scaledc1;
        sumZ += scaledc3;
        sumX += scaledc0;
        sumgZ += gyroz;
    } else if ((timecount == 4000)) {
        sum4Z /= 2000;
        sum4X /= 2000;
        sumZ /= 2000;
        sumX /= 2000;
        sumgZ /= 2000;

    } else {
        scaledc2 -= sum4Z;
        scaledc1 -= sum4X;
        scaledc0 -= sumX;
        scaledc3 -= sumZ;
        gyroz -= sumgZ;
        z4 = (scaledc2 ) * 100;
        x4 = (scaledc1) * 100;
        Z = (scaledc3 ) * 400;
        X = (scaledc0 ) * 400;


        //        uLeft = readEncWheel();
        //        uRight = readEncWheel();
        //        turn = readEncWheel()/4;
        // inside SWI1 before PI speed control

        switch (right_wall_follow_state) {
        case 1:
            //Left Turn
            turn = Kp_front_wall*(14.5 - LADARfront);
            vref = front_turn_velocity;
            if (LADARfront > left_turn_Stop_threshold) {
                right_wall_follow_state = 2;
            }
            break;
        case 2:
            //Right Wall Follow
            turn = Kp_right_wall*(ref_right_wall - LADARrightfront);
            vref = forward_velocity;
            if (LADARfront < left_turn_Start_threshold) {
                right_wall_follow_state = 1;
            }
            break;
        }
        if (turn > turn_command_saturation){
            turn = turn_command_saturation;
        }
        if (turn < -turn_command_saturation){
            turn = -turn_command_saturation;
        }
        // Add code here to saturate the turn command so that it is not larger
        // than turn_command_saturation or less than -turn_command_saturation

        p_current1 = left_feet;
        p_current2 = right_feet;
        v1 = (p_current1-p_old1) / 0.001;
        v2 = (p_current2-p_old2) / 0.001;
        errSteer = Kp_turn * (v2 - v1 + turn);
        errLeft =  vref - v1 + errSteer;
        errRight =  vref - v2 - errSteer;
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

    if (newLinuxCommands == 1) {
        newLinuxCommands = 0;
        vref = LinuxCommands[0];
        turn = LinuxCommands[1];
        ref_right_wall = LinuxCommands[2];
        left_turn_Start_threshold = LinuxCommands[3];
        left_turn_Stop_threshold = LinuxCommands[4];
        Kp_right_wall = LinuxCommands[5];
        Kp_front_wall = LinuxCommands[6];
        front_turn_velocity = LinuxCommands[7];
        forward_velocity = LinuxCommands[8];
        turn_command_saturation = LinuxCommands[9];
        //value11 = LinuxCommands[10];
    }


    if (NewLVData == 1) {
        NewLVData = 0;
        printLV1 = fromLVvalues[0];
        printLV2 = fromLVvalues[1];
    }

    if((timecount%250) == 0) {
        DataToLabView.floatData[0] = ROBOTps.x;
        DataToLabView.floatData[1] = ROBOTps.y;
        DataToLabView.floatData[2] = (float)timecount;
        DataToLabView.floatData[3] = ROBOTps.theta;
        DataToLabView.floatData[4] = ROBOTps.theta;
        DataToLabView.floatData[5] = ROBOTps.theta;
        DataToLabView.floatData[6] = ROBOTps.theta;
        DataToLabView.floatData[7] = ROBOTps.theta;
        LVsenddata[0] = '*';  // header for LVdata
        LVsenddata[1] = '$';
        for (i=0;i<LVNUM_TOFROM_FLOATS*4;i++) {
            if (i%2==0) {
                LVsenddata[i+2] = DataToLabView.rawData[i/2] & 0xFF;
            } else {
                LVsenddata[i+2] = (DataToLabView.rawData[i/2]>>8) & 0xFF;
            }
        }
        serial_sendSCID(&SerialD, LVsenddata, 4*LVNUM_TOFROM_FLOATS + 2);
    }



    timecount++;


    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
    //##############################################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_10 (use MINT12 and MG12_10 masks):
//
__interrupt void SWI2_MiddlePriority(void)     // RAM_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_10;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    if (LADARpingpong == 1) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_ping;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_ping < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_ping;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_ping*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_ping*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    } else if (LADARpingpong == 0) {
        // LADARrightfront is the min of dist 52, 53, 54, 55, 56
        LADARrightfront = 19; // 19 is greater than max feet
        for (LADARi = 52; LADARi <= 56 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARrightfront) {
                LADARrightfront = ladar_data[LADARi].distance_pong;
            }
        }
        // LADARfront is the min of dist 111, 112, 113, 114, 115
        LADARfront = 19;
        for (LADARi = 111; LADARi <= 115 ; LADARi++) {
            if (ladar_data[LADARi].distance_pong < LADARfront) {
                LADARfront = ladar_data[LADARi].distance_pong;
            }
        }
        LADARxoffset = ROBOTps.x + (LADARps.x*cosf(ROBOTps.theta)-LADARps.y*sinf(ROBOTps.theta - PI/2.0));
        LADARyoffset = ROBOTps.y + (LADARps.x*sinf(ROBOTps.theta)-LADARps.y*cosf(ROBOTps.theta - PI/2.0));
        for (LADARi = 0; LADARi < 228; LADARi++) {

            ladar_pts[LADARi].x = LADARxoffset + ladar_data[LADARi].distance_pong*cosf(ladar_data[LADARi].angle + ROBOTps.theta);
            ladar_pts[LADARi].y = LADARyoffset + ladar_data[LADARi].distance_pong*sinf(ladar_data[LADARi].angle + ROBOTps.theta);

        }
    }




    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

}

//
// Connected to PIEIER12_11 (use MINT12 and MG12_11 masks):
//
__interrupt void SWI3_LowestPriority(void)     // FLASH_CORRECTABLE_ERROR
{
    // Set interrupt priority:
    volatile Uint16 TempPIEIER = PieCtrlRegs.PIEIER12.all;
    IER |= M_INT12;
    IER    &= MINT12;                          // Set "global" priority
    PieCtrlRegs.PIEIER12.all &= MG12_11;  // Set "group"  priority
    PieCtrlRegs.PIEACK.all = 0xFFFF;    // Enable PIE interrupts
    __asm("  NOP");
    EINT;

    //###############################################################################################
    // Insert SWI ISR Code here.......

    //###############################################################################################
    //
    // Restore registers saved:
    //
    DINT;
    PieCtrlRegs.PIEIER12.all = TempPIEIER;

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
    serial_sendSCIC(&SerialC, G_command, G_len);
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
//SM Added
__interrupt void SPIB_isr(void){
    //SE423 E3
    //    spivalue1 = SpibRegs.SPIRXBUF; //SM Read first 16 bit value off RX FIFO. Probably is zero since no chip
    //    spivalue2 = SpibRegs.SPIRXBUF; //SM Read second 16 bit value off RX FIFO. Again probably zero
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //SM Set GPIO 66 high to end Slave Select. Now to Scope. Later to deselect MPU9250.  
    spivalue1 = SpibRegs.SPIRXBUF;
    spivalue2 = SpibRegs.SPIRXBUF;
    spivalue3 = SpibRegs.SPIRXBUF;
    spivalue4 = SpibRegs.SPIRXBUF;
    spivalue5 = SpibRegs.SPIRXBUF;
    spivalue6 = SpibRegs.SPIRXBUF;
    spivalue7 = SpibRegs.SPIRXBUF;
    spivalue8 = SpibRegs.SPIRXBUF;

    accelx = (spivalue2/32767.0)*4.0;
    accely = (spivalue3/32767.0)*4.0;
    accelz = (spivalue4/32767.0)*4.0;
    //        temp = 0;
    gyrox = (spivalue6/32767.0)*250.0;
    gyroy = (spivalue7/32767.0)*250.0;
    gyroz = (spivalue8/32767.0)*250.0;


    PostSWI1();

    spibCount++;
    // Later when actually communicating with the MPU9250 do something with the data. Now do nothing.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt
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

    adccCount++;

    //  if ((numTimer0calls%50) == 0) {
    //    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;    // Manually cause the interrupt for the SWI
    //  }


    //Clear GPIO66 Low to act as a Slave Select. Right now, just to scope. Later to select MPU9250 chip
    //    GpioDataRegs.GPCSET.bit.GPIO66 = 1;  //SM set gpio66 on
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;  //SM clear gpio66
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when two values are in the RX FIFO

    //SM E5
    SpibRegs.SPITXBUF = (0x8000 | 0x3A00);
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;



    if ((adccCount % 100) == 0) {
        UARTPrint = 1;
    }


    //
    //
    //    if ((adccCount % 200) == 0) {
    //        UARTPrint = 1;
    //    }


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
void PostSWI1(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI1
}
void PostSWI3(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1; // Manually cause the interrupt for the SWI3
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


void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Also do not forget to cut and paste the GPIO settings for GPIO63, 64, 65, 66 which are also a part of the SPIB setup.
    //-----------------------------------------------------------------------------------------------------------------
    //SM E3
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB    //SM CHANGED
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB    //SM CHANGED
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB    //SM CHANGED
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for    SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for    SPI
    EDIS;
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0x0; // Put SPI in Reset    //SM CHANGED
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 0x1; // Set to SPI Master    //SM CHANGED
    SpibRegs.SPICCR.bit.SPICHAR = 0xF; // Set to transmit and receive 16 bits each write to SPITXBUF      //SM CHANGED
    SpibRegs.SPICTL.bit.TALK = 0x1; // Enable transmission      //SM CHANGED
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0x0; // Disables the SPI interrupt    //SM CHANGED
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is    //SM CHANGED
    // 50MHZ. And this setting divides that base clock to create SCLKs period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 0x1;// Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.    //SM CHANGED
    SpibRegs.SPIFFTX.bit.SPIFFENA = 0x1; // Enable SPI FIFO enhancements    //SM CHANGED
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 0x1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set    //SM CHANGED
    SpibRegs.SPIFFRX.bit.RXFFIENA = 0x1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL    //SM CHANGED
    SpibRegs.SPIFFCT.bit.TXDLY = 0x0; //Set delay between transmits to 0 spi clocks.    //SM CHANGED
    SpibRegs.SPICCR.bit.SPISWRESET = 0x1; // Pull the SPI out of reset    //SM CHANGED
    SpibRegs.SPIFFTX.bit.TXFIFO = 0x1; // Release transmit FIFO from reset.    //SM CHANGED
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I dont think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 0x10; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below      //SM CHANGED

    //Step 2.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself. GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16 bit transfers, so two registers at a time after the first 16 bit transfer.
    // To address 00x13 write 0x00
    SpibRegs.SPITXBUF= 0x1300;
    // To address 00x14 write 0x00
    // To address 00x15 write 0x00
    SpibRegs.SPITXBUF= 0x0000;
    // To address 00x16 write 0x00
    // To address 00x17 write 0x00
    SpibRegs.SPITXBUF= 0x0000;
    // To address 00x18 write 0x00
    // To address 00x19 write 0x13
    SpibRegs.SPITXBUF= 0x0013;
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    SpibRegs.SPITXBUF= 0x0200;
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    SpibRegs.SPITXBUF= 0x0806;
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    SpibRegs.SPITXBUF= 0x0000;
    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST != 7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 3.
    // perform a multiple 16 bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x2300;
    // To address 00x24 write 0x40
    // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x408C;
    // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    SpibRegs.SPITXBUF = 0x0288;
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    SpibRegs.SPITXBUF = 0x0C0A;

    // wait for the correct number of 16 bit values to be received into the RX FIFO
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    // read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 4.
    // perform a single 16 bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    // Write to address 0x2A the value 0x81
    SpibRegs.SPITXBUF = (0x2A | 0x81);
    // wait for one byte to be received
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0001); // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700 Xoff H
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0046); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x00EC); // 0x7A00 Y off
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x0012); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0019); // 0x7D00 Z off
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x00E8); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}


#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h> // for sched_yield()
#include <assert.h>
#include <sys/resource.h>
#include <math.h>
#include <dirent.h>
#include <sys/time.h>
#include <errno.h>
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>
#include <ctype.h>

int gs_quit = 0;
int gs_exit = 0;

float vref = 1.0;
float turn = 0.0;
float ref_right_wall = 0.0;
float left_turn_Start_threshold = 0.0;
float left_turn_Stop_threshold = 0.0;
float Kp_right_wall = 0.1;
float Kp_front_wall = 0.1;
float front_turn_velocity = 0.4;
float forward_velocity = 0.0;
float turn_command_saturation = 1.0;

int sem_count_send = 0; //for sem_getvalue
#define CMDNUM_FROM_FLOATS 11
#define RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME "/sem-LINUXCMDApp-recvfrom"
#define RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME "/sharedmem-LINUXCMDApp-recvfrom"

typedef union {
    char data_char[4*CMDNUM_FROM_FLOATS];
    float data_flts[CMDNUM_FROM_FLOATS];
} int_FromCMD_union;

struct shared_memory_recvfrom_LINUXCMDApp
{
  int_FromCMD_union new_FromCMD;
};

struct shared_memory_recvfrom_LINUXCMDApp *shared_mem_ptr_recvfrom_LINUXCMDApp;
sem_t *recvfrom_LINUXCMDApp_mutex_sem;
int recvfrom_LINUXCMDApp_fd_shm;


int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}


// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}

/*  
* gs_killapp()
*   ends application safely
*
*/
void gs_killapp(int s)
{
	printf("\nTerminating\n");
	gs_quit = 1;
	gs_exit = 1;
	return;
}

/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int i = 0;

	char buffer[200];  // used by fgets to read character string typed by user.
	char mychar;
	float tempfloat = 0;
	
	fflush(stdout);
	 
  	
    //create the semaphore for recv 
    if ((recvfrom_LINUXCMDApp_mutex_sem = sem_open(RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("Error recv LINUXCMDApp sem_open");

    // create shared memory for recv
    if ((recvfrom_LINUXCMDApp_fd_shm = shm_open(RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("Error shm_open LINUXCMDApp");

    //map the memory to virtual address
    if ((shared_mem_ptr_recvfrom_LINUXCMDApp = mmap(NULL, sizeof(struct shared_memory_recvfrom_LINUXCMDApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                recvfrom_LINUXCMDApp_fd_shm, 0)) == MAP_FAILED)
        error("Error mmap LINUXCMDApp");

	printf("Setting signal handler...\n");
	signal(SIGKILL, gs_killapp);
	signal(SIGINT, gs_killapp);
	printf("...OK\n");
	printf(".\n");
	while (!gs_exit) {
		sched_yield();

		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		printf("e - Exit Application\n");
		printf("s - enter Desired Velocity Setpoint and RightWall speed (ft/s)\n");
		printf("q - increment Left\n");
		printf("p - increment Right\n");
		printf("z - ref_right_wall\n");
		printf("x - left_turn_Start_threshold\n");
		printf("c - left_turn_Stop_threshold\n");
		printf("v - Kp_right_wall\n");
		printf("b - Kp_front_wall\n");
		printf("n - front_turn_velocity\n");
		printf("m - forward_velocity\n");
		printf(", - turn_command_saturation\n");
		printf("l - List All Parameters\n");

		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'q':
			if (turn > 0.0) {
				turn = 0.0;
			} else {
				turn = turn - 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'p':                                
			if (turn < 0.0) {
				turn = 0.0;
			} else {
				turn = turn + 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'e':
			gs_exit = 1;
			break;
		case 's':
			printf("Enter Desired Velocity (vref) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					vref = tempfloat;
					printf("DVel = %.3f\n",vref);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;
		case 'z':
			printf("Enter Desired ref_right_wall (ref_right_wall) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					ref_right_wall = tempfloat;
					printf("desired ref_right_wall = %.3f\n",ref_right_wall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: ref_right_wall not changed\n");
			}
			
			break;	
		case 'x':
			printf("Enter Desired left_turn_Start_threshold (left_turn_Start_threshold) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					left_turn_Start_threshold = tempfloat;
					printf("desired left_turn_Start_threshold = %.3f\n",left_turn_Start_threshold);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: left_turn_Start_threshold not changed\n");
			}
			
			break;
		case 'c':
			printf("Enter Desired left_turn_Stop_threshold and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					left_turn_Stop_threshold = tempfloat;
					printf("desired left_turn_Stop_threshold = %.3f\n",left_turn_Stop_threshold);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: left_turn_Stop_threshold not changed\n");
			}
			
			break;
		case 'v':
			printf("Enter Desired Kp_right_wall and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_right_wall = tempfloat;
					printf("desired Kp_right_wall = %.3f\n",Kp_right_wall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: Kp_right_wall not changed\n");
			}
			
			break;	
		case 'b':
			printf("Enter Desired Kp_front_wall and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					Kp_front_wall = tempfloat;
					printf("desired Kp_front_wall = %.3f\n",Kp_front_wall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: Kp_front_wall not changed\n");
			}
			
			break;	
		case 'n':
			printf("Enter Desired front_turn_velocity and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					front_turn_velocity = tempfloat;
					printf("desired front_turn_velocity = %.3f\n",front_turn_velocity);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: front_turn_velocity not changed\n");
			}
			
			break;	
		case 'm':
			printf("Enter Desired forward_velocity and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					forward_velocity = tempfloat;
					printf("desired forward_velocity = %.3f\n",forward_velocity);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: forward_velocity not changed\n");
			}
			
			break;	
		case ',':
			printf("Enter Desired turn_command_saturation and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					turn_command_saturation = tempfloat;
					printf("desired turn_command_saturation = %.3f\n",turn_command_saturation);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: turn_command_saturation not changed\n");
			}
			
			break;	
		// case '.':
		// 	printf("Enter Desired Velocity (vref) and Right Wall velocity\n");
		// 	fgets(buffer,190,stdin); 
		// 	buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
		// 	if (buffer[0] != '\0') {
		// 		if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
		// 			vref = tempfloat;
		// 			printf("DVel = %.3f\n",vref);
		// 		}  else {
		// 			printf("Error: Non numerical value typed\n");
		// 		}
		// 	} else {
		// 		printf("Error: vref not changed\n");
		// 	}
			
		// 	break;	
		case 'l':
			printf("\n");
			printf("turn = %.3f\n",turn);
			printf("vref = %.3f\n",vref);
			printf("ref_right_wall = %.3f\n",ref_right_wall);
			printf("left_turn_Start_threshold = %.3f\n",left_turn_Start_threshold);
			printf("left_turn_Stop_threshold = %.3f\n",left_turn_Stop_threshold);
			printf("Kp_right_wall = %.3f\n",Kp_right_wall);
			printf("Kp_front_wall = %.3f\n",Kp_front_wall);
			printf("front_turn_velocity = %.3f\n",front_turn_velocity);
			printf("forward_velocity = %.3f\n",forward_velocity);
			printf("turn_command_saturation = %.3f\n",turn_command_saturation);

			break;	
		default:
			
			break;
		}
		
		shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[0] = vref;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[1] = turn;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[2] = ref_right_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[3] = left_turn_Start_threshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[4] = left_turn_Stop_threshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[5] = Kp_right_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[6] = Kp_front_wall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[7] = front_turn_velocity;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[8] = forward_velocity;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[9] = turn_command_saturation;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[10] = 0;

		if (sem_getvalue(recvfrom_LINUXCMDApp_mutex_sem,  &sem_count_send) == 0) {
			if (sem_post(recvfrom_LINUXCMDApp_mutex_sem) == -1){
				printf("Error LINUXCMDApp sem_post: recvfrom_mutex");
			}
		}
		
	}

}



