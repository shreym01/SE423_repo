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

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312

//This function sets DACA to the voltage between 0V and 3V passed to this function.
//If outside 0V to 3V the output is saturated at 0V to 3V
//Example code
//float myu = 2.25;
//setDACA(myu);   // DACA will now output 2.25 Volts
void setDACA(float dacouta0) {
    if (dacouta0 >  3.0) dacouta0 =  3.0;
    if (dacouta0 < 0.0) dacouta0 = 0.0;

    DacaRegs.DACVALS.bit.DACVALS = dacouta0*4095.0/3.0;  // perform scaling of 0-3 to 0-4095
}

void setDACB(float dacouta1) {
    if (dacouta1 >  3.0) dacouta1 =  3.0;
    if (dacouta1 < 0.0) dacouta1 = 0.0;

    DacbRegs.DACVALS.bit.DACVALS = dacouta1*4095.0/3.0;  // perform scaling of 0-3 to 0-4095
}

uint16_t readbuttons(void) {
    uint16_t returnvalue = 0;

    if (GpioDataRegs.GPEDAT.bit.GPIO157 == 0) {
        returnvalue |= 1;
    }
    if (GpioDataRegs.GPEDAT.bit.GPIO158 == 0) {
        returnvalue |= 2;
    }
    if (GpioDataRegs.GPEDAT.bit.GPIO159 == 0) {
        returnvalue |= 4;
    }
    if (GpioDataRegs.GPFDAT.bit.GPIO160 == 0) {
        returnvalue |= 8;
    }

    return(returnvalue);
}

void setEPWM1A(float controleffort){
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort > 10) {
        controleffort = 10;
    }
    uint16_t period = EPwm1Regs.TBPRD;
    float value = (controleffort+10)*period/20.0;
    EPwm1Regs.CMPA.bit.CMPA = value;
}
void setEPWM2A(float controleffort){
    if (controleffort < -10) {
        controleffort = -10;
    }
    if (controleffort > 10) {
        controleffort = 10;
    }
    uint16_t period = EPwm2Regs.TBPRD;
    float value = (controleffort+10)*period/20.0;
    EPwm2Regs.CMPA.bit.CMPA = value;
}

void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;    // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;    // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2;   // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0;    // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0;   // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0;      // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0;      // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0;        // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;   // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1;    // Enable EQep

    EALLOW;
    // setup QEP2 pins for input
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;    // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;    // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO55 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep

    EALLOW;
    // setup QEP3 pins for input
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO54 (EQEP3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO55 (EQEP3B)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 2;   // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 2;   // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 5); // set GPIO6 and eQep2A
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 5); // set GPIO7 and eQep2B
    EQep3Regs.QEPCTL.bit.QPEN = 0;   // make sure qep reset
    EQep3Regs.QDECCTL.bit.QSRC = 0;  // Quadrature count mode
    EQep3Regs.QPOSCTL.all = 0x0;     // Disable eQep Position Compare
    EQep3Regs.QCAPCTL.all = 0x0;     // Disable eQep Capture
    EQep3Regs.QEINT.all = 0x0;       // Disable all eQep interrupts
    EQep3Regs.QPOSMAX = 0xFFFFFFFF;  // use full range of the 32 bit count.
    EQep3Regs.QEPCTL.bit.FREE_SOFT = 2;  // EQep uneffected by emulation suspend
    EQep3Regs.QPOSCNT = 0;
    EQep3Regs.QEPCTL.bit.QPEN = 1;   // Enable EQep

}

float readEncLeft(void) {
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    return (raw*(-2*PI/(2000.0*20.0)));
}

float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    return (raw*(2*PI/(2000.0*20.0)));
}

float readEncWheel(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U  -1 32bit signed int

    raw = EQep3Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;  // I don't think this is needed and never true

    return (raw*(2*PI/4000.0));
}

uint16_t dummyread = 0;
void setupSpib(void)        //for mpu9250
{

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    SpibRegs.SPICCR.bit.SPISWRESET   = 0;    // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE    = 1;
    SpibRegs.SPICCR.bit.CLKPOLARITY  = 0;

    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;    // SPI master
    SpibRegs.SPICCR.bit.SPICHAR      = 0xF;  // Set to transmit 16 bits
    SpibRegs.SPICTL.bit.TALK         = 1;    // Enables transmission for the 4-pin option
    SpibRegs.SPIPRI.bit.FREE         = 1;    // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA    = 0;    // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49;   // 50MHz/(49+1) = 1MHz

    SpibRegs.SPISTS.all              = 0x0000;

    SpibRegs.SPIFFTX.bit.SPIRST      = 1;    // SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA    = 1;    // SPI FIFO enhancements are enabled
    SpibRegs.SPIFFTX.bit.TXFIFO      = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR  = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFOVF]
    SpibRegs.SPIFFRX.bit.RXFFINTCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpibRegs.SPIFFRX.bit.RXFFIENA    = 1;    // RX FIFO interrupt based on RXFFIL match

    SpibRegs.SPIFFCT.bit.TXDLY       = 0; // The next word in the TX FIFO buffer is transferred to SPITXBUF immediately upon completion of transmission of the previous word.

    SpibRegs.SPICCR.bit.SPISWRESET   = 1;    // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO      = 1;    // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA    = 1;    // Enables the SPI interrupt.
    SpibRegs.SPIFFRX.bit.RXFFIL      = 16;    // A RX FIFO interrupt request is generated when there are 1 or more words in the RX buffer.

    //-----------------------------------------------------------------------------------------------------------------

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x1300 | 0x0000); // 0x1300 (0x0000)
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // 0x1400 (0x0000),       0x1500 (0x0000)
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // 0x1600 (0x0000),       0x1700 (0x0000)
    SpibRegs.SPITXBUF = (0x0000 | 0x0013); // 0x1800 (0x0000),       0x1900 (0x0013)
    SpibRegs.SPITXBUF = (0x0200 | 0x0000); // 0x1A00 (0x0002),       0x1B00 (0x0000), 250 dps
    SpibRegs.SPITXBUF = (0x0800 | 0x0006); // 0x1C00 (0x0008), +-4g, 0x1D00 (0x0006), 5Hz
    SpibRegs.SPITXBUF = (0x0000 | 0x0000); // 0x1E00 (0x0000),       0x1F00 (0x0000)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x2300 | 0x0000); // 0x2300 (0x0000)
    SpibRegs.SPITXBUF = (0x4000 | 0x008C); // 0x2400 (0x0040),       0x2500 (0x008C)
    SpibRegs.SPITXBUF = (0x0200 | 0x0088); // 0x2600 (0x0002),       0x2700 (0x0088)
    SpibRegs.SPITXBUF = (0x0C00 | 0x000A); // 0x2800 (0x000C),       0x2900 (0x000A)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x2A00 | 0x0081);  // 0x2A00 (0x0081)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001);  // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001);  // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001);  // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020);  // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071);  // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00FA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    dummyread = SpibRegs.SPIRXBUF;


    DELAY_US(10);

    while(SpibRegs.SPIFFRX.bit.RXFFST !=0) {
        dummyread = SpibRegs.SPIRXBUF;
    }

}

void init_RCServoPWM_3AB_5AB_6A(void) {

    //epwm3AB
    EPwm3Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm3Regs.TBCTL.bit.FREE_SOFT = 3; // free run mode
    EPwm3Regs.TBCTL.bit.CLKDIV = 4; //divide by 16
    EPwm3Regs.TBCTL.bit.PHSEN = 0; //disable the phase loading
    EPwm3Regs.TBCTR = 0; //Start the timers at zero.
    EPwm3Regs.TBPRD = 62500;
    EPwm3Regs.CMPA.bit.CMPA = 5000; //start the duty cycle at 8%.
    EPwm3Regs.CMPB.bit.CMPB = 5000;
    EPwm3Regs.AQCTLA.bit.CAU = 1; //set it up such that the signal is cleared when CMPA is reached.
    EPwm3Regs.AQCTLA.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm3Regs.AQCTLB.bit.CBU = 1; //set it up such that the signal is cleared when CMPB is reached.
    EPwm3Regs.AQCTLB.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm3Regs.TBPHS.bit.TBPHS = 0;

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1; // For EPWM3A
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1; // For EPWM3B
    EDIS;

    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 1);  //EPWM3A
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 1);  //EPWM3B


    //epwm5AB
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm5Regs.TBCTL.bit.FREE_SOFT = 3; // free run mode
    EPwm5Regs.TBCTL.bit.CLKDIV = 4; //divide by 16
    EPwm5Regs.TBCTL.bit.PHSEN = 0; //disable the phase loading
    EPwm5Regs.TBCTR = 0; //Start the timers at zero.
    EPwm5Regs.TBPRD = 62500;
    EPwm5Regs.CMPA.bit.CMPA = 5000; //start the duty cycle at 8%.
    EPwm5Regs.CMPB.bit.CMPB = 5000;
    EPwm5Regs.AQCTLA.bit.CAU = 1; //set it up such that the signal is cleared when CMPA is reached.
    EPwm5Regs.AQCTLA.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm5Regs.AQCTLB.bit.CBU = 1; //set it up such that the signal is cleared when CMPB is reached.
    EPwm5Regs.AQCTLB.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm5Regs.TBPHS.bit.TBPHS = 0;

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1; // For EPWM5A
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1; // For EPWM5B
    EDIS;

    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 1);  //EPWM5A
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 1);  //EPWM5B

    //epwm6A
    EPwm6Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 3; // free run mode
    EPwm6Regs.TBCTL.bit.CLKDIV = 4; //divide by 16
    EPwm6Regs.TBCTL.bit.PHSEN = 0; //disable the phase loading
    EPwm6Regs.TBCTR = 0; //Start the timers at zero.
    EPwm6Regs.TBPRD = 62500;
    EPwm6Regs.CMPA.bit.CMPA = 5000; //start the duty cycle at 8%.
    EPwm6Regs.AQCTLA.bit.CAU = 1; //set it up such that the signal is cleared when CMPA is reached.
    EPwm6Regs.AQCTLA.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm6Regs.TBPHS.bit.TBPHS = 0;

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1; // For EPWM6A
    EDIS;

    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);  //EPWM6A

}

void setEPWM3A_RCServo(float angle) {
    if (angle > 90) {
        angle = 90;
    }
    if (angle < -90) {
        angle = -90;
    }
    EPwm3Regs.CMPA.bit.CMPA = 5000 + angle*2500.0/90.0;
}

void setEPWM3B_RCServo(float angle) {
    if (angle > 90) {
        angle = 90;
    }
    if (angle < -90) {
        angle = -90;
    }
    EPwm3Regs.CMPB.bit.CMPB = 5000 + angle*2500.0/90.0;
}

void setEPWM5A_RCServo(float angle) {
    if (angle > 90) {
        angle = 90;
    }
    if (angle < -90) {
        angle = -90;
    }
    EPwm5Regs.CMPA.bit.CMPA = 5000 + angle*2500.0/90.0;
}

void setEPWM5B_RCServo(float angle) {
    if (angle > 90) {
        angle = 90;
    }
    if (angle < -90) {
        angle = -90;
    }
    EPwm5Regs.CMPB.bit.CMPB = 5000 + angle*2500.0/90.0;
}

void setEPWM6A_RCServo(float angle){ 
    if (angle > 90) {
        angle = 90;
    }
    if (angle < -90) {
        angle = -90;
    }
    EPwm6Regs.CMPA.bit.CMPA = 5000 + angle*2500.0/90.0;
}


void init_EPWM1and2(void) {
    //epwm1A
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 3; // free run mode
    EPwm1Regs.TBCTL.bit.CLKDIV = 0; //divide by 1
    EPwm1Regs.TBCTL.bit.PHSEN = 0; //disable the phase loading
    EPwm1Regs.TBCTR = 0; //Start the timers at zero.
    //Set the period (carrier frequency) of the PWM signal to 20KHz which is a period of
    //50 microseconds. Remember the clock source that the TBCTR register is counting is 50MHz.
    EPwm1Regs.TBPRD = 2500;
    EPwm1Regs.CMPA.bit.CMPA = 1250; //start the duty cycle at 50%.
    EPwm1Regs.AQCTLA.bit.CAU = 1; //set it up such that the signal is cleared when CMPA (or CMPB ) is reached.
    EPwm1Regs.AQCTLA.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm1Regs.AQCTLB.bit.CBU = 1; //set it up such that the signal is cleared when CMPA (or CMPB ) is reached.
    EPwm1Regs.AQCTLB.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm1Regs.TBPHS.bit.TBPHS = 0;

    //EPWM2A
    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //count up mode
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 3; // free run mode
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //divide by 1
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //disable the phase loading
    EPwm2Regs.TBCTR = 0; //Start the timers at zero.
    //Set the period (carrier frequency) of the PWM signal to 20KHz which is a period of
    //50 microseconds. Remember the clock source that the TBCTR register is counting is 50MHz.
    EPwm2Regs.TBPRD = 2500;
    EPwm2Regs.CMPA.bit.CMPA = 1250; //start the duty cycle at 50%.
    EPwm2Regs.AQCTLA.bit.CAU = 1; //set it up such that the signal is cleared when CMPA (or CMPB ) is reached.
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm2Regs.AQCTLB.bit.CBU = 1; //set it up such that the signal is cleared when CMPA (or CMPB ) is reached.
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //Have the pin be set when the TBCTR register is zero
    EPwm2Regs.TBPHS.bit.TBPHS = 0;

    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1; // For EPWM1A
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    EDIS;

    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1);  //EPWM1A
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1);  //EPWM2A

}

void init_ADCsAndDACs(void) {
    EALLOW;
    //write configurations for all ADCs  ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdccRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcdRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    AdcSetMode(ADC_ADCD, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcdRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcdRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);

    //Select the channels to convert and end of conversion flag
    //Many statements commented out,  To be used when using ADCA or ADCB
    //ADCA
    //AdcaRegs.ADCSOC0CTL.bit.CHSEL = ???;  //SOC0 will convert Channel you choose Does not have to be A0
    //AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = ???;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    //AdcaRegs.ADCSOC1CTL.bit.CHSEL = ???;  //SOC1 will convert Channel you choose Does not have to be A1
    //AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = ???;// EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    //AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = ???; //set to last SOC that is converted and it will set INT1 flag ADCA1
    //AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    //AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCB  Microphone is connected to ADCINB4
    //    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert Channel you choose Does not have to be B0
    //    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 17; // EPWM7 ADCSOCA or another trigger you choose will trigger SOC0
    //    AdcbRegs.ADCSOC1CTL.bit.CHSEL = ???;  //SOC1 will convert Channel you choose Does not have to be B1
    //    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = ???; // EPWM7 ADCSOCA or another trigger you choose will trigger SOC1
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //set to last SOC that is converted and it will set INT1 flag ADCB1
    //#warn  do I need INT1E on for DMA  try both ways
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    //    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    //    AdcbRegs.ADCINTSEL1N2.bit.INT1CONT = 1;     // Interrupt pulses regardless of flag state

    //ADCC
    AdccRegs.ADCSOC0CTL.bit.CHSEL = 2;  //SOC0 will convert Channel you choose Does not have to be B0
    AdccRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC0
    AdccRegs.ADCSOC1CTL.bit.CHSEL = 3;  //SOC1 will convert Channel you choose Does not have to be B1
    AdccRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC1CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC1
    AdccRegs.ADCSOC2CTL.bit.CHSEL = 4;  //SOC2 will convert Channel you choose Does not have to be B2
    AdccRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC2CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC2
    AdccRegs.ADCSOC3CTL.bit.CHSEL = 5;  //SOC3 will convert Channel you choose Does not have to be B3
    AdccRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdccRegs.ADCSOC3CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA or another trigger you choose will trigger SOC3
    AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 3; //set to last SOC that is converted and it will set INT1 flag ADCB1
    AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    //ADCD
    //    AdcdRegs.ADCSOC0CTL.bit.CHSEL = 0;  // set SOC0 to convert pin D0
    //    AdcdRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC0CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA will trigger SOC0
    //    AdcdRegs.ADCSOC1CTL.bit.CHSEL = 1;  //set SOC1 to convert pin D1
    //    AdcdRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //    AdcdRegs.ADCSOC1CTL.bit.TRIGSEL = 11; // EPWM4 ADCSOCA will trigger SOC1
    //AdcdRegs.ADCSOC2CTL.bit.CHSEL = ???;  //set SOC2 to convert pin D2
    //AdcdRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM4 ADCSOCA will trigger SOC2
    //AdcdRegs.ADCSOC3CTL.bit.CHSEL = ???;  //set SOC3 to convert pin D3
    //AdcdRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcdRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM4 ADCSOCA will trigger SOC3
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1 flag ADCD1
    //    AdcdRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
    //    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;

    // Enable DACA and DACB outputs
    EALLOW;
    DacaRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacA output-->uses ADCINA0
    DacaRegs.DACCTL.bit.LOADMODE = 0;   //load on next sysclk
    DacaRegs.DACCTL.bit.DACREFSEL = 1;  //use ADC VREF as reference voltage

    DacbRegs.DACOUTEN.bit.DACOUTEN = 1; //enable dacB output-->uses ADCINA1
    DacbRegs.DACCTL.bit.LOADMODE = 0;   //load on next sysclk
    DacbRegs.DACCTL.bit.DACREFSEL = 1;  //use ADC VREF as reference voltage
    EDIS;

}

void InitSE423DefaultGPIO(void) {
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

}

void PostSWI1(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI1
}
void PostSWI2(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx10 = 1; // Manually cause the interrupt for the SWI2
}
void PostSWI3(void) {
    PieCtrlRegs.PIEIFR12.bit.INTx11 = 1; // Manually cause the interrupt for the SWI3
}

float PI_LeftWheel_1 = 0;
float PI_RightWheel_1 = 0;
float PI_LeftVel = 0;
float PI_RightVel = 0;
float PI_Cpos = 2.8*.6;
float PI_Cneg = -2.6*.6;
float PI_Vpos = 2.17*.6;
float PI_Vneg = 2.15*.6;

float PI_e1       = 0.0F;
float PI_e2       = 0.0F;
float PI_e1s      = 0.0F;
float PI_e2s      = 0.0F;
float PI_esteer   = 0.0F;
// Tunable gains
// Closed-loop coupled velocity control
float PI_Kp       = 3.0F;
float PI_Ki       = 15.0F;
float PI_Kp_turn  = 3.0F;

void PIcontrol(float *uLeftlocal,float *uRightlocal,float vreflocal, float turnlocal, float LeftWheellocal,float RightWheellocal){

    PI_LeftVel = (1.235/12.0)*(LeftWheellocal - PI_LeftWheel_1)*1000;
    PI_RightVel = (1.235/12.0)*(RightWheellocal - PI_RightWheel_1)*1000;

    PI_esteer   = PI_RightVel - PI_LeftVel + turnlocal;
    PI_e1  =  vreflocal - PI_LeftVel + PI_Kp_turn*PI_esteer;
    PI_e2  =  vreflocal - PI_RightVel - PI_Kp_turn*PI_esteer;
    PI_e1s =  PI_e1s + PI_e1;
    PI_e2s =  PI_e2s + PI_e2;
    *uLeftlocal  = PI_Kp*PI_e1 + PI_Ki*0.001*PI_e1s;
    *uRightlocal  = PI_Kp*PI_e2 + PI_Ki*0.001*PI_e2s;

    if (PI_LeftVel > 0) {
        *uLeftlocal = *uLeftlocal + PI_Vpos*PI_LeftVel + PI_Cpos;
    } else {
        *uLeftlocal = *uLeftlocal + PI_Vneg*PI_LeftVel + PI_Cneg;
    }

    if (PI_RightVel > 0) {
        *uRightlocal = *uRightlocal + PI_Vpos*PI_RightVel + PI_Cpos;
    } else {
        *uRightlocal = *uRightlocal + PI_Vneg*PI_RightVel + PI_Cneg;
    }

    // Check for integral windup
    if (fabs(*uLeftlocal)>10.0) PI_e1s = PI_e1s * 0.99;
    if (fabs(*uRightlocal)>10.0) PI_e2s = PI_e2s * 0.99;

    // Final check to make sure within range
    if (*uLeftlocal> 10)  *uLeftlocal = 10.0;
    if (*uLeftlocal<-10)  *uLeftlocal = -10.0;
    if (*uRightlocal> 10)  *uRightlocal = 10.0;
    if (*uRightlocal<-10)  *uRightlocal = -10.0;

    PI_LeftWheel_1 = LeftWheellocal;
    PI_RightWheel_1 = RightWheellocal;

}
