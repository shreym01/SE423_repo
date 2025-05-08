/* SERIAL.C: This code is designed to act as a low-level serial driver for
    higher-level programming.  Ideally, one could simply call init_serial()
    to initialize the serial port, then use serial_send("data", 4) to send
    an array of data (8-bit unsigned character strings).

    WRITTEN BY : Paul Miller <pamiller@uiuc.edu>
    $Id: serial.c,v 1.4 2003/08/08 16:08:56 paul Exp $
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <buffer.h>
#include <F28379dSerial.h>
#include <F2837xD_sci.h>
#include "SE423Lib.h"

serialSCIA_t SerialA;
serialSCIB_t SerialB; 
serialSCIC_t SerialC;
serialSCID_t SerialD;

char RXAdata = 0;
char RXBdata = 0;
char RXCdata = 0;
char RXDdata = 0;
uint32_t numRXA = 0;
uint32_t numRXB = 0;
uint32_t numRXC = 0;
uint32_t numRXD = 0;

char buf[477]; //store all the data
uint16_t dis_index = 0;
uint16_t G_count = 0; //number of thing received after 'G'
uint16_t LF_count = 0; //number of consecutive line feed
uint16_t buf_index = 0; //for debug
uint16_t get_dis_first;//if it is the first part of the two parts data
uint16_t print_dis_index=0;
uint32_t numRCXerrors = 0;
int16_t tempscid = 0;
int16_t distance = 0;
float checkdist = 0;

uint16_t COMA_state = 0;
uint16_t COMALSB = 0;
uint16_t received_CAM_countThreshold1 = 0;
uint16_t NewCAMDataThreshold1 = 0;  // Flag new data
CAMRecFloats_t DataFromCameraThreshold1;
float fromCAMvaluesThreshold1[CAMNUM_FROM_FLOATS];
uint16_t received_CAM_countThreshold2 = 0;
uint16_t NewCAMDataThreshold2 = 0;  // Flag new data
CAMRecFloats_t DataFromCameraThreshold2;
float fromCAMvaluesThreshold2[CAMNUM_FROM_FLOATS];

uint16_t received_CAM_countAprilTag1 = 0;
uint16_t NewCAMDataAprilTag1 = 0;  // Flag new data
CAMRecFloats_t DataFromCameraAprilTag1;
float fromCAMvaluesAprilTag1[CAMNUM_FROM_FLOATS];

uint16_t tempLSB = 0;
LVRecFloats_t DataFromLabView;
LVSendFloats_t DataToLabView;
uint16_t received_LV_count = 0;
uint16_t NewLVData = 0;
char LVsenddata[LVNUM_TOFROM_FLOATS*4+2];
float fromLVvalues[LVNUM_TOFROM_FLOATS];

CMDRecFloats_t DataFromLinuxCMD;
uint16_t received_CMD_count = 0;
uint16_t newLinuxCommands = 0;
float LinuxCommands[CMDNUM_FROM_FLOATS];
uint16_t com_state = 0;//the state of receive from pi (SCID)
uint16_t LADARpingpong = 0;
uint16_t read_ladar_state = 0; //states for ladar reading, initialized as wait for S command
datapts ladar_data[228]; //distance data from ladar

uint16_t new_optitrack = 0;
float Optitrackdata[OPTITRACKDATASIZE];  // x, y, heading, trackableID, framecount
uint16_t received_opti_count = 0;//the count for receiving optitrack data
optiData_t optirx;

char path_received[81];
int16_t received_path_count = 0;
int16_t newAstarPath = 0;
float ladar161distance = 0;
float ladar199distance = 0;


// for SerialA
uint16_t init_serialSCIA(serialSCIA_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialA) {
        sci = &SciaRegs;
        s->sci = sci;

        init_bufferSCIA(&s->TX);

        GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_PUSHPULL);

    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 1;
        IER |= (M_INT9);
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP9);
        GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
        GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
        GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_PUSHPULL);

    }

    return 0;
}

void uninit_serialSCIA(serialSCIA_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialA) {
        PieCtrlRegs.PIEIER9.bit.INTx1 = 0;
        PieCtrlRegs.PIEIER9.bit.INTx2 = 0;
        IER &= ~M_INT9;
    }
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIA(serialSCIA_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIA) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIA_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}

// For SerialB
uint16_t init_serialSCIB(serialSCIB_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialB) {
        sci = &ScibRegs;
        s->sci = sci;

        init_bufferSCIB(&s->TX);

        GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PUSHPULL);


    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialB) {
        PieCtrlRegs.PIEIER9.bit.INTx3 = 1;
        PieCtrlRegs.PIEIER9.bit.INTx4 = 1;
        IER |= (M_INT9);
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP9);
        GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 2);
        GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 2);
        GPIO_SetupPinOptions(14, GPIO_OUTPUT, GPIO_PUSHPULL);

    }

    return 0;
}

void uninit_serialSCIB(serialSCIB_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialB) {
        PieCtrlRegs.PIEIER9.bit.INTx3 = 0;
        PieCtrlRegs.PIEIER9.bit.INTx4 = 0;
        IER &= ~M_INT9;
    }

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIB(serialSCIB_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIB) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIB_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}

// for SerialC
uint16_t init_serialSCIC(serialSCIC_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialC) {
        sci = &ScicRegs;
        s->sci = sci;

        init_bufferSCIC(&s->TX);
        GPIO_SetupPinMux(139, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(139, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_PUSHPULL);

    } else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialC) {
        PieCtrlRegs.PIEIER8.bit.INTx5 = 1;
        PieCtrlRegs.PIEIER8.bit.INTx6 = 1;
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP8);
        IER |= (M_INT8);
        GPIO_SetupPinMux(139, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(139, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_PUSHPULL);
    } 

    return 0;
}

void uninit_serialSCIC(serialSCIC_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialC) {
        PieCtrlRegs.PIEIER8.bit.INTx5 = 0;
        PieCtrlRegs.PIEIER8.bit.INTx6 = 0;
        IER &= ~M_INT8;
    } 

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCIC(serialSCIC_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCIC) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCIC_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}

// For Serial D
uint16_t init_serialSCID(serialSCID_t *s, uint32_t baud)
{
    volatile struct SCI_REGS *sci;
    uint32_t clk;

    if (s == &SerialD) {
        sci = &ScidRegs;
        s->sci = sci;

        init_bufferSCID(&s->TX);
        GPIO_SetupPinMux(105, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(105, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(104, GPIO_MUX_CPU1, 0);
        GPIO_SetupPinOptions(104, GPIO_OUTPUT, GPIO_PUSHPULL);
    }
    else {
        return 1;
    }

    /* init for standard baud,8N1 comm */
    sci->SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    sci->SCICCR.all = 0x0;
    sci->SCICTL1.all = 0x0;
    sci->SCICTL2.all = 0x0;
    sci->SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    sci->SCILBAUD.all = clk & 0xFF;
    sci->SCIHBAUD.all = (clk >> 8) & 0xFF;

    sci->SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    sci->SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    sci->SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    sci->SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    sci->SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    sci->SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    sci->SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    sci->SCIFFTX.bit.TXFIFORESET = 0;
    sci->SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    sci->SCIFFTX.bit.TXFIFORESET = 1;

    sci->SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    sci->SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    sci->SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    sci->SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    sci->SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.TXWAKE = 0;
    sci->SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    sci->SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    sci->SCICTL1.bit.SWRESET = 1;       // re-enable SCI

    /* enable PIE interrupts */
    if (s == &SerialD) {
        PieCtrlRegs.PIEIER8.bit.INTx7 = 1;
        PieCtrlRegs.PIEIER8.bit.INTx8 = 1;
        PieCtrlRegs.PIEACK.all = (PIEACK_GROUP8);
        IER |= (M_INT8);
        GPIO_SetupPinMux(105, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(105, GPIO_INPUT, GPIO_PULLUP);
        GPIO_SetupPinMux(104, GPIO_MUX_CPU1, 6);
        GPIO_SetupPinOptions(104, GPIO_OUTPUT, GPIO_PUSHPULL);

    }

    return 0;
}

void uninit_serialSCID(serialSCID_t *s)
{
    volatile struct SCI_REGS *sci = s->sci;

    /* disable PIE interrupts */
    if (s == &SerialD) {
        PieCtrlRegs.PIEIER8.bit.INTx7 = 0;
        PieCtrlRegs.PIEIER8.bit.INTx8 = 0;
        IER &= ~M_INT8;
    }

    sci->SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    sci->SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    sci->SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    sci->SCICTL1.bit.RXENA = 0;         // disable SCI receiver
    sci->SCICTL1.bit.TXENA = 0;         // disable SCI transmitter
}



/***************************************************************************
 * SERIAL_SEND()
 *
 * "User level" function to send data via serial.  Return value is the
 * length of data successfully copied to the TX buffer.
 ***************************************************************************/
uint16_t serial_sendSCID(serialSCID_t *s, char *data, uint16_t len)
{
    uint16_t i = 0;
    if (len && s->TX.size < BUF_SIZESCID) {
        for (i = 0; i < len; i++) {
            if (buf_writeSCID_1(&s->TX, data[i] & 0x00FF) != 0) break;
        }
        s->sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
        s->sci->SCIFFTX.bit.TXFFIENA = 1;       // TX: enable fifo interrupt
    }
    return i;
}



/***************************************************************************
 * TXxINT_DATA_SENT()
 *
 * Executed when transmission is ready for additional data.  These functions
 * read the next char of data and put it in the TXBUF register for transfer.
 ***************************************************************************/
#ifdef _FLASH
#pragma CODE_SECTION(TXAINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXAINT_data_sent(void)
{
    char data;
    if (buf_readSCIA_1(&SerialA.TX,0,&data) == 0) {
        while ( (buf_readSCIA_1(&SerialA.TX,0,&data) == 0)
                && (SerialA.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIA(&SerialA.TX, 1);
            SerialA.sci->SCITXBUF.all = data;
        }
    } else {
        SerialA.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialA.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//for serialB
#ifdef _FLASH
#pragma CODE_SECTION(TXBINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXBINT_data_sent(void)
{
    char data;
    if (buf_readSCIB_1(&SerialB.TX,0,&data) == 0) {
        while ( (buf_readSCIB_1(&SerialB.TX,0,&data) == 0)
                && (SerialB.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIB(&SerialB.TX, 1);
            SerialB.sci->SCITXBUF.all = data;
        }
    } else {
        SerialB.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialB.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


//for serialC
#ifdef _FLASH
#pragma CODE_SECTION(TXCINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXCINT_data_sent(void)
{
    char data;
    if (buf_readSCIC_1(&SerialC.TX,0,&data) == 0) {
        while ( (buf_readSCIC_1(&SerialC.TX,0,&data) == 0)
                && (SerialC.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCIC(&SerialC.TX, 1);
            SerialC.sci->SCITXBUF.all = data;
        }
    } else {
        SerialC.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialC.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//for serialD
#ifdef _FLASH
#pragma CODE_SECTION(TXDINT_data_sent, ".TI.ramfunc");
#endif
__interrupt void TXDINT_data_sent(void)
{
    char data;
    if (buf_readSCID_1(&SerialD.TX,0,&data) == 0) {
        while ( (buf_readSCID_1(&SerialD.TX,0,&data) == 0)
                && (SerialD.sci->SCIFFTX.bit.TXFFST != 0x10) ) {
            buf_removeSCID(&SerialD.TX, 1);
            SerialD.sci->SCITXBUF.all = data;
        }
    } else {
        SerialD.sci->SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    }
    SerialD.sci->SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}

//for SerialA
#ifdef _FLASH
#pragma CODE_SECTION(RXAINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXAINT_recv_ready(void)
{
    RXAdata = SciaRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXAdata & 0xC000) {
        SciaRegs.SCICTL1.bit.SWRESET = 0;
        SciaRegs.SCICTL1.bit.SWRESET = 1;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
        SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXAdata = RXAdata & 0x00FF;
        if (COMA_state == 0) {

            if (RXAdata == '*') {
                COMA_state = 1; //for receiving path
            } else {
                COMA_state = 0;
            }
        } else if (COMA_state == 1) {

            if (RXAdata == '*') {
                COMA_state = 10; // into the read mode
                received_CAM_countThreshold1 = 0;
            } else if (RXAdata == '!') {
                COMA_state = 20; // into the read mode
                received_CAM_countThreshold2 = 0;
            } else if (RXAdata == '$') {
                COMA_state = 30; // into the read mode
                received_CAM_countAprilTag1 = 0;
            }
            else {
                COMA_state = 0;
            }
        } else if (COMA_state == 10) {

            if ((received_CAM_countThreshold1 % 2) == 0) {
                COMALSB = RXAdata;
            } else {
                DataFromCameraThreshold1.rawData[received_CAM_countThreshold1/2] = (RXAdata << 8) | COMALSB;
            }
            received_CAM_countThreshold1++;
            if (received_CAM_countThreshold1 >= 4*CAMNUM_FROM_FLOATS) {
                received_CAM_countThreshold1 = 0;
                COMA_state = 0;
                fromCAMvaluesThreshold1[0] = DataFromCameraThreshold1.floatData[0];
                fromCAMvaluesThreshold1[1] = DataFromCameraThreshold1.floatData[1];
                fromCAMvaluesThreshold1[2] = DataFromCameraThreshold1.floatData[2];
                fromCAMvaluesThreshold1[3] = DataFromCameraThreshold1.floatData[3];
                fromCAMvaluesThreshold1[4] = DataFromCameraThreshold1.floatData[4];
                fromCAMvaluesThreshold1[5] = DataFromCameraThreshold1.floatData[5];
                fromCAMvaluesThreshold1[6] = DataFromCameraThreshold1.floatData[6];
                fromCAMvaluesThreshold1[7] = DataFromCameraThreshold1.floatData[7];
                fromCAMvaluesThreshold1[8] = DataFromCameraThreshold1.floatData[8];
                NewCAMDataThreshold1 = 1;  // Flag new data
            }
        } else if (COMA_state == 20) {
            if ((received_CAM_countThreshold2 % 2) == 0) {
                COMALSB = RXAdata;
            } else {
                DataFromCameraThreshold2.rawData[received_CAM_countThreshold2/2] = (RXAdata << 8) | COMALSB;
            }
            received_CAM_countThreshold2++;
            if (received_CAM_countThreshold2 >= 4*CAMNUM_FROM_FLOATS) {
                received_CAM_countThreshold2 = 0;
                COMA_state = 0;
                fromCAMvaluesThreshold2[0] = DataFromCameraThreshold2.floatData[0];
                fromCAMvaluesThreshold2[1] = DataFromCameraThreshold2.floatData[1];
                fromCAMvaluesThreshold2[2] = DataFromCameraThreshold2.floatData[2];
                fromCAMvaluesThreshold2[3] = DataFromCameraThreshold2.floatData[3];
                fromCAMvaluesThreshold2[4] = DataFromCameraThreshold2.floatData[4];
                fromCAMvaluesThreshold2[5] = DataFromCameraThreshold2.floatData[5];
                fromCAMvaluesThreshold2[6] = DataFromCameraThreshold2.floatData[6];
                fromCAMvaluesThreshold2[7] = DataFromCameraThreshold2.floatData[7];
                fromCAMvaluesThreshold2[8] = DataFromCameraThreshold2.floatData[8];
                NewCAMDataThreshold2 = 1;  // Flag new data
            }
        } else if (COMA_state == 30) {
            if ((received_CAM_countAprilTag1 % 2) == 0) {
                COMALSB = RXAdata;
            } else {
                DataFromCameraAprilTag1.rawData[received_CAM_countAprilTag1/2] = (RXAdata << 8) | COMALSB;
            }
            received_CAM_countAprilTag1++;
            if (received_CAM_countAprilTag1 >= 4*CAMNUM_FROM_FLOATS) {
                received_CAM_countAprilTag1 = 0;
                COMA_state = 0;
                fromCAMvaluesAprilTag1[0] = DataFromCameraAprilTag1.floatData[0];
                fromCAMvaluesAprilTag1[1] = DataFromCameraAprilTag1.floatData[1];
                fromCAMvaluesAprilTag1[2] = DataFromCameraAprilTag1.floatData[2];
                fromCAMvaluesAprilTag1[3] = DataFromCameraAprilTag1.floatData[3];
                fromCAMvaluesAprilTag1[4] = DataFromCameraAprilTag1.floatData[4];
                fromCAMvaluesAprilTag1[5] = DataFromCameraAprilTag1.floatData[5];
                fromCAMvaluesAprilTag1[6] = DataFromCameraAprilTag1.floatData[6];
                fromCAMvaluesAprilTag1[7] = DataFromCameraAprilTag1.floatData[7];
                fromCAMvaluesAprilTag1[8] = DataFromCameraAprilTag1.floatData[8];
                NewCAMDataAprilTag1 = 1;  // Flag new data
            }
        }

        numRXA ++;
    }

    SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;

}

//for SerialB
#ifdef _FLASH
#pragma CODE_SECTION(RXBINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXBINT_recv_ready(void)
{
    RXBdata = ScibRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXBdata & 0xC000) {
        ScibRegs.SCICTL1.bit.SWRESET = 0;
        ScibRegs.SCICTL1.bit.SWRESET = 1;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScibRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXBdata = RXBdata & 0x00FF;
        // Do something with recieved character
        numRXB ++;
    }

    ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
}


// for SerialC
#ifdef _FLASH
#pragma CODE_SECTION(RXCINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXCINT_recv_ready(void)
{
    RXCdata = ScicRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXCdata & 0xC000) {
        ScicRegs.SCICTL1.bit.SWRESET = 0;
        ScicRegs.SCICTL1.bit.SWRESET = 1;
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXCdata = RXCdata & 0x00FF;
        if (read_ladar_state == 0) {//wait for the starting char 'G'
            buf_index = 0;
            if (RXCdata =='G') {
                read_ladar_state = 1;
                G_count = 1;
            }
        } else if (read_ladar_state == 1) {//get the headers
            G_count += 1;//count the number of headers
            //check the status code and if it is not '0', wait for the next packet
            if (G_count == 11) {
                if (RXCdata != '0') {
                    read_ladar_state = 0;
                }
            }
            if (G_count >= 12) {//there are total 12 things before the data
                read_ladar_state = 2;
                get_dis_first = 1;
                dis_index = 0;
            }
        } else if (read_ladar_state == 2) {//get actual data
            if (RXCdata == 0x0a) {//check for line feed
                LF_count += 1;
            } else {
                LF_count = 0;
                if (get_dis_first == 1) {
                    tempscid = (RXCdata - 0x30) << 6;//first data is the top 6 bit of the distance reading
                    get_dis_first = 0;
                } else if (get_dis_first == 0) {
                    distance = tempscid + (RXCdata - 0x30);//second data is the bottom 6 bit of the distance reading
                    if (distance == 0 || distance == 6) {//error code
                        distance = LADAR_MAX_READING; //set to the max reading
                    } else if (distance == 16) { //distance is 4096mm
                        distance = 4096;
                    }
                    if (LADARpingpong == 0) {
                        //ladar_data[dis_index%228].distance_ping = distance*0.00328084; // fill in the distance_ping array
                        checkdist = distance*0.00328084;
                        if ((checkdist > 0.25) && (checkdist < 20.0)) {
                            ladar_data[dis_index].distance_ping = checkdist; // fill in the distance_ping array
                            if (dis_index == 161) {
                                ladar161distance = checkdist;
                            }
                            if (dis_index == 199) {
                                ladar199distance = checkdist;
                            }

                        } else {
                            numRCXerrors++;
                            ladar_data[dis_index].distance_ping = ladar_data[dis_index].distance_pong;
                            if (dis_index == 161) {
                                ladar161distance = ladar_data[dis_index].distance_pong;
                            }
                            if (dis_index == 199) {
                                ladar199distance = ladar_data[dis_index].distance_pong;
                            }
                        }
                    } else if (LADARpingpong == 1) {
                        //ladar_data[dis_index%228].distance_pong = distance*0.00328084; // fill in the distance_pong array
                        checkdist = distance*0.00328084;
                        if ((checkdist >= 0.25) && (checkdist < 20.0)) {
                            ladar_data[dis_index].distance_pong = checkdist; // fill in the distance_ping array
                            if (dis_index == 161) {
                                ladar161distance = checkdist;
                            }
                            if (dis_index == 199) {
                                ladar199distance = checkdist;
                            }
                        } else {
                            numRCXerrors++;
                            ladar_data[dis_index].distance_pong = ladar_data[dis_index].distance_ping;
                            if (dis_index == 161) {
                                ladar161distance = ladar_data[dis_index].distance_ping;
                            }
                            if (dis_index == 199) {
                                ladar199distance = ladar_data[dis_index].distance_ping;
                            }
                        }
                    }
                    dis_index += 1;
                    get_dis_first = 1;
                }
            }
            if (LF_count == 2) { //two consecutive line feed means the end of the current data stream
                read_ladar_state = 0;
                print_dis_index = dis_index;
                PostSWI2(); // Manually cause the interrupt for the SWI2
                LADARpingpong = 1-LADARpingpong;
            }
        }
        buf[buf_index % 477] = RXCdata;//fill in the buf array, debug purpose
        buf_index++;
        numRXC ++;

    }

    ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}


//for SerialD
#ifdef _FLASH
#pragma CODE_SECTION(RXDINT_recv_ready, ".TI.ramfunc");
#endif
__interrupt void RXDINT_recv_ready(void)
{
    uint16_t i = 0;//for loop
    RXDdata = ScidRegs.SCIRXBUF.all;

    /* SCI PE or FE error */
    if (RXDdata & 0xC000) {
        ScidRegs.SCICTL1.bit.SWRESET = 0;
        ScidRegs.SCICTL1.bit.SWRESET = 1;
        ScidRegs.SCIFFRX.bit.RXFIFORESET = 0;
        ScidRegs.SCIFFRX.bit.RXFIFORESET = 1;
    } else {
        RXDdata = RXDdata & 0x00FF;
        if (com_state == 0) {
            if (RXDdata == '*') {
                com_state = 1; //for receiving path
            } else if (RXDdata == 'O') {
                com_state = 2; //for receiving optitrack data
            } else {
                com_state = 0;
            }
        } else if (com_state == 1) {
            if (RXDdata == '*') {
                com_state = 10; // into the read mode
                received_path_count = 0;
                newAstarPath = 0;
            } else if (RXDdata == '$') {
                com_state = 30;
                received_LV_count = 0;
            } else if (RXDdata == '!') {
                com_state = 40;
                received_CMD_count = 0;
            } else {
                com_state = 0;
            }
        } else if (com_state == 10) {  // this should not occurred left it here for future code with A* path planning
            path_received[received_path_count] = RXDdata;
            received_path_count++;
            if (received_path_count >= 81) {
                received_path_count = 0;
                com_state = 0;
                newAstarPath = 1;
            }
        } else if (com_state == 2) {
            if (RXDdata == 'p'){
                com_state = 20;
            } else {
                com_state = 0;
            }
        } else if (com_state == 20) {
            if (RXDdata == 't'){
                com_state = 21;
            } else {
                com_state = 0;
            }
        } else if (com_state == 21) {
            if (RXDdata == 'i'){
                com_state = 22;
            } else {
                com_state = 0;
            }
        } else if (com_state == 22) {
            if (RXDdata == ':'){
                com_state = 23;
                //                received_opti_count = 0;
            } else {
                com_state = 0;
            }
        } else if (com_state == 23) {
            if (received_opti_count % 2 == 0){  // this is for motion tracking system  left for future code
                tempLSB = RXDdata;
            } else {
                optirx.rawData[received_opti_count/2] = (RXDdata << 8)|tempLSB;
            }
            received_opti_count++;
            if (received_opti_count >= OPTITRACKDATASIZE*4) {
                if (new_optitrack == 0) {
                    for (i=0;i<OPTITRACKDATASIZE;i++) {  // x, y, heading, trackableID, framecount
                        Optitrackdata[i] = optirx.floatData[i];
                    }
                    new_optitrack = 1;
                }
                received_opti_count = 0;
                com_state = 0;
            }
        } else if (com_state == 30) {
            if ((received_LV_count % 2) == 0) {
                tempLSB = RXDdata;
            } else {
                DataFromLabView.rawData[received_LV_count/2] = (RXDdata << 8)|tempLSB;
            }
            received_LV_count++;
            if (received_LV_count >= 4*LVNUM_TOFROM_FLOATS) {
                received_LV_count = 0;
                com_state = 0;
                fromLVvalues[0] = DataFromLabView.floatData[0];
                fromLVvalues[1] = DataFromLabView.floatData[1];
                fromLVvalues[2] = DataFromLabView.floatData[2];
                fromLVvalues[3] = DataFromLabView.floatData[3];
                fromLVvalues[4] = DataFromLabView.floatData[4];
                fromLVvalues[5] = DataFromLabView.floatData[5];
                fromLVvalues[6] = DataFromLabView.floatData[6];
                fromLVvalues[7] = DataFromLabView.floatData[7];
                NewLVData = 1;  // Flag new data
            }
        } else if (com_state == 40) {
            if ((received_CMD_count % 2) == 0) {
                tempLSB = RXDdata;
            } else {
                DataFromLinuxCMD.rawData[received_CMD_count/2] = (RXDdata << 8)|tempLSB;
            }
            received_CMD_count++;
            if (received_CMD_count >= 4*CMDNUM_FROM_FLOATS) {
                received_CMD_count = 0;
                com_state = 0;

                LinuxCommands[0] = DataFromLinuxCMD.floatData[0];
                LinuxCommands[1] = DataFromLinuxCMD.floatData[1];
                LinuxCommands[2] = DataFromLinuxCMD.floatData[2];
                LinuxCommands[3] = DataFromLinuxCMD.floatData[3];
                LinuxCommands[4] = DataFromLinuxCMD.floatData[4];
                LinuxCommands[5] = DataFromLinuxCMD.floatData[5];
                LinuxCommands[6] = DataFromLinuxCMD.floatData[6];
                LinuxCommands[7] = DataFromLinuxCMD.floatData[7];
                LinuxCommands[8] = DataFromLinuxCMD.floatData[8];
                LinuxCommands[9] = DataFromLinuxCMD.floatData[9];
                LinuxCommands[10] = DataFromLinuxCMD.floatData[10];
                newLinuxCommands = 1;  // Flag new data

            }
        }
    }
    numRXD ++;
    ScidRegs.SCIFFRX.bit.RXFFINTCLR = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP8;
}


// SerialA only setup for Tera Term connection
char serial_printf_bufSCIA[BUF_SIZESCIA];

uint16_t serial_printf(serialSCIA_t *s, char *fmt, ...)
{
    va_list ap;

    va_start(ap,fmt);
    vsprintf(serial_printf_bufSCIA,fmt,ap);
    va_end(ap);

    return serial_sendSCIA(s,serial_printf_bufSCIA,strlen(serial_printf_bufSCIA));
}

//For Text LCD  SCIB only
char UART_printf_bufferSCIB[BUF_SIZESCIB];
void UART_vprintfLine(unsigned char line, char *format, va_list ap)
{
    char sendmsg[24];

    int i;

    vsprintf(UART_printf_bufferSCIB,format,ap);

    // Add header information and pad end of transfer with spaces to clear display
    sendmsg[0] = 0xFE;
    sendmsg[1] = 'G';
    sendmsg[2] = 1;
    sendmsg[3] = line;
    for (i=4;i<24;i++) {
        if (i >= strlen(UART_printf_bufferSCIB)+4) {
            sendmsg[i] = ' ';
        } else {
            sendmsg[i] = UART_printf_bufferSCIB[i-4];
        }
    }
    serial_sendSCIB(&SerialB,sendmsg,24);
}

void UART_printfLine(unsigned char line, char *format, ...)
{
    va_list ap;
    va_start(ap, format);
    UART_vprintfLine(line,format,ap);
}


