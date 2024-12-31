#ifndef _CLA_SHARED_H_
#define _CLA_SHARED_H_
//#############################################################################
//
// FILE:   cla_matrix_mpy_shared.h
//
// TITLE:  Matrix Multiplication Test
//
//#############################################################################
// $TI Release: F2837xS Support Library v210 $
// $Release Date: Tue Nov  1 15:35:23 CDT 2016 $
// $Copyright: Copyright (C) 2014-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//#############################################################################

//
// Included Files
//
#include "F2837xD_Cla_defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//
// Globals
//

//
//Task 1 (C) Variables
//
// vars in CLA msg RAM
extern float CLAin_Value;
extern float CLAreturn_Value;
extern float CLA_u;
extern float CLA_enc1;
extern float CLA_enc2;

// vars in CLA Data RAM
extern float setpt;
extern float x1old;
extern float x2old;
extern float x1dot_old;
extern float x2dot_old;
extern int16_t catchit;
extern int16_t swingup;


extern int32_t numtimes1;
extern int32_t numtimes2;
extern int32_t numtimes3;
extern int32_t numtimes4;
extern int32_t numtimes5;
extern int32_t numtimes6;
extern int32_t numtimes7;
extern int32_t numtimes8;



//
//Task 2 (C) Variables
//

//
//Task 3 (C) Variables
//

//
//Task 4 (C) Variables
//

//
//Task 5 (C) Variables
//

//
//Task 6 (C) Variables
//

//
//Task 7 (C) Variables
//

//
//Task 8 (C) Variables
//

//
//Common (C) Variables
//

//
// Function Prototypes
//
// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.
//
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"

#endif //end of _CLA_SHARED_H_ definition

//
// End of file
//
