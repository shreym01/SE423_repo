#ifndef _SE423LIB_H_
#define _SE423LIB_H_

void setDACA(float dacouta0);
void setDACB(float dacouta1);
void setEPWM1A(float controleffort);
void setEPWM2A(float controleffort);
void init_eQEPs(void);
float readEncLeft(void);
float readEncRight(void);
float readEncWheel(void);
void setupSpib(void);
uint16_t readbuttons(void);
void init_RCServoPWM_3AB_5AB_6A(void);
void setEPWM3A_RCServo(float angle);
void setEPWM3B_RCServo(float angle);
void setEPWM5A_RCServo(float angle);
void setEPWM5B_RCServo(float angle);
void setEPWM6A_RCServo(float angle);
void init_EPWM1and2(void);
void init_ADCsAndDACs(void);
void InitSE423DefaultGPIO(void);
void PostSWI1(void);
void PostSWI2(void);
void PostSWI3(void);
void PIcontrol(float *uLeftlocal,float *uRightlocal,float vreflocal, float turnlocal, float LeftWheellocal,float RightWheellocal);

#endif
