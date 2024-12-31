#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "F28x_Project.h"     // Device Headerfile and Examples Include File
#include <F28379dSerial.h>
#include "OptiTrack.h"
#define PI          3.1415926535897932384626433832795

float Optiprevious_frame = -1;
extern float Optitrackdata[OPTITRACKDATASIZE];
int32_t Optiframe_error = 0;
float Optitemp_theta = 0.0;
float OptitempOPTITRACK_theta = 0.0;
uint32_t OptitrackableIDerror = 0;
uint16_t Optifirstdata = 1;
int32_t OptitrackableID = -1;

pose UpdateOptitrackStates(pose localROBOTps, int16_t * flag) {

    pose localOPTITRACKps;

    // Check for frame errors / packet loss
    if (Optiprevious_frame == Optitrackdata[OPTITRACKDATASIZE-1]) {
        Optiframe_error++;
    }
    Optiprevious_frame = Optitrackdata[OPTITRACKDATASIZE-1];

    // Set local trackableID if first receive data
    if (Optifirstdata){
        //trackableID = (int)Optitrackdata[OPTITRACKDATASIZE-1]; // removed to add new trackableID in shared memory
        OptitrackableID = Optitrackdata[OPTITRACKDATASIZE-2];
        Optifirstdata = 0;
    }

    // Check if local trackableID has changed - should never happen
    if (OptitrackableID != Optitrackdata[OPTITRACKDATASIZE-2]) {
        OptitrackableIDerror++;
        // do some sort of reset(?)
    }

    // Save position and yaw data
    if (isnan(Optitrackdata[0]) != 1) {  // this checks if the position data being received contains NaNs
        // check if x,y,yaw all equal 0.0 (almost certainly means the robot is untracked)
        if ((Optitrackdata[0] != 0.0) && (Optitrackdata[1] != 0.0) && (Optitrackdata[2] != 0.0)) {
            // save x,y
            // adding 2.5 so everything is shifted such that optitrack's origin is the center of the arena (while keeping all coordinates positive)
            // This was the old way for Optitrack coordinates
            //localOPTITRACKps.x = Optitrackdata[0]*FEETINONEMETER; // was 2.5 for size = 5
            //localOPTITRACKps.y = -1.0*Optitrackdata[1]*FEETINONEMETER+4.0;

            // This is the new coordinates for Motive
            localOPTITRACKps.x = -1.0*Optitrackdata[0]*FEETINONEMETER;
            localOPTITRACKps.y = Optitrackdata[1]*FEETINONEMETER+4.0;

            // make this a function
            Optitemp_theta = fmodf(localROBOTps.theta,(float)(2*PI));//(theta[trackableID]%(2*PI));
            OptitempOPTITRACK_theta = Optitrackdata[2];
            if (Optitemp_theta > 0) {
                if (Optitemp_theta < PI) {
                    if (OptitempOPTITRACK_theta >= 0.0) {
                        // THETA > 0, kal in QI/II, OT in QI/II
                        localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                    } else {
                        if (Optitemp_theta > (PI/2)) {
                            // THETA > 0, kal in QII, OT in QIII
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + OptitempOPTITRACK_theta*2*PI/360.0);
                        } else {
                            // THETA > 0, kal in QI, OT in QIV
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                } else {
                    if (OptitempOPTITRACK_theta <= 0.0) {
                        // THETA > 0, kal in QIII, OT in QIII
                        localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + PI + (PI + OptitempOPTITRACK_theta*2*PI/360.0);
                    } else {
                        if (Optitemp_theta > (3*PI/2)) {
                            // THETA > 0, kal in QIV, OT in QI
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + 2*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        } else {
                            // THETA > 0, kal in QIII, OT in QII
                            localOPTITRACKps.theta = (floorf((localROBOTps.theta)/((float)(2.0*PI))))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                }
            } else {
                if (Optitemp_theta > -PI) {
                    if (OptitempOPTITRACK_theta <= 0.0) {
                        // THETA < 0, kal in QIII/IV, OT in QIII/IV
                        localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                    } else {
                        if (Optitemp_theta < (-PI/2)) {
                            // THETA < 0, kal in QIII, OT in QII
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + OptitempOPTITRACK_theta*2*PI/360.0);
                        } else {
                            // THETA < 0, kal in QIV, OT in QI
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                } else {
                    if (OptitempOPTITRACK_theta >= 0.0) {
                        // THETA < 0, kal in QI/II, OT in QI/II
                        localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI - PI + (-PI + OptitempOPTITRACK_theta*2*PI/360.0);
                    } else {
                        if (Optitemp_theta < (-3*PI/2)) {
                            // THETA < 0, kal in QI, OT in QIV
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI - 2*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        } else {
                            // THETA < 0, kal in QII, OT in QIII
                            localOPTITRACKps.theta = ((int16_t)((localROBOTps.theta)/(2*PI)))*2.0*PI + OptitempOPTITRACK_theta*2*PI/360.0;
                        }
                    }
                }
            }
            *flag = 1;
        }
    }
    return localOPTITRACKps;
}

