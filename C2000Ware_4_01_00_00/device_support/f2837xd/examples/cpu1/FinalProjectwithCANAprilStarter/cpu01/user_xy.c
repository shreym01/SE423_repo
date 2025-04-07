 /*    
     ____________________________________________________
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |                    |            |                 |
    |                    |            |                 |
    |                    |            |                 |
    |                    |            |                 |
    |                    |____________|                 |
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |                                                   |
    |____________________     y       __________________|
                              ^
                              |
                              |    
                              |------> x
                              (0,0) Theta Right hand rule
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "F2837xD_SWPrioritizedIsrLevels.h"
#include "driverlib.h"
#include "device.h"
#include "xy.h"


#define PI          3.1415926535897932384626433832795
#define HALFPI PI/2.0

float my_atanf(float dy, float dx)
{
	float ang;
    
	if (fabsf(dy) <= 0.001F) {
		if (dx >= 0.0F) {
			ang = 0.0F;
		} else {
			ang = PI;
		}
	} else if (fabsf(dx) <= 0.001F) {
		if (dy > 0.0F) {
			ang = HALFPI;
		} else {
			ang = -HALFPI;
		}
	} else {
		ang = atan2f(dy,dx);
	}
	return ang;
}

int16_t xy_control(float *vref_forxy, float *turn_forxy,float turn_thres, float x_pos,float y_pos,float x_desired,float y_desired,float thetaabs,float target_radius,float target_radius_near)
{
	float dx,dy,alpha;
	float dist = 0.0F;
	float dir;
	float theta;
	int16_t target_near = 0;
    float turnerror = 0;

		// calculate theta (current heading) between -PI and PI
	if (thetaabs > PI) {
		theta = thetaabs - 2.0*PI*floorf((thetaabs+PI)/(2.0*PI));
	} else if (thetaabs < -PI) {
		theta = thetaabs - 2.0*PI*ceilf((thetaabs-PI)/(2.0*PI));
	} else {
		theta = thetaabs;
	}

	dx = x_desired - x_pos;
	dy = y_desired - y_pos;
	dist = sqrtf( dx*dx + dy*dy );
	dir = 1.0F;

	// calculate alpha (trajectory angle) between -PI and PI
	alpha = my_atanf(dy,dx);

	// calculate turn error
	turnerror = theta - alpha;

	// check for shortest path
	if (fabsf(turnerror + 2.0*PI) < fabsf(turnerror)) turnerror += 2.0*PI;
	else if (fabsf(turnerror - 2.0*PI) < fabsf(turnerror)) turnerror -= 2.0*PI;

	if (dist < target_radius_near) {
		target_near = 1;
		// Arrived to the target's (X,Y)
		if (dist < target_radius) {
			dir = 0.0F;
			turnerror = 0.0F;
		} else {
			// if we overshot target, we must change direction. This can cause the robot to bounce back and forth when remaining at a point.
			if (fabsf(turnerror) > HALFPI) {
				dir = -dir;
			}            
			turnerror = 0;
		}
	} else {
		target_near = 0;
	}

	// vref is 1 tile/sec; but slower when close to target.  
	*vref_forxy = dir*MIN(dist,1);

    if (fabsf(*vref_forxy) > 0.0) {
        // if robot 1 tile away from target use a scaled KP value.  
        *turn_forxy = (*vref_forxy*2)*turnerror;
    } else {
        // normally use a Kp gain of 2
        *turn_forxy = 2*turnerror;
    }
    
    // This helps with unbalanced wheel slipping.  If turn value greater than turn_thres (I use 2) then just spin in place
	if (fabsf(*turn_forxy) > turn_thres) {
		*vref_forxy = 0;
	}
	return(target_near);
}

