// pseudocode2.cpp : Defines the entry point for the console application.
//

#include "mathclock.h"
#include <stdio.h>		/* printf */
#include <math.h>       /* sin */

CMathClock MathClock;


void CMathClock::Reset()
{
    this->m_timesTable = 1;
	this->m_movementRate = 0.2f; 
}


void CMathClock::Loop()
{
    // printf("CMathClock::Loop() timesTable=%.1f\n", this->m_timesTable);
	// printf("ID, targetPoint, \tAngle\n");

	static float PI = 3.14159265;

    for (unsigned short offset = 0; offset < SETTING_MAX_NODES; offset++)
    {
		// Calulate what node to look at.
		float pointingOffset = (offset * this->m_timesTable);
		float targetPoint = fmod( pointingOffset, SETTING_MAX_NODES);
		float deltaPoint = fmod( (pointingOffset - offset), SETTING_MAX_NODES);
		
		// Check for no movement 
		float thetaDegrees = 0; 
		if (deltaPoint != 0.0f) {
			// I don't understand any of this. This was made by @Luthor2k
			// https://github.com/funvill/mathclock/commits/master?author=Luthor2k
			// It finds the angle between to points on a circle. Basic grade 11 math. 
			// It works! 
			float beta = deltaPoint * ((2 * PI) / SETTING_MAX_NODES);
			float rho = atan(sin(beta) / (1 - cos(beta)));
			float theta = (PI / 2) - rho;
			thetaDegrees = (theta / PI) * 180;
		}

		// Print it for debug
		// printf("[%02d], %.1f (%.1f),\t%.1f\n", offset, targetPoint, deltaPoint, thetaDegrees);

		// Move the servo to look at the next point.
		this->m_node[offset] = thetaDegrees;
    }

    this->m_timesTable += SETTING_MOVEMENT_RATE;
}

float CMathClock::GetNodeAngle(unsigned short nodeID)
{
	if( nodeID > SETTING_MAX_NODES ) {
		return 0.0f; 
	}
    return this->m_node[nodeID];
}
