// pseudocode2.cpp : Defines the entry point for the console application.
//

#include "mathclock.h"
#include <SoftwareSerial.h>

CMathClock MathClock;

void CMathClock::Reset()
{
    this->m_timesTable = 2;
}

void CMathClock::Loop()
{
    // Serial.print("CMathClock::Loop() timesTable=");
    // Serial.println(this->m_timesTable);
    // Serial.println("ID, Point at, Difference, Angle");

    for (unsigned short offset = 0; offset < SETTING_MAX_NODES; offset++)
    {
		// Calulate what node to look at.
		unsigned short pointingOffset = (offset * this->m_timesTable);
		float pointAt = pointingOffset % SETTING_MAX_NODES;

		float difference = (pointingOffset - offset) % SETTING_MAX_NODES;

		// Based on where the node is located,
		// This is wrong but its close enugh.
		float angle = ((360 / SETTING_MAX_NODES) / 2) * difference;

		// Print it for debug
		/*
		Serial.print(offset);
		Serial.print("\t");
		Serial.print(pointAt);
		Serial.print("\t");
		Serial.print(difference);
		Serial.print("\t");
		Serial.print(angle);
		Serial.println("");
		*/

		// Move the servo to look at the next point.
		this->m_node[offset] = angle;
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


