//Kinecto.h : contains the declaration of the Kinecto class

#ifndef KINECTO_H
#define KINECTO_H

#include "WPILib.h"

class Kinecto
{
public:
	
	Kinecto();
	
	bool IsTracked();	//returns true if kinect is tracking a skeleton 
	
	float GetRForeArmAngle();	//returns the angle from the right elbow to the right wrist, returns 720.0 if skeleton is not tracked
	float GetLForeArmAngle();	//returns the angle from the left elbow to the left wrist, returns 720.0 if skeleton is not tracked
	
	bool IsRLegExtend();
	bool IsStanding();
	
	void PrintVar(bool minimize);
	
private:
	
	double AngleXY(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored);
	double CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax);
	
	//Classes
	Kinect* kinect;
		
	//Varibles
	static const float pi = 3.14159;
	
	float RForeArmAng;
	float LForeArmAng;
	
};

#endif //KINECTO_H
