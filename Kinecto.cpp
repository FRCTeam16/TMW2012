//Kinecto.cpp : contains the definition of the Kinecto class

#include "Kinecto.h"
#include "math.h"

Kinecto::Kinecto()
{
	kinect = Kinect::GetInstance();
}

//returns true if kinect is tracking a skeleton
bool Kinecto::IsTracked()
{
	bool isTracked = false;
	
	//if kinect is tracking a skeleton
	if (kinect->GetTrackingState() == Kinect::kTracked)
		isTracked = true;
	else
		isTracked = false;
	
	return isTracked;
}
//returns the angle from the right elbow to the right wrist
//returns 720.0 if skeleton is not tracked
float Kinecto::GetRForeArmAngle()
{
	AngleAB = 720.0;
	
	if(this->IsTracked() == true)
	{
		AngleAB = this->AngleXY(kinect->GetSkeleton().GetElbowRight(), kinect->GetSkeleton().GetWristRight(), false);
		
		if(AngleAB > 90)
			AngleAB = 90;
		if(AngleAB < -90)
			AngleAB = -90;
	}
	
	RForeArmAng = AngleAB;
	return AngleAB;
}

//returns the angle from the left elbow to the left wrist
//returns 720.0 if skeleton is not tracked
float Kinecto::GetLForeArmAngle()
{
	AngleAB = 720.0;
	
	if(this->IsTracked() == true)
	{
		AngleAB = this->AngleXY(kinect->GetSkeleton().GetElbowLeft(), kinect->GetSkeleton().GetWristLeft(), true);
		
		if(AngleAB > 90)
			AngleAB = 90;
		if(AngleAB < -90)
			AngleAB = -90;
	}
	
	LForeArmAng = AngleAB;
	return AngleAB;	
}

bool Kinecto::IsStanding()
{
	bool IsStanding = true;
	
	if(kinect->GetSkeleton().GetAnkleRight().y > -0.3)
		IsStanding = false;
	
	return IsStanding;
}

bool Kinecto::IsRLegExtend()
{
	bool IsExtended = false;
	
	if(kinect->GetSkeleton().GetHipRight().x - kinect->GetSkeleton().GetAnkleRight().x < -0.3)
		IsExtended = true;
		
	return IsExtended;
}
/*
void Kinecto::PrintVar(bool minimize)
{
	if(minimize == false)
	{
		SmartDashboard::Log(LForeArmAng, "KinLeftArmPos");
		SmartDashboard::Log(RForeArmAng, "KinRightArmPos");
		SmartDashboard::Log(kinect->GetSkeleton().GetAnkleRight().y, "KinRightAnkY");
		SmartDashboard::Log(kinect->GetSkeleton().GetHipRight().x - kinect->GetSkeleton().GetAnkleRight().x, "KinHip2Ank");
		SmartDashboard::Log(kinect->GetSkeleton().GetHipRight().x, "KinRightHipX");
		SmartDashboard::Log(kinect->GetSkeleton().GetAnkleRight().x, "KinRightAnkX");
	}
	
	SmartDashboard::Log(this->IsStanding(), "KinStanding");
	SmartDashboard::Log(this->IsRLegExtend(), "KinKicked");
}

double Kinecto::AngleXY(Skeleton::Joint origin, Skeleton::Joint measured, UINT8 mirrored)
{
    return (atan2((measured.y- origin.y), (mirrored) ? (origin.x - measured.x) : (measured.x - origin.x)) * 180/pi);
}

double Kinecto::CoerceToRange(double input, double inputMin, double inputMax, double outputMin, double outputMax)
{
	// Determine the center of the input range
	double inputCenter = fabs(inputMax - inputMin) / 2 + inputMin;
	double outputCenter = fabs(outputMax - outputMin) / 2 + outputMin;
	
	// Scale the input range to the output range
	double scale = (outputMax - outputMin) / (inputMax - inputMin);
	
	// Apply the transformation
	double result = (input + -inputCenter) * scale + outputCenter;
	
	// Constrain to the result range
	return max(min(result, outputMax), outputMin);
}
*/
