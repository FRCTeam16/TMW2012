//Pickup.h : contains declaration of Pickup class

#ifndef PICKUP_H
#define PICKUP_H

#include "WPILib.h"

class Pickup
{
public:
	
	Pickup();
	
	void BoxUp();
	void MaintainPos();
	void BoxDown();
	bool IsBoxDown();
	
	void BarIn();
	void BarStop();
	void BarOut();
	void RunBar(float Spd);
	float GetBarAmps();
	
	void EnableSaftey();
	void PrintVar(bool minimize);
	
	void HardStop(bool out);
	bool IsHardStop();
	
	void ExtStinger(bool extend);
	bool GetStinger();
	bool IsStingSafe();
	
	void RunRollex(int direction);
	
private:
	
	/*User Classes and Members*/
	
	float BarStallCount;
	
	/*Robot Classes*/
	
	DigitalInput IsUp;
	DigitalInput StingerReady;
	
	CANJaguar Bar;
	
	Solenoid Down;
	Solenoid Up;
	
	Solenoid Stop;
	
	Solenoid Stinger;
	
	Relay Rollex;
};

#endif	//PICKUP_H
