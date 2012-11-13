//Pickup.cpp : contains definition of Pickup class

#include "Pickup.h"

Pickup::Pickup():
	
	IsUp(2),
	StingerReady(7),
	Bar(12),
	Down(1),
	Up(2),
	Stop(4),
	Stinger(5),
	Rollex(3, Relay::kBothDirections)
{
	BarStallCount = 0.0;
}
//Enable MotorSafety class
void Pickup::EnableSaftey()
{
	/*
	Bar.SetExpiration(10.0);
	Bar.SetSafetyEnabled(true);
	*/	
}
//Bring Box up
void Pickup::BoxUp()
{
	Up.Set(true);
	Down.Set(false);
}
//Bring Box down
void Pickup::BoxDown()
{
	Up.Set(false);
	Down.Set(true);
	Stop.Set(false);
}
//Returns true if Box is down
bool Pickup::IsBoxDown()
{
	/*
	//1 = Down, 0 = Up
	if(IsUp.Get() == 0)
		return false;
	else
		return true;
	*/
	return true;
}
//Maintains position of the box
void Pickup::MaintainPos()
{
	Down.Set(false);
	Up.Set(false);
}
//Pull balls into box
void Pickup::BarIn()
{
	Bar.Set(1.0);
}
//Stop beater bar
void Pickup::BarStop()
{
	Bar.Set(0.0);
}
//Push balls out of box
void Pickup::BarOut()
{
	Bar.Set(-1.0);
}
//Extend the hard stops
void Pickup::HardStop(bool out)
{
	Stop.Set(out);	//true = Engaged
}
//Returns state of hard stops
bool Pickup::IsHardStop()
{
	bool state = false;
	
	if(Stop.Get() == true)
		state = true;
	
	return state;
}
//Prints variables
//void Pickup::PrintVar(bool minimize)
//{
//	if(minimize == false)	//Print everything
//	{
//		SmartDashboard::Log(Bar.GetOutputVoltage(), "Bar Speed");
//		SmartDashboard::Log((INT32)IsUp.Get(), "Box Up");		
//	}
//	SmartDashboard::Log(this->IsStingSafe(), "IsStingerReady");
//}
//Runs beater bar at variable speed
void Pickup::RunBar(float Spd)
{
	Bar.Set(Spd);	
}
//Returns true if the beater bar is stalled 
float Pickup::GetBarAmps()
{
	return Bar.GetOutputCurrent();
}
void Pickup::ExtStinger(bool extend)
{
	Stinger.Set(extend);
}
bool Pickup::GetStinger()
{
	return Stinger.Get();
}
bool Pickup::IsStingSafe()
{
	return (bool)StingerReady.Get();
}
void Pickup::RunRollex(int direction)
{
	if(direction < -1)
		direction = -1;
	if(direction > 1)
		direction = 1;
	
	switch(direction)
	{
		case -1:
		{
			Rollex.Set(Relay::kForward);
			break;
		}
		case 0:
		{
			Rollex.Set(Relay::kOff);
			break;
		}
		case 1:
		{
			Rollex.Set(Relay::kReverse);
			break;
		}
	}
}
