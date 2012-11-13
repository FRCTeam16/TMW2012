//Shooter.cpp : contains definition of Shooter class

#include "Shooter.h"
#include "math.h"

Shooter::Shooter():
	
	PullBack1(10),
	PullBack2(11),
	Release(3),
//	HardStop(6),
	Left(6),
	Center(7),
	Right(8),
	Pos(6),
	Load(7),
	WinchPID(0.1, 0.0, 0.0, 0.0),
	IsBallLoad(8),
	ReleaseTime(0.0),
	StopWinchTime(0.0),
	WinchDeadStop(3.193),
	WinchTime(0.0),
	WinchPullBackLimit(2.37),
	WinchPos(0.0)
{
	WinchPID.SetSPLimits(100, 0);
	WinchPID.SetPVLimits(100, 0);
	WinchPID.SetMVLimits(+1.0, -1.0);
	WinchPID.Disable();

//	SmDb = SmartDashboard::GetInstance();
	
//	KShotSP = SmDb->GetDouble("KeyShotSP");
//	BShotSP = SmDb->GetDouble("BarrierShotSP");
	
	FriedWinch = false;
}
/*
void Shooter::Pullback()
{	
	if(this->IsCocked() == false)
	{
		PullBack1.Set(WinchPID.GetMV(WinchSP, ((Pos.GetAverageVoltage()* -1)+5.5)));
		PullBack2.Set(WinchPID.GetMV(WinchSP, ((Pos.GetAverageVoltage()* -1)+5.5)));
			
	
	}
}
*/
//Sets Set Point for the Winch
void Shooter::SetPos(float SP)
{
	if(SP < 25.0)
		SP = 25;
	if(SP > 98.0)
		SP = 98.0;
		
	if(StopWinchTime <= GetClock() || SP != WinchSP)
	WinchPID.Enable();
	StopWinchTime = GetClock() + 4.0;
	WinchSP = SP;
	
	cout << "Winch Position Set to: " << WinchSP << endl;
}
//Returns Position of Winch
const float Shooter::GetPos()
{
	return WinchPos;
}
//Returns current Set Point for the winch
const float Shooter::GetWinchSP()
{
	return WinchSP;
}
//Fires the Catapult
void Shooter::Shoot(bool shoot)
{
	if(shoot == true)
	{
		Release.Set(true);
		ReleaseTime = GetClock();
		Cocked = false;
	}
	
	if(ReleaseTime + .15 < GetClock())
	{
		Release.Set(false);
	}
}

void Shooter::ShortShot(bool yes)
{
//	HardStop.Set(yes);
}
//Returns true if Catapult is cocked
bool Shooter::IsCocked()
{
	if(fabs(WinchSP - WinchPos) < 10.0) //10	//1.0 changed to 5.0 for practice bot
		return true;
	else
		return false;
}
bool Shooter::CanLoad()
{
	if(WinchPos > 45)
		return true;
	else
		return false;
}
/*
//Engage clutch
void Shooter::Engage()
{
	Release.Set(false);
}
*/
//Defry Winch
void Shooter::ResetFriedWinch()
{
	FriedWinch = false;
	WinchTime = 0.0;
}
/*
//Manually control Winch
void Shooter::RunWinch(float Spd)  
{
	float FriedPoint = 50.0;
	
	if(PullBack1.GetOutputCurrent() > FriedPoint || PullBack2.GetOutputCurrent() > FriedPoint || FriedWinch == true)
	{
		if(WinchTime < GetClock() && WinchStarted == false)
		{
			WinchTime = GetClock() + 1.5;
			WinchStarted = true;
		}
		else if(WinchTime < GetClock())
			FriedWinch = true;			

		if(FriedWinch == true)
		{
			PullBack1.Set(0.0);
			PullBack2.Set(0.0);
		}
	}
	else
	{
		if(Pos.GetAverageVoltage() < 2.45)
		{
			PullBack1.Set(-fabs(Spd));
			PullBack2.Set(-fabs(Spd));
		}
		else
		{
			PullBack1.Set(Spd);
			PullBack2.Set(Spd);
			WinchStarted = false;
			WinchTime = 0.0;
			FriedWinch = false;
		}
	}
}
*/
//Automaticly contol Winch
void Shooter::RunWinch()
{
	float FriedPoint = 50.0;
	
	WinchPos = 100-((Pos.GetAverageVoltage()-WinchPullBackLimit)/(WinchDeadStop-WinchPullBackLimit))*100;
	
	if(PullBack1.GetOutputCurrent() > FriedPoint || PullBack2.GetOutputCurrent() > FriedPoint || FriedWinch == true)
	{
		if(WinchTime < GetClock() && WinchStarted == false)
		{
			WinchTime = GetClock() + 1.5;
			WinchStarted = true;
		}
		else if(WinchTime < GetClock())
			FriedWinch = true;			

		if(FriedWinch == true)
		{
			PullBack1.Set(0.0);
			PullBack2.Set(0.0);
		}
	}
	else if(StopWinchTime > GetClock())
	{
		PullBack1.Set(WinchPID.GetMV(WinchSP, WinchPos));
		PullBack2.Set(WinchPID.GetMV(WinchSP, WinchPos));
		WinchStarted = false;
		WinchTime = 0.0;
		FriedWinch = false;
		Cocked = false;
	}			
	else 
	{
		WinchPID.Disable();
		PullBack1.Set(0.0);
		PullBack2.Set(0.0);
		Cocked = true;
	}
/*	
	if(WinchPos - Pos.GetAverageVoltage() < -1.0 && PullBack1.GetOutputVoltage() < 0)
	{
		PullBack1.Set(0.0);
		PullBack2.Set(0.0);
	}
*/	
}
void Shooter::StopWinch()
{
	PullBack1.Set(0.0);
	PullBack2.Set(0.0);
}
bool Shooter::IsBallLoaded()
{
	return !IsBallLoad.Get();
}
void Shooter::PrintVar(bool minimize)
{
	if(minimize == false)
	{
//	SmartDashboard::Log(PullBack1.GetOutputVoltage(), "WinchOut");
/*	SmartDashboard::Log(KShotSP, "ActiveKSP");	//Current Winch Set Point for Key
	SmartDashboard::Log(BShotSP, "ActiveBSP");	//Current Winch Set Point for Barrier
	SmartDashboard::Log(this->IsCocked(), "IsWinchCocked");
	SmartDashboard::Log(WinchPID.GetMV(WinchSP, WinchPos), "WinchPIDOut");
	SmartDashboard::Log(PullBack1.GetOutputCurrent(), "Winch1Amps");
	SmartDashboard::Log(PullBack2.GetOutputCurrent(), "Winch2Amps");
*/
	}
	
//	SmartDashboard::Log(WinchSP, "WinchSP");	//Set Point currently used for the Winch
//	SmartDashboard::Log(100-((Pos.GetAverageVoltage()-WinchPullBackLimit)/(WinchDeadStop-WinchPullBackLimit))*100, "WinchPos");	//WinchPos
//	SmartDashboard::Log((Load.GetAverageVoltage() * 20), "WinchLoad");
//	SmartDashboard::Log(WinchPID.GetMV(WinchSP, WinchPos), "WinchPIDOut");
//	SmartDashboard::Log(FriedWinch, "IsWinchFried");
//	SmartDashboard::Log(this->IsBallLoaded(), "IsBallLoaded");
	
//	SmartDashboard::Log(PullBack1.GetOutputVoltage(), "PB1 VoltOut:");
//	SmartDashboard::Log(PullBack2.GetOutputVoltage(), "PB2 VoltOut:");
//	SmartDashboard::Log(PullBack1.GetOutputCurrent(), "PB1 AmpOut:");
//	SmartDashboard::Log(PullBack2.GetOutputCurrent(), "PB2 AmpOut:");
}
void Shooter::EnableWinch()
{
	StopWinchTime = GetClock() + 4.0;
	WinchPID.Enable();
}
//Turns lights on to indicate if we are aimed at the goal or not
//using Solenoid outputs 6, left; 7, on target; and 8, right
void Shooter::CamLightsOn(int error, int tolerance)
{
	if(abs(error) < abs(6))
	{
		Left.Set(false);
		Center.Set(true);
		Right.Set(false);
	}
	else
	{
		if(error < 0)
		{
			Left.Set(true);
			Center.Set(false);
			Right.Set(false);
		}
		else
		{
			Left.Set(false);
			Center.Set(false);
			Right.Set(true);
		}
	}
}
