//Shooter.h contains declaration of Shooter class

#ifndef SHOOTER_H
#define SHOOTER_H

#include "WPILib.h"
#include "HOT_PID.h"

class Shooter
{
public:
	
	Shooter();
	
	
//	void Pullback();
//	void LoadBall();
	void Shoot(bool shoot);
	void ShortShot(bool yes);
	bool IsCocked();
	bool CanLoad();
//	void Engage();
	const float GetPos();
	const float GetWinchSP();
	void SetPos(float deg);
	
	void ResetFriedWinch();
	
	void RunWinch();
//	void RunWinch(float Spd);
	void StopWinch();
	
	bool IsBallLoaded();
	
	void PrintVar(bool minimize);
	void EnableWinch();
	
	void CamLightsOn(int error, int tolerance);
	
private:

	CANJaguar PullBack1;
	CANJaguar PullBack2;
	
	Solenoid Release;

//	Solenoid HardStop;
	
	Solenoid Left;
	Solenoid Center;
	Solenoid Right;
	
	AnalogChannel Pos;
	AnalogChannel Load;
	
	HOT_PID WinchPID;
	
	DigitalInput IsBallLoad;
	
	float KShotSP;
	float BShotSP;
	
	float WinchSP;
	float ReleaseTime;
	float StopWinchTime;
	float WinchDeadStop;
	float WinchTime;
	float WinchPullBackLimit;
	float WinchPos;

	bool WinchStarted;
	
	//Signals
	bool FriedWinch;
	bool Cocked;
};

#endif //SHOOTER_H

