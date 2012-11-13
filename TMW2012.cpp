#include "WPILib.h"
#include "GamePad.h"
#include "Swerve.h"
#include "Pickup.h"
#include "Shooter.h"
#include "Kinecto.h"
//#include "CameraAim.h"
#include "HOT_PID.h"
#include "math.h"

#include <iostream>
#include <vector>

using namespace std;

class TMW2012 : public IterativeRobot
{
private:
	
	/*WPILib Classes*/
	
	Watchdog Watch;

	//Driving Controls
	DriverStation *ds;
	Joystick stick;
	GamePad GPad;
	
	//Pneumatics
	Compressor AirPump;
	
	//LED
	Relay BotColor;
	
	//Camera
	//AxisCamera* Camera;
	
	/*Created Classes and Members*/
	
	//User Classes
	Swerve Drive;
	Pickup Box;
	Shooter Catapult;
	Kinecto kinect;
	
	HOT_PID CamAngPID;
	
//	CameraAim Camera;
	
	NetworkTable* CamTable;
	
	//Autonomous enumerations & control variables
	typedef enum {Kinect = 0, Stay, Bridge, Eater, Super}AutoTW;
	typedef enum {None = 0, CoOp, Ours}AutoBridge;
	typedef enum {Left = 0, Mid, Right}AutoBallLoc;
	typedef enum {Init = -1, Cock, Launch, Load, ToBridge, BoxUp, LoadBall, BoxDown, ToKey, Aim, Eat, ShootOnReturn, Done}AutoStep;
	
	AutoTW ATWMode;
	AutoBridge ABridge;
	AutoBallLoc ABSide;
	AutoStep AStep;

	AutoStep ASteps[];
	
	AutoStep AFeedSteps[];
	
	bool AStepInit;
	bool Is2ndBall;
	bool AStopFlag;
	
	float ATime;
	float ATimeDifference;
	
	//Flags
	bool F2BFlag;
	bool AxisFlag;
	bool LockFlag;
	bool HardStopFlag;
	bool BoxDownFlag;
	bool GyroReset;
	bool FireFlag;
	bool PrintSmDb;
	bool CatResetFlag;
	bool UpdateTarget;
	bool PosConInit;
	bool ResetCat;
	
	bool StingInitSenseFlag;
	bool StingerDriveMode;
	bool StingerSwitchFlag;
	bool StingerDirectionFlag;
	
	bool IsDriveStandard;
	
	bool BarGrab;
	bool Gone2Bridge;
	
	//Time Variables
	float StingerTime;
	float StingerInitTime;
	
	float LastShotTime;
	
	float AWaitTime;
		
	//Kinect
	const float KinMajicAng;	//Degrees
	
	//Thumbwheel
	int TW;
	int BarGrabScans;
	
	//Offsets
	float SteerOffset;
	float AxisOffset;
	
	int ACamTarHeight;
	
	/*Camera*/
	
	//Time
	float CamErTime;
	
	//Error
	int CamXError;
	
	//Positions
	const static float TeleCatPos = 82.0;
	
public:
	
	//Default Functions
	TMW2012();
	
	void RobotInit();
	void DisabledInit();
	void AutonomousInit();
	void TeleopInit();

	void TeleopContinuous();
	
	void DisabledPeriodic();
	void AutonomousPeriodic();
	void TeleopPeriodic();
	
	//Created Functions
	bool IsKinectStop();
	bool IsKinectFire();
	
	float GetSteerOffset();
	
	void UseCamPID(bool yes);
	int GetCamError();
	int GetCamArea();
	
	void SmDb(bool minimize);
	void PrintAutoModes();

};
TMW2012::TMW2012():
	
	stick(1),
	GPad(2),
	
	AirPump(1, 1),
	
	BotColor(2, Relay::kForwardOnly),
	
	CamAngPID(20.0, 0.0, 0.0, 0.1),
	
	KinMajicAng(45.0)
{
	//DriverSation
	ds = DriverStation::GetInstance();
	
	//Watchdog
	GetWatchdog().SetExpiration(5.0);
	
	//Compressor
	AirPump.Start();
	
	CamTable = NetworkTable::GetTable("camera");
/*	
	//Camera
	Camera = &AxisCamera::GetInstance();
	Camera->WriteResolution(AxisCamera::kResolution_320x240);
	Camera->WriteBrightness(0);
*/
	
}

void TMW2012::RobotInit()
{

	CamAngPID.SetSPLimits(0.0, 0.0);	//Pixels
	CamAngPID.SetPVLimits(320.0, -320.0);	//Pixels
	CamAngPID.SetMVLimits(60.0, -60.0);	//Degrees
	
	Drive.Init();
		
	F2BFlag = false;
	LockFlag = false;
	HardStopFlag = true;
	BoxDownFlag = Box.IsBoxDown();
	CatResetFlag = false;
	StingInitSenseFlag = false;
	StingerDriveMode = false;
	StingerSwitchFlag = false;
	StingerDirectionFlag = false;
	UpdateTarget = false;
	PosConInit = false;
	BarGrab = false;
	Gone2Bridge = false;
	
	IsDriveStandard = true;
	
	StingerTime = GetClock();
	StingerInitTime = GetClock();
	LastShotTime = 0.0;
	
	TW = 0;

	SteerOffset = 0.24;
	AxisOffset = 0.0;
	
	CamXError = 0;
	
	ACamTarHeight = 52;
}
void TMW2012::DisabledInit()
{
	Drive.Init();
		
	F2BFlag = false;
	LockFlag = false;
	HardStopFlag = true;
	StingerDriveMode = false;
	StingerSwitchFlag = false;
	StingerDirectionFlag = false;
	GyroReset = false;
	UpdateTarget = false;
	PosConInit = false;
	BarGrab = false;
	Gone2Bridge = false;
	
	IsDriveStandard = true;
	
	BoxDownFlag = Box.IsBoxDown();
	
	TW = 0;
	
	CamXError = 0;
	CamErTime = 0.0;

	if(SmartDashboard::GetBoolean("Found") == true)
		ACamTarHeight = SmartDashboard::GetNumber("TargetHeight");

	AxisOffset = 0.0;
	
	StingerTime = GetClock();
	StingerInitTime = GetClock();
}
void TMW2012::AutonomousInit()
{
	//Initialization
	Drive.Init();
	Drive.SetF2B(false);
	
	//Enumerations
	AStep = Init;

	StingInitSenseFlag = false;
	StingerDriveMode = false;
	StingerSwitchFlag = false;
	StingerDirectionFlag = false;
	
	//Varibles
	Is2ndBall = false;
	AStepInit = true;
	AStopFlag = false;
	ATime = GetClock();
	ATimeDifference = 0.0;
	BarGrab = false;
	Gone2Bridge = false;
	
	UpdateTarget = false;
	PosConInit = false;
	
	CamXError = 0;
	CamErTime = 0.0;

	if(SmartDashboard::GetBoolean("Found") == true)
		ACamTarHeight = SmartDashboard::GetNumber("TargetHeight");
	
	AxisOffset = 0.0;
	
	BarGrabScans = 0;
	StingerTime = GetClock();
	StingerInitTime = GetClock();
	
	//Flags
	F2BFlag = false;
	LockFlag = false;
	HardStopFlag = true;
	BoxDownFlag = Box.IsBoxDown();
	
	IsDriveStandard = true;
}
void TMW2012::TeleopInit()
{
	Drive.Init();
	Drive.SetF2B(true);
	
	F2BFlag = false;
	AxisFlag = false;
	LockFlag = false;
	HardStopFlag = true;
	FireFlag = false;
//	Box.BoxDown();
	BoxDownFlag = Box.IsBoxDown();
	PrintSmDb = false;
	StingInitSenseFlag = false;
	StingerDriveMode = false;
	StingerSwitchFlag = false;
	StingerDirectionFlag = false;
	PosConInit = false;
	BarGrab = false;
	Gone2Bridge = false;
	
	IsDriveStandard = true;
	
	UpdateTarget = false;
	CamXError = 0;
	CamErTime = 0.0;
	ResetCat = false;
	
	AxisOffset = 0.0;
	
	StingerTime = GetClock();
	StingerInitTime = GetClock();
}
void TMW2012::TeleopContinuous()
{
	if(stick.GetRawButton(2) == true || GPad.GetRawButton(9) == true)
	{
/*		if(CatResetFlag == false)
		{
			Catapult.Shoot(true);
			LastShotTime = GetClock();
			CatResetFlag = true;
		}
		else
		{
			Catapult.Shoot(false);
			Catapult.SetPos(83.0);
		}
*/
		Catapult.Shoot(true);
		Catapult.SetPos(TeleCatPos);
		LastShotTime = GetClock();
		Catapult.EnableWinch();
	}
	else
	{
		Catapult.Shoot(false);
//		CatResetFlag = false;
	}
}
void TMW2012::DisabledPeriodic()
{
	GetWatchdog().SetEnabled(true);
	GetWatchdog().Feed();
	
	GetWatchdog().SetEnabled(false);
	Box.EnableSaftey();
	
	//Get Kinect Stuff
	if(kinect.GetRForeArmAngle() < KinMajicAng && -KinMajicAng > kinect.GetRForeArmAngle())
		ABridge = None;
	else if(kinect.GetRForeArmAngle() > KinMajicAng)
		ABridge = CoOp;
	else
		ABridge = Ours;
	
	if(kinect.GetLForeArmAngle() < KinMajicAng && -KinMajicAng > kinect.GetLForeArmAngle())
		ABSide = Left;
	else if(kinect.GetLForeArmAngle() > KinMajicAng)
		ABSide = Right;
	else
		ABSide = Mid;

	//Thumbwheel Auto
	TW = 0;
	
	for(int i = 1, PlaceVal = 1; i < 5; i++, PlaceVal += PlaceVal)
	{
		if(ds->GetDigitalIn(i) == false)
			TW += PlaceVal;
	}
	
	ATWMode = Stay;	//Go to bride without Kinect by default
	
	if(TW == 0)
		ATWMode = Kinect;
	
	if(TW == 1)
		ATWMode = Stay;	//Don't use Kinect, don't move just shoot our two balls
	
	if(TW == 2)
		ATWMode = Bridge;	//Don't use the Kinect, get two balls off of bridge
	
	if(TW == 3)
		ATWMode = Eater;
	
	if(TW == 4)
		ATWMode = Super;
	
//	if(TW == 4)
	
	//Reset Gyro
	if(ds->GetDigitalIn(5) == false)
		GyroReset = true;
	
	if(GyroReset == true)
	{
		if(Drive.ZeroGyro(10.0) == true)
			GyroReset = false;
		
		for(int i = 1; i < 7; i++)
			ds->SetDigitalOut(i, true);
	}
	
	//Save Offsets for Steering Modules
	if(ds->GetDigitalIn(6) == false)
	{
		Drive.SaveSteerPotOffset();
		
		for(int i = 1; i < 7; i++)
			ds->SetDigitalOut(i, true);
	}
	else if(GyroReset == false)
	{
		for(int i = 1; i < 7; i++)
			ds->SetDigitalOut(i, false);		
	}
	
	if(stick.GetRawButton(11) == true)
		SteerOffset = this->GetSteerOffset();
	
	this->GetCamError();
	
//	SmartDashboard::Log(Box.IsBoxDown(), "IsBoxDown");
	
	if(stick.GetRawButton(8) == true)
		this->SmDb(true);
	
	Catapult.CamLightsOn(SmartDashboard::GetNumber("TargetXFromCenter"), 3);
}
void TMW2012::AutonomousPeriodic()
{
	GetWatchdog().SetEnabled(true);
	GetWatchdog().Feed();

	//Drive Control
	float BotSpd = 0.0;
	double BotStr = 90.0;
	const float FrCrab = 0.0;
	bool Crab = false;
	bool F2B = false;
	bool Axis = false;
	
	bool PosCon = false;
	
//	const float DistanceToBridge = -1.45;
	
	//Box Control
	bool RaiseBox = true;
	bool Bar = true;
	bool HardStop = false;
	
	//Catapult Control
	bool Shoot = false;
	float InitFirePos = 84.0;	//88
	float FirePos = TeleCatPos;	//82
	float BridgeFirePos = TeleCatPos;	//82
//	if(ATWMode == Side)
//		FirePos = 87.0;
	float LoadPos = TeleCatPos;
	
	
	//Should we stop moving
	bool StopAuto = false;
	
	//Are we using the Kinect
	if(ATWMode == Kinect)
	{
		StopAuto = this->IsKinectStop();	//returns true if we need to stop
		
		if(kinect.IsTracked() == false)
			ATWMode = Stay;
		
	}
		
	/*
	 * Create constant arrays to hold the order of the steps in auto
	 * 
	 */
	

	if(StopAuto == false)
	{
		//If we just returned from being stopped
		//restore difference between ATime and GetClock()
		if(AStopFlag == true)
		{
			ATime = GetClock() + ATimeDifference;
			AStopFlag = false;
		}
		
		switch(AStep)
		{
		case Init:
			{
				cout << "Init " << endl;
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				if(Is2ndBall == false)
					Bar = false;
				else
					Bar = true;
				
				if(Is2ndBall == true || ATime + 1.25 < GetClock())
					Box.RunRollex(-1);
				
				//Catapult Control
				Catapult.Shoot(false);
				
				if(Box.IsBoxDown() == true && ATime + 2.5 < GetClock())
				{
					ATime = GetClock();
					AStep = Cock;
					AStepInit = true;
				}
				
				break;
			}
		case Cock:
			{
				cout << "Cock " <<  "IsCocked = " << Catapult.IsCocked() << endl;
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;

				//Box Control
				RaiseBox = false;
				
				if(Is2ndBall == true)
					Bar = true;
				else
					Bar = false;
				
				Box.RunRollex(-1);
				
				//Catapult Code
				if(AStepInit == true)
				{
					if(Is2ndBall == true)	//if this is the second ball
					{
						Catapult.SetPos(FirePos);
						AStepInit = false;
					}
					else	//if this is the first ball
					{
						Catapult.SetPos(InitFirePos);
						AStepInit = false;
					}
				}
				
				bool KinNext = false;
				if(ATWMode == Kinect)
					KinNext = this->IsKinectFire();
				
				if((Is2ndBall == false && ATime + 2.0 < GetClock() && ATWMode != Kinect && Catapult.IsBallLoaded() == true) || 
						
						(AStepInit == false && /*ATime + 4.0 < GetClock() &&*/ Catapult.IsCocked() == true && ATWMode != Kinect && Catapult.IsBallLoaded() == true) ||
						
						(KinNext == true && Catapult.IsCocked() == true))// && ATime + 1.0 < GetClock())
				{
					ATime = GetClock();
					AStep = Launch;
					AStepInit = true;
					
					Box.RunRollex(0);
				}
				
				break;
			}
		case Launch:
			{				
				cout << "Shooting a ball ";
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				Bar = true;
			
				Box.RunRollex(1);
				
				if(AStepInit == true)
				{
					if(ATime + 0.1 > GetClock())// && Box.IsBoxDown() == true)
					{
						Catapult.Shoot(true);	//6-4-12 7:23 change from true to false
						RaiseBox = true;
						HardStop = true;
					}	
					else
					{
						Catapult.Shoot(false);	//6-4-12 7:23 change from false to true 
						
						if(ATime + 0.26 < GetClock())
						{
							AStepInit = false;
							
							if(Gone2Bridge == true)
								Catapult.SetPos(BridgeFirePos);
							else
								Catapult.SetPos(LoadPos);
						}
					}	
				}
				
				if(AStepInit == false)
				{
					cout << " Done shooting,";
					
					if(Is2ndBall == true)
					{
						switch(ATWMode)
						{
						case Bridge:
							{
								AStep = ToBridge;
								Drive.ZeroWhPos();
								break;
							}
						case Kinect:
							{
								AStep = Load;
								break;
							}
						case Eater:
							{
								AStep = Eat;
								break;
							}
						case Stay:
							{
								AStep = Done;
								break;
							}
						case Super:
							{
								if(Gone2Bridge == false)
									AStep = ToBridge;
								else
									AStep = ShootOnReturn;
								break;
							}
						}
						
						ATime = GetClock();
						AStepInit = true;
					}
					else if(Catapult.IsCocked() == true)
					{
						AStep = Load;
						ATime = GetClock();
						AStepInit = true;
					}
				}
					
				cout << endl;
				
				break;
			}
		case Load:
			{
				cout << "Load " << endl;
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				Bar = true;

				Box.RunRollex(-1);
				
				if(Catapult.IsBallLoaded() == true)
				{					
					if(ATime + 1.0 < GetClock())
					{
						Is2ndBall = true;
						
						ATime = GetClock();
						AStep = Cock;	//Used to go to Launch
						AStepInit = true;
						
						Box.RunRollex(0);
					}
				}
				else
					ATime = GetClock();
				
				break;
			}
		case ToBridge:
			{
				cout << "ToBridge, ";
				
				//Drive Control
				BotSpd = 0.5;	//.25
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				Drive.SetSteerMode(Swerve::S);
				
				PosCon = false;
				//BotSpd = DistanceToBridge;
								
				//Box Control
				RaiseBox = true;
				Bar = true;
				
				HardStop = true;
				
				//Catapult Control
				Shoot = false;
				
				Gone2Bridge = true;
				
				if(ATime + 3.5 < GetClock())	//6-4-12 2.5 changed to 3.5	//Drive.GetPosError(BotSpd) < 1)
				{
					AStep = LoadBall;
					AStepInit = true;
					ATime = GetClock();
				}
				
				
//				cout << "PosError = " << Drive.GetPosError(BotSpd);
//				cout << " GyroAngle = " << Drive.GyroAngle();
				cout << endl;
				
				break;
			}
		case BoxUp:
			{
				//Drive Control
				BotSpd = 0.5;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
								
				//Box Control
				RaiseBox = true;
				Bar = true;	//Have to run bar so next step will execute correctly
				
				HardStop = true;
				
				//Catapult Control
				Shoot = false;

				if(ATime + 2.0 < GetClock())
				{
					AStep = LoadBall;
					AStepInit = true;
					ATime = GetClock();
				}
				
				break;
			}
		case LoadBall:
			{
				cout << "Loading Ball ";
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = true;
				Bar = true;
				
				HardStop = true;
				
				//Catapult Control
				Shoot = false;
				
				if(Drive.IsWhPosConOn() == true && AStepInit == true)
				{
					PosCon = false;
					AStepInit = false;
				}
				else
				{
					PosCon = true;
				}
				
				if(Box.GetBarAmps() > 7.0)
					BarGrabScans = BarGrabScans + 1;
				else				
					BarGrabScans = 0;
				
				if(BarGrabScans > 2)
					BarGrab = true;
				
				if(!BarGrab)
					ATime = GetClock();
				
				if(ATime + 0.85 < GetClock())
				{
					if(ATWMode == Super)
						AStep = ToKey;
					else
						AStep = Done;
					
					AStepInit = true;
					ATime = GetClock();
					PosCon = false;
				}
				
				cout << "Bar Amps " << Box.GetBarAmps() << "BarGrabScans: " << BarGrabScans << endl;
				
				break;
			}			
		case ToKey:
			{
				cout << "ToKey ";
				
				//Drive Control
				BotSpd = -0.5;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				PosCon = true;
				BotSpd = 1.12;	//32
				
				//Box Control
				RaiseBox = false;
				Bar = true;
				
				HardStop = false;
				
				if(Catapult.IsBallLoaded() == false && Catapult.CanLoad())
					Box.RunRollex(-1);
				else
					Box.RunRollex(0);
				
				//Catapult Control
				Shoot = false;

				
				//y = mx + b
				//y = feet from goal in feet
				//x = height of target in pixels
				//b = ACamTarHeight or initial value of x at the beginning of Autonomous
				//m = (height at the bridge - the height at the key) / the distance to the bridge
				//y equals 0 when x equals ACamTarHeight
				//ACamTarHeight is approximately 53 pixels at 12 feet
				
				//float m = (ACamTarHeight - 42) / 149;
				
				//float y = (m * SmartDashboard::GetInstance()->GetInt("TargetHeight")) + ACamTarHeight;
				
				if(Drive.GetPosError(BotSpd) < 5)
				{
					if(ATime + .25 < GetClock())
					{
						if(Drive.GyroAngle()>4)
							AStep = Aim;
						else
							AStep = ShootOnReturn;
						
						AStepInit = true;
						ATime = GetClock();
						PosCon = false;
					}
				}
				else
					ATime = GetClock();
				
				cout << "BotPos = " << Drive.GetFWPos();
				cout << endl;
				
				break;
			}
		case Aim:
			{
				cout << "Aim: ";
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 0.0;
				PosCon = true;
				Drive.SetSteerMode(Swerve::A);
				
				if(UpdateTarget == false && SmartDashboard::GetBoolean("Found") == true)
				{
					CamXError = SmartDashboard::GetNumber("TargetXFromCenter");
					CamErTime = GetClock();
					UpdateTarget = true;
					Drive.ZeroWhPos();
				}
				
				//Execute
				if(Drive.AreModsOnTarget() == true && UpdateTarget == true)	//if modules are in correct position
				{
					BotSpd =  -(0.0005 * (float)CamXError);
					
					if(Drive.GetPosError(BotSpd) < .5)
					{	
						if(CamErTime + .15 < GetClock())
							UpdateTarget = false;
					}
					else
						CamErTime = GetClock();
							
				}
				
				
				//Box Control
				RaiseBox = false;
				Bar = true;
				
				HardStop = false;
				
				//Catapult Control
				Shoot = false;
				
				if(abs(SmartDashboard::GetNumber("TargetXFromCenter")) < 4)
				{
					if(ATime + .5 < GetClock())
					{
						AStep = ShootOnReturn;
						AStepInit = true;
						ATime = GetClock();
						PosCon = false;
					}
				}
				else
					ATime = GetClock();
				
				cout << "CamXError = " << CamXError;
				cout << "TarX2Cen = " << SmartDashboard::GetNumber("TargetXFromCenter");
				cout << ", BotSpd = " << BotSpd;
				cout << ", PosError = " << Drive.GetPosError(BotSpd);
				cout << ", AStepInit = " << AStepInit;
				
				cout << endl;
				
				break;
			}

		case Eat:
			{
				cout << "Eat....";
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				Bar = true;
				
				if(Catapult.IsBallLoaded() == false)
				{
					if(Catapult.CanLoad() == true)
						Box.RunRollex(-1);
				}
				else
					Box.RunRollex(0);
				
				//Catapult Control
				Shoot = false;

//				if(ATWMode == Bridge)
//					ATWMode = Eater;
				
				if(Catapult.IsBallLoaded() == true && Catapult.IsCocked() == true)
				{
					cout << "There's a Ball!!!!!!!!!!!!!!!!!! ";
					
					if(ATime + 1.0 < GetClock())
					{
						AStep = Launch;
						AStepInit = true;
						ATime = GetClock();
					}
				}
				else
					ATime = GetClock();
				
				cout << endl;
				
				break;
			}

		case ShootOnReturn:
			{
				cout << "ShootOnReturn....";
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				PosCon = true;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				Bar = true;
				
				if(Catapult.IsBallLoaded() == false)
				{
					if(Catapult.CanLoad() == true)
						Box.RunRollex(-1);
				}
				else
					Box.RunRollex(0);
				
				//Catapult Control
				Shoot = false;
		
				if(Catapult.IsBallLoaded() == true && Catapult.IsCocked() == true)
				{
					cout << "There's a Ball!!!!!!!!!!!!!!!!!! ";
					
					if(ATime + 1.0 < GetClock())
					{
						AStep = Launch;
						AStepInit = true;
						ATime = GetClock();
					}
				}
				else
					ATime = GetClock();
				
				cout << endl;
				
				break;
			}
	
		case Done:
			{
				cout << "Done " << endl;
				
				//Drive Control
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
				
				//Box Control
				RaiseBox = false;
				Bar = false;
				
				Box.RunRollex(0);
				
				//Catapult Control
				Shoot = false;

				break;
			}
		default:
			{
				BotSpd = 0.0;
				BotStr = 90.0;
				Crab = false;
				F2B = false;
				Axis = false;
			}
		}//switch				
	}
	else	//Human player has signaled to stop
	{
		BotSpd = 0.0;	//Stop robot
			
		ATimeDifference = ATime - GetClock();
		
		AStopFlag = true;
	}

	
	//Drive Control
	if(PosCon == true)
		Drive.UseWhPosCon(true);
	else
		Drive.UseWhPosCon(false);
	
//	if(AStep != Aim)
//	{
//		if(AStep == ToKey || AStep == Eat)
//			BotStr -= Drive.GyroAngle()/2;
//		else
			BotStr += Drive.GyroAngle()/2;
//	}
	
	Drive.Drive(BotSpd, FrCrab, BotStr);
	
	//Box Control
	if(RaiseBox == true)
		Box.BoxUp();
	else
		Box.BoxDown();
	
	if(Bar == true)
	{
		if(AStep == Eat || ToKey)
			Box.RunBar(0.5);
		else
			Box.RunBar(1.0);
	}
	else
		Box.RunBar(0.0);
	
	if(HardStop == true)
		Box.HardStop(true);
	else
		Box.HardStop(false);
	
	//Catapult Control
	if(Box.IsBoxDown() == true)
		Catapult.RunWinch();
	
	if(Catapult.IsBallLoaded() == true)
		BotColor.Set(Relay::kOn);
	else
		BotColor.Set(Relay::kOff);
	
//	if(stick.GetRawButton(8) == true)
//		this->SmDb(true);
}
void TMW2012::TeleopPeriodic()
{
	GetWatchdog().SetEnabled(true);
	GetWatchdog().Feed();
	
	IsDriveStandard = true;
	
	/*   Stinger Code	 */
	bool IsSafeSting = Box.IsStingSafe();
	
	if(IsSafeSting == true || StingInitSenseFlag == true)
	{
		StingInitSenseFlag = true;

		if(stick.GetRawButton(4) == true)
		{
			if(StingerDirectionFlag == false)
			{
				if(Box.GetStinger() == true)
				{
					Box.ExtStinger(false);
					StingerDriveMode = false;
				}
				else
				{
					Box.ExtStinger(true);
					StingerDriveMode = true;
				}
				
				StingerDirectionFlag = true;
			}
		}
		else
			StingerDirectionFlag = false;
	}
	else
	{
		Box.ExtStinger(false);
		StingerDriveMode = false;
		StingerDirectionFlag = false;
	}

	
	/*Drive Code*/
	float throttle = 0;
	if (stick.GetRawButton(5) == false)
		throttle = stick.GetY();
		
 	//Raw voltage range of the steering wheel is 0.5v to 2.7v
 	//here we subtract 0.5 from the input to give us a range of 0.0v to 2.2v
 	//which is easier to work with
	double AjustedSteering = ds->GetAnalogIn(2) - SteerOffset;//competetion driver station - 0.24; //practice driverstation = 0
		
	//Converting from a range of 0.0v to 2.2v to a range of 0 degress to 180 degrees
	//180 / 2.2 = 81.45
	double SteerDeg = AjustedSteering * 81.45; //practice driver station = 58.82; //competetion driver station = 81.45;

	//Sets Crab Mode
	if(stick.GetTrigger() == true && !stick.GetRawButton(5))
	{
		IsDriveStandard = false;		
		
		Drive.SetSteerMode(Swerve::C);
		
		if(SteerDeg < 25)
			SteerDeg = 0.0;
		
		if(SteerDeg > 158)
			SteerDeg = 180.0;
	}
		

	//Sets Axis Mode
	if(stick.GetRawButton(5) == true && !stick.GetTrigger() && !LockFlag)
	{
		//Initialize necessary variables and modes
		IsDriveStandard = false;
		
		Drive.SetSteerMode(Swerve::A);		//Drive.SetAxis(true);
		Drive.UseWhPosCon(true);		
		
/*		
		if(LastShotTime + 10 < GetClock())
		{
			Catapult.SetPos(55.0);
			ResetCat = true;
		}
		
		if(Catapult.GetPos() < 60 && ResetCat)
		{
			Catapult.SetPos(TeleCatPos);
			LastShotTime = GetClock();
			ResetCat = false;
		}
*/		
		//int CamXErIn = SmartDashboard::GetInstance()->GetInt("TargetXFromCenter");
		
		if(UpdateTarget == false)
		{
			if(!SmartDashboard::GetBoolean("IgnoreCamData")  && SmartDashboard::GetBoolean("Found") == true)
				CamXError = SmartDashboard::GetNumber("TargetXFromCenter");
			else
				CamXError = 0;
			CamErTime = GetClock();
			UpdateTarget = true;
			Drive.ZeroWhPos();
		}
		
		//Execute
		if(Drive.AreModsOnTarget() == true && UpdateTarget == true)	//if modules are in correct position
		{
			if(PosConInit == false)
				AxisOffset = SteerDeg;
			
			PosConInit = true;
			
			float difference = 0.0;
			
			if(PosConInit == true)
				difference = (SteerDeg - AxisOffset) / 4000;
			
			throttle =  -(0.0005 * (float)CamXError) + difference;
			
			if(Drive.GetPosError(throttle) < .3)
			{	
				if(CamErTime + .25 < GetClock())
					UpdateTarget = false;
			}
			else
				CamErTime = GetClock();
					
		}		
			
/*		
		if(PosConInit == false)
			AxisOffset = SteerDeg;
		
		if(Drive.AreModsOnTarget() == true && PosConInit == true)
		{
			float difference = SteerDeg - AxisOffset;
			
			throttle = difference / 1800;
		}
		
		PosConInit = true;
*/	
/*		if(LastShotTime + 10.0 < GetClock() && CatResetFlag == false)	//It has been more than 10 seconds since we shot last
		{
			Catapult.SetPos(55.0);	//Offset Catapult
			
			if(Catapult.IsCocked() == true)
				CatResetFlag = true;	//Reset Catpult
		}
		else if(CatResetFlag == true)
		{
			Catapult.SetPos(83.0);	//Reset Catapult
			
			LastShotTime = GetClock();	//Rest Stored Time
		}
*/
	}
	else
	{
		UpdateTarget = false;
		PosConInit = false;
		CamErTime = GetClock() - .5;
		AxisFlag = true;
		CamXError = 0;
		
		//Wheel Position Control
		if(stick.GetRawButton(7) == true)
			Drive.UseWhPosCon(true);
		else
			Drive.UseWhPosCon(false);
		
		CatResetFlag = false;
	}	
		
	//Sets Front Pivot Mode
	if(stick.GetRawButton(10) == true)
	{
		Drive.SetSteerMode(Swerve::FP);
		
		IsDriveStandard = false;
	}
	
	//Toggle for Front to Back Mode
	if(stick.GetRawButton(9) == true)
	{
		if(F2BFlag == false)
		{
			Drive.SwitchF2B();
			F2BFlag = true;
		}
	}
	else
	{
		F2BFlag = false;
	}
		
	if(stick.GetRawButton(6))
		Drive.ZeroWhPos();
			
	if(StingerDriveMode == true)
	{
		IsDriveStandard = false;
		
		Drive.SetSteerMode(Swerve::BL);		//Drive.SetCrab(false);
		Drive.UseWhPosCon(false);
				
		if(fabs(stick.GetY()) > 0.01)
		{
			Drive.SetSteerMode(Swerve::S);
			
			float throttle = stick.GetY();	//Drive
			float StingerWaitTime = 0.125;
			
			if(StingerTime + StingerWaitTime < GetClock())	//if we have waited long enough to toggle stinger
			{
				if(Box.GetStinger() == true)	//Toggle Stinger
					Box.ExtStinger(false);
				else
					Box.ExtStinger(true);
				
				StingerTime = GetClock();	//Reset StingerTime
			}
			
			Drive.Drive(throttle, 0.0, 90);	//Drive straight
			
		}
		else
		{
			Drive.Drive(0.0, 0.0, 180.0);	//Locks wheels
			Box.ExtStinger(true);
			StingerSwitchFlag = false;
			StingerTime = GetClock();
		}
	}
	else	
	{	
		/*********************Toggle for lock mode********************/		
		if(stick.GetRawButton(3) == true || (stick.GetRawButton(5) && stick.GetTrigger()))
		{
			if(LockFlag == false)
			{
				if(Drive.GetSteerMode() == Swerve::L)
					Drive.SetSteerMode(Swerve::S);
				else
					Drive.SetSteerMode(Swerve::L);
				
				LockFlag = true;
			}
		}
		else if(!stick.GetRawButton(5) && !stick.GetTrigger())
			LockFlag = false;

		if(Drive.GetSteerMode() == Swerve::L)
		{
			IsDriveStandard = false;
			throttle = 0.0;
		}
		
		
		
		if(IsDriveStandard == true)
			Drive.SetSteerMode(Swerve::S);

		
		Drive.Drive(throttle, stick.GetX(), SteerDeg);
		
		StingerTime = GetClock();
		StingerInitTime = GetClock();
	}
	
	
	/*Box Code*/
	
	if(GPad.GetRawButton(5) == true && Catapult.CanLoad() == true)
	{		
		Box.BoxUp();
		BoxDownFlag = false;
	}

	if(GPad.GetRawButton(7) == true)
	{
		Box.BoxDown();
		BoxDownFlag = true;
	}

	if(GPad.GetRawButton(5) == false && GPad.GetRawButton(7) == false && BoxDownFlag == Box.IsBoxDown())
		Box.MaintainPos();
	
	
	if(fabs(GPad.GetLeftY()) > 0.1)
	{
		Box.RunBar(-GPad.GetLeftY());
		
		if(GPad.GetRawButton(1) == true && Catapult.CanLoad() == true)
			Box.RunRollex(-1);
		else
			Box.RunRollex(1);
	}
	else
	{
		Box.BarStop();
		
		if(GPad.GetRawButton(1) == true && Catapult.CanLoad() == true)
			Box.RunRollex(-1);	
		else if(GPad.GetRawButton(2) == true)
			Box.RunRollex(1);
		else
			Box.RunRollex(0);
	}
		
		
	if(GPad.GetRawButton(6) == true)
	{
		Box.BoxUp();
		Box.HardStop(true);
	}
	
	/*Catapult Code*/
	
	if(GPad.GetRawButton(10) == true)
		this->Catapult.ResetFriedWinch();

	if(Box.IsBoxDown() == true)
	{
		if(GPad.GetRawButton(1) == true)
		{
//			Catapult.SetPos(82.0);
			Catapult.ShortShot(true);
		}
		
		if(GPad.GetRawButton(2) == true)
		{
//			Catapult.SetPos(80.00);
			Catapult.ShortShot(false);
		}
	
		if(GPad.GetRawButton(3) == true)
		{
			//Catapult.SetPos(TeleCatPos);	//85
			Catapult.SetPos(SmartDashboard::GetNumber("Proportional") + 85);
			Catapult.ShortShot(false);
		}

		if(GPad.GetRawButton(4) == true)
		{
//			Catapult.SetPos(95);
			Catapult.ShortShot(false);
		}
		
//		if(LastShotTime + 60.0 < GetClock())
//			Catapult.SetPos(87.0);
		
//		if(GPad.GetRawButton(8) == true)
//			Catapult.SetPos(Catapult.GetPos() - 2.0);
/*
		if(fabs(GPad.GetRightY()) > 0.1)
		{
//			if(GPad.GetRightY() > 0.1)
//				Catapult.SetPos(Catapult.GetPos() + 2.0);
			if(GPad.GetRightY() < -0.1)
				Catapult.SetPos(Catapult.GetPos() - 2.0);
		}
*/
		
//		if(GPad.GetRockerX() > 0.5)
//			Catapult.SetPos(Catapult.GetPos() + 2.0);
				
		
//		if(stick.GetRawButton(3) == true)
//			Catapult.SetPos(((this->GetCamArea() * -0.32) - 72.5));
		
		
		Catapult.RunWinch();	
	}
	else
		Catapult.StopWinch();
	
	if(Catapult.IsBallLoaded() == true)
		BotColor.Set(Relay::kOn);
	else
		BotColor.Set(Relay::kOff);
	
	//SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("TargetXFromCenter"), "CamError");
	//SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("TargetHeight"), "CamTargetHeight");
	//SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("TargetWidth"), "CamTargetWidth");
	//SmartDashboard::Log(Drive.AreModsOnTarget(),"ModsOnTarget");
	//SmartDashboard::Log(throttle,"Throttle");
	
	//if(stick.GetRawButton(9) == true)
	//{
	//	if(PrintSmDb == false)
	//		PrintSmDb = true;
	//	else
	//		PrintSmDb = false;
	//}
	
	//SmartDashboard::Log(Box.IsBoxDown(), "IsBoxDown");
	
	if(stick.GetRawButton(8) == true)
		this->SmDb(true);
	
	cout << "TargetHeight = " << SmartDashboard::GetNumber("TargetHeight") << endl;
	
	Catapult.CamLightsOn(SmartDashboard::GetNumber("TargetXFromCenter"), 3);
}

/*
 * User functions
 * 
 * */
//Gets Offset for Steering wheel
float TMW2012::GetSteerOffset()
{
	return ds->GetAnalogIn(2);
}

void TMW2012::UseCamPID(bool yes)
{
	
	//SmartDashboard::Log(yes, "CamIsShoot");
//	SmartDashboard::Log(SmartDashboard::GetInstance()->GetBoolean("Found"), "CamFound");
	
	if(yes == true)
	{
		//CamAngPID.Enable();
		
//		CamXError = SmartDashboard::GetInstance()->GetInt("TargetXFromCenter");
		
		//SmartDashboard::Log(CamXError, "CamError");	
		
		//int CamP = SmartDashboard::GetInstance()->GetInt("Proportional");
		//int CamI = SmartDashboard::GetInstance()->GetInt("Integral");
		//int CamD = SmartDashboard::GetInstance()->GetInt("Derivative");
		
		//float fCamP = (float)CamP / 100;
		//float fCamI = (float)CamI / 1000;
		//float fCamD = (float)CamD / 100;
		
		//SmartDashboard::Log(fCamP, "CamP");
		//SmartDashboard::Log(fCamI, "CamI");
		//SmartDashboard::Log(fCamD, "CamD");
		
		//CamAngPID.SetGains(fCamP, fCamI, fCamD);
		
		//SmartDashboard::Log(CamAngPID.GetMV(0.0, CamXError), "CamPIDOut");
	}
	else
		CamAngPID.Disable();

}
//Gets Camera info from SmartDashboard
int TMW2012::GetCamError()
{
	return SmartDashboard::GetNumber("TargetXFromCenter");
}
int TMW2012::GetCamArea()
{
	return SmartDashboard::GetNumber("TargetArea");
}
//Returns true if human player signals for robot to stop 
bool TMW2012::IsKinectStop()
{
	bool IsStop = false;
	
	if(kinect.GetLForeArmAngle() < -80 && kinect.GetRForeArmAngle() < -80)
		IsStop = true;

	return IsStop;
}

bool TMW2012::IsKinectFire()
{
	if(kinect.GetLForeArmAngle() > 80 && kinect.GetRForeArmAngle() > 80)
		return true;
	else
		return false;
}
//Prints debug variables to SmartDasboard
void TMW2012::SmDb(bool minimize)
{
/*
	if(minimize == false)
	{
		SmartDashboard::Log(Drive.IsGyroDrift(), "GyroDrift");
		
		//Auto
		this->PrintAutoModes();	
	}
	
	//Drive Debug
	SmartDashboard::Log(stick.GetY(), "Joystick Y");
//	SmartDashboard::Log(stick.GetX(), "Joystick X");
	
	SmartDashboard::Log(TW, "ThumbWheel");
	SmartDashboard::Log(GyroReset, "ZeroGyro");
	
	SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("Derivative"), "DRead");
*/	
	//Swerve
	Drive.PrintVar(minimize);
/*	
	//Kinect
	kinect.PrintVar(minimize);
	
	//Box
	Box.PrintVar(minimize);
	
*/	//Capapult
	Catapult.PrintVar(minimize);
	
	//Camera variables
//	int CamError = SmartDashboard::GetInstance()->GetInt("TargetXFromCenter");
//	int CamHeight = SmartDashboard::GetInstance()->GetInt("TargetHeight");
//	int CamARatio = SmartDashboard::GetInstance()->GetInt("ARatio");
//	SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("TargetXFromCenter"), "CamError");
//	SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("TargetHeight"), "CamTargetHeight");
//	SmartDashboard::Log(SmartDashboard::GetInstance()->GetInt("ARatio"), CamARatio);
}
void TMW2012::PrintAutoModes()
{
/*
	//Selected bridge
	if(ABridge == None)
		SmartDashboard::Log("None", "AutoBridge");
	if(ABridge == CoOp)
		SmartDashboard::Log("CoOp", "AutoBridge");
	if(ABridge == Ours)
		SmartDashboard::Log("Ours", "AutoBridge");
	
	//On which side of the bridge
	if(ABSide == Left)
		SmartDashboard::Log("Left", "AutoBrideSide");
	if(ABSide == Mid)
		SmartDashboard::Log("Middle", "AutoBrideSide");
	if(ABSide == Right)
		SmartDashboard::Log("Right", "AutoBrideSide");
	
	//Thumbwheel Modes
	if(ATWMode == Kinect)
		SmartDashboard::Log("Kinect", "AutoTWMode");
	if(ATWMode == Stay)
		SmartDashboard::Log("Stay", "AutoTWMode");
	if(ATWMode == Bridge)
		SmartDashboard::Log("Bridge", "AutoTWMode");
	
	SmartDashboard::Log(AStep, "AStep");
*/
}

START_ROBOT_CLASS(TMW2012);
