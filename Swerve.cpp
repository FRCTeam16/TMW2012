//Steer.cpp : contains definition of Steer class

#include "Swerve.h"
#include "math.h"

Swerve::Swerve():
	
	DriveCalc(9.375, 23.75, 9.375, 0.5),
	
	FLDrive(2),
	FRDrive(3),
	RLDrive(4),
	RRDrive(5),
	
	FLSteer(6),
	FRSteer(7),
	RLSteer(8),
	RRSteer(9),
	
	FLPID(19.0, 0.0, 0.0, 0.1),
	FRPID(19.0, 0.0, 0.0, 0.1),
	RLPID(19.0, 0.0, 0.0, 0.1),
	RRPID(19.0, 0.0, 0.0, 0.1),
	
	FLPos(2),
	FRPos(3),
	RLPos(4),
	RRPos(5),
	
	gyroChannel(1),
	gyro(&gyroChannel),
	
	FWPos(3, 4, true, Encoder::k1X),
	WhPID(0.2, 0.0, 0.0, 0.1),
	WhAPID(.3, 0.003, 0.0, .01)
{
	//Create file to save Steering Pots
	File = RAWCConstants::getInstance();

	//Gyro maintenance
	GyroZeroFlag = false;
	GyroZeroTime = 0.0;
	GDTime = 0.0;
	GDGyroAng = 0.0;	
	
	//Diameter of wheel = 6"
	//Circumference = 6*pi
	//Ratio from encoder to wheel = 5 : 1
	//128 pulses per revolution in 1x mode
	//0.0295 = ((6 * pi) / 5 ) / 128)
	FWPos.SetDistancePerPulse(0.0295);
	FWPos.Start();
	
	double ModPos[] = {FLPos.GetVoltage(), FRPos.GetVoltage(), RLPos.GetVoltage(), RRPos.GetVoltage()};
	for(int i = 0; i < 5; i++)
		LastPos[i] = ModPos[i];
	
	for(int i = 0; i < 4; i++)
		StRoll[i] = 0;
	
	this->Init();
}
//Enable MotorSafety class, commetted out could be used in place the Watchdog
void Swerve::EnableSaftey()
{
	/*
	
	float Expire = 10.0;
	
	FLDrive.SetExpiration(Expire);
	FLDrive.SetSafetyEnabled(true);
	
	FRDrive.SetSafetyEnabled(Expire);
	FRDrive.SetSafetyEnabled(true);
	
	RLDrive.SetExpiration(Expire);
	RLDrive.SetSafetyEnabled(true);
	
	RRDrive.SetExpiration(Expire);
	RRDrive.SetSafetyEnabled(true);
	
	*/
}
//initializes various PID loops etc...	
void Swerve::Init()
{
	/*Initialize PID Loops here*/
	
	//Front Left Module PID
				  	//max, min
	FLPID.SetSPLimits(4.9, 0.1);
					//max, min
	FLPID.SetPVLimits(4.9, 0.1);
					 //max, min
	FLPID.SetMVLimits(+5.0, -5.0);
	
	//Front Right Module PID
	                //max, min
	FRPID.SetSPLimits(4.9, 0.1);
					//max, min
	FRPID.SetPVLimits(4.9, 0.1);
					 //max, min
	FRPID.SetMVLimits(+5.0, -5.0);

	//Rear Left Module PID
				   	//max, min
	RLPID.SetSPLimits(4.9, 0.9);
					//max, min
	RLPID.SetPVLimits(4.9, 0.1);
					 //max, min
	RLPID.SetMVLimits(+5.0, -5.0);
	
	//Rear Right Module PID
					//max, min
	RRPID.SetSPLimits(4.9, 0.1);
					//max, min
	RRPID.SetPVLimits(4.9, 0.1);
					 //max, min
	RRPID.SetMVLimits(+5.0, -5.0);
	
	//Enable PID Loops
	FLPID.Enable();
	FRPID.Enable();
	RLPID.Enable();
	RRPID.Enable();
	
	Crab = F2B = Axis = FrontPivot = false;
	
	//Use Previous Steering Offsets
	FLOffset = File->getValueForKey("FLOff");
	FROffset = File->getValueForKey("FROff");
	RLOffset = File->getValueForKey("RLOff");
	RROffset = File->getValueForKey("RROff");
	
	//Controls the position of the wheels
					  //Max,  Min
	WhPID.SetSPLimits(1000.0, -1000.0);
	WhPID.SetPVLimits(1000.0, -1000.0);
	WhPID.SetMVLimits(0.9, -0.7);

	WhPID.Disable();
	WhPosCon = false;

	WhAPID.SetSPLimits(1000.0, -1000.0);
	WhAPID.SetPVLimits(1000.0, -1000.0);
	WhAPID.SetMVLimits(0.6, -0.6);

	WhAPID.Disable();
	
	ModsOnTarget = false;
}
//Steers and drives, must be called every scan
void Swerve::Drive(float JoyY, float JoyX, double SteerDeg)
{
	float Speed = JoyY;
	float FCRatio = JoyX;
	
	//Converting degrees to radians,
	//To convert degrees to radians, multiply number of degrees by pi/180
	double SteeringRadians = ((SteerDeg * pi) / 180);
	
	/*
	 * In Standard Drive:
	 * 
	 * 				Position	Speed
	 * Front Left	VoltSP[0]	WheelSpd[0]
	 * Front Right	VoltSP[1]	WheelSpd[1]
	 * Rear Left	VoltSP[2]	WheelSpd[2]
	 * Rear Right	VoltSP[3]	WheelSpd[3]
	 * Rear Center	VoltSP[4]	WheelSpd[4]
	 * 
	 */
	
	double ModSP[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
	float WheelSpd[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

	int ModSPLen = 5;
	
	switch(StMode)
	{
	case S:
		{
			DriveCalc.SetA(0.5);	//Set Pivot Point to center of robot
			//Calc4WheelTurn() expects radians
			DriveCalc.Calc4WheelTurn(SteeringRadians);

			ModSP[0] = ((-2.5 / pi) * DriveCalc.GetThetaFL()) + 3.75;
			ModSP[1] = ((-2.5 / pi) * DriveCalc.GetThetaFR()) + 3.75;
			//ModSP[4] = ((-2.5 / pi) * DriveCalc.GetThetaRC()) + 3.75;
			ModSP[2] = ((-2.5 / pi) * DriveCalc.GetThetaRL()) + 3.75;
			ModSP[3] = ((-2.5 / pi) * DriveCalc.GetThetaRR()) + 3.75;

			WheelSpd[0] = DriveCalc.GetFRRatio() * Speed * -1;
			WheelSpd[1] = DriveCalc.GetFLRatio() * Speed;
			WheelSpd[2] = DriveCalc.GetRLRatio() * Speed * -1;
			WheelSpd[3] = DriveCalc.GetRRRatio() * Speed;
			
			break;
		}
	case C:
		{
			for(int i = 0; i < 5; i++)
				ModSP[i] = ((-2.5 / pi) * SteeringRadians) + 3.75;
		
			WheelSpd[0] = (Speed * -1) - FCRatio;
			WheelSpd[1] = Speed - FCRatio;
			WheelSpd[2] = (Speed * -1) - FCRatio;
			WheelSpd[3] = Speed - FCRatio;
			
			break;
		}
	case A:
		{
			DriveCalc.SetA(0.5);	//Set Pivot Point to center of robot
			//Calc4WheelTurn() expects radians
			DriveCalc.Calc4WheelTurn(0.0);	//Set to hard left turn

			ModSP[0] = ((-2.5 / pi) * (DriveCalc.GetThetaFL()+pi)) + 3.75;
			ModSP[1] = ((-2.5 / pi) * DriveCalc.GetThetaFR()) + 3.75;
			//ModSP[4] = ((-2.5 / pi) * DriveCalc.GetThetaRC()) + 3.75;
			ModSP[2] = ((-2.5 / pi) * (DriveCalc.GetThetaRL()-pi)) + 3.75;
			ModSP[3] = ((-2.5 / pi) * DriveCalc.GetThetaRR()) + 3.75;

			WheelSpd[0] = .5*(((SteerDeg / 90) - 1)*-1);
			WheelSpd[1] = .5*((SteerDeg / 90) - 1);
			WheelSpd[2] = .5*(((SteerDeg / 90) - 1)*-1);
			WheelSpd[3] = .5*((SteerDeg / 90) - 1);
			
			break;
		}
	case FP:
		{
			DriveCalc.SetA(1.0);	//Set Pivot Point to front wheels
			//Calc4WheelTurn() expects radians
			DriveCalc.Calc4WheelTurn(SteeringRadians);

			ModSP[0] = ((-2.5 / pi) * DriveCalc.GetThetaFL()) + 3.75;
			ModSP[1] = ((-2.5 / pi) * DriveCalc.GetThetaFR()) + 3.75;
			//ModSP[4] = ((-2.5 / pi) * DriveCalc.GetThetaRC()) + 3.75;
			ModSP[2] = ((-2.5 / pi) * DriveCalc.GetThetaRL()) + 3.75;
			ModSP[3] = ((-2.5 / pi) * DriveCalc.GetThetaRR()) + 3.75;

			WheelSpd[0] = DriveCalc.GetFRRatio() * Speed * -1;
			WheelSpd[1] = DriveCalc.GetFLRatio() * Speed;
			WheelSpd[2] = DriveCalc.GetRLRatio() * Speed * -1;
			WheelSpd[3] = DriveCalc.GetRRRatio() * Speed;
			
			break;
		}
	case L:
		{
			ModSP[0] = 1.25;	//Left	Boxside		Locks wheel like below:
			ModSP[2] = 1.875;	//Left				-	-
			ModSP[1] = 1.25;	//Right Boxside			
			ModSP[3] = 3.125;	//Right				\	/

			for(int i = 0; i < ModSPLen; i++)
				WheelSpd[i] = 0.0;
			
			break;
		}
	case BL:
		{
			ModSP[0] = 1.25;	//Left	Boxside		Locks wheel like below:
			ModSP[2] = 1.875;	/*Left				/	\	*/	
			ModSP[1] = 1.25;	//Right Boxside			
			ModSP[3] = 3.125;	/*Right				/	\	*/

			for(int i = 0; i < ModSPLen; i++)
				WheelSpd[i] = 0.0;
			
			break;
		}
	default:
		{
			DriveCalc.SetA(0.5);
			for(int i = 0; i < ModSPLen; i++)
				ModSP[i] = 2.5;

			for(int i = 0; i < ModSPLen; i++)
				WheelSpd[i] = 0.0;
			
			break;
		}
	}


	if(WhPosCon == true)
	{	
		float AdjSpd = Speed * 100;
				
		if(F2B == true || StMode == A)
		{
			if(StMode != A)
			{
				float error = -WhPID.GetMV(AdjSpd, (float)FWPos.GetDistance());
				WheelSpd[1] = WheelSpd[3] = -error;	//Right Side
				WheelSpd[0] = WheelSpd[2] = error;	//Left Side
			}
			else
			{
				float error = -WhAPID.GetMV(AdjSpd, (float)FWPos.GetDistance());
				WheelSpd[1] = WheelSpd[3] = -error;	//Right Side
				WheelSpd[0] = WheelSpd[2] = -error;	//Left Side
			}
		}
		else
		{
			float error = -WhPID.GetMV(AdjSpd, (float)FWPos.GetDistance());
			
			WheelSpd[1] = WheelSpd[3] = error;	//Right Side
			WheelSpd[0] = WheelSpd[2] = -error;	//Left Side
		}
	}	

	float margin = 0.5;
	ModsOnTarget = false;

	//Detect and correct for rollover
	double ModPos[] = {FLPos.GetVoltage(), FRPos.GetVoltage(), RLPos.GetVoltage(), RRPos.GetVoltage()};
	
	for(int i = 0; i < 4; i++)
	{
		if(ModPos[i] - LastPos[i] < -3.0)
			StRoll[i]++;
		
		if(ModPos[i] - LastPos[i] > 3.0)
			StRoll[i]--;
		
		LastPos[i] = ModPos[i];
	}
	
	for(int i = 0; i < 4; i++)
		ModPos[i] += (StRoll[i] * 5.0); 
	
	//Switches front and back
	if(F2B == true)
	{

		FLSteer.Set(FLPID.GetMV(ModSP[3], (ModPos[0] - FLOffset)) / 5);
		FRSteer.Set(FRPID.GetMV(ModSP[2], (ModPos[1] - FROffset)) / 5);
		RLSteer.Set(RLPID.GetMV(ModSP[1], (ModPos[2] - RLOffset)) / 5);
		RRSteer.Set(RRPID.GetMV(ModSP[0], (ModPos[3] - RROffset)) / 5);


		FLDrive.Set(WheelSpd[3]);
		FRDrive.Set(WheelSpd[2]);
		RLDrive.Set(WheelSpd[1]);
		RRDrive.Set(WheelSpd[0]);

		//Checks to see if wheel modules are on target
		if(	margin > fabs(ModSP[3] - (ModPos[0] - FLOffset)) &&
			margin > fabs(ModSP[2] - (ModPos[1] - FROffset)) &&
			margin > fabs(ModSP[1] - (ModPos[2] - RLOffset)) &&
			margin > fabs(ModSP[0] - (ModPos[3] - RROffset))
			)
			ModsOnTarget = true;


	}
	else
	{
		FLSteer.Set(FLPID.GetMV(ModSP[0], (ModPos[0] - FLOffset)) / 5);
		FRSteer.Set(FRPID.GetMV(ModSP[1], (ModPos[1] - FROffset)) / 5);
		RLSteer.Set(RLPID.GetMV(ModSP[2], (ModPos[2] - RLOffset)) / 5);
		RRSteer.Set(RRPID.GetMV(ModSP[3], (ModPos[3] - RROffset)) / 5);
		
		FLDrive.Set(WheelSpd[0]);
		FRDrive.Set(WheelSpd[1]);
		RLDrive.Set(WheelSpd[2]);
		RRDrive.Set(WheelSpd[3]);

		//Checks to see if wheel modules are on target
		if(	margin > fabs(ModSP[0] - (ModPos[0] - FLOffset)) &&
			margin > fabs(ModSP[1] - (ModPos[1] - FROffset)) &&
			margin > fabs(ModSP[2] - (ModPos[2] - RLOffset)) &&
			margin > fabs(ModSP[3] - (ModPos[3] - RROffset))
			)
			ModsOnTarget = true;

	}
}
//Saves offsets of steering modules, returns true when complete
bool Swerve::SaveSteerPotOffset()
{	
	FLOffset = FROffset = RLOffset = RROffset = 0.0;
	
	for(int i = 0; i <= StOffAvLoops; i++)
	{
		FLOffset += (FLPos.GetAverageVoltage() - 2.5) / StOffAvLoops;
		FROffset += (FRPos.GetAverageVoltage() - 2.5) / StOffAvLoops;
		RLOffset += (RLPos.GetAverageVoltage() - 2.5) / StOffAvLoops;
		RROffset += (RRPos.GetAverageVoltage() - 2.5) / StOffAvLoops;
	}

	File->restoreData();
	
	File->insertKeyAndValue("FLOff", FLOffset);
	File->insertKeyAndValue("FROff", FROffset);
	File->insertKeyAndValue("RLOff", RLOffset);
	File->insertKeyAndValue("RROff", RROffset);
	
	File->save();

	return true;
}
//Sets the current steering mode
void Swerve::SetSteerMode(Swerve::SteerMode Mode)
{
	StMode = Mode;
}
//Returns the current steering mode
Swerve::SteerMode Swerve::GetSteerMode()
{
	return StMode;
}
//Sets state of F2B
void Swerve::SetF2B(bool IsF2B)
{
	F2B = IsF2B;
}
//Toggles F2B Mode
void Swerve::SwitchF2B()
{
	if(F2B == false)
		F2B = true;
	else
		F2B = false;
}
//returns state of F2B
bool Swerve::GetF2B()
{
	return F2B;
}
//returns true if all wheel modules are on target
bool Swerve::AreModsOnTarget()
{
	return ModsOnTarget;
}

/****************************Position Control************************/

//Enables PID control of wheel positions
void Swerve::UseWhPosCon(bool yes)
{
	if(yes == true)
	{
		if(WhPosCon == false)
			this->ZeroWhPos();
		
		WhPosCon = true;
		WhPID.Enable();
		WhAPID.Enable();
	}
	else
	{
		WhPosCon = false;
		WhPID.Disable();
		WhAPID.Disable();
	}
}
//returns boolean indicating if we are currently in Wheel Position Control
bool Swerve::IsWhPosConOn()
{
	return WhPosCon;
}
//Zeros encoder reading wheel postion
void Swerve::ZeroWhPos()
{
	FWPos.Reset();
}
//returns current distance from Quad Encoder on the Front Wheel
float Swerve::GetFWPos()
{
	return FWPos.GetDistance();
}
//Returns error from set point to current position
//throttle's range is from 1.0 to -1.0
float Swerve::GetPosError(float throttle)
{
	return fabs(100*throttle - (float)FWPos.GetDistance());
}

/***********************************Gyro**********************************/

//returns current angle reading from gyro
float Swerve::GyroAngle()
{
	if(FWPos.GetDirection() == true)
		return -gyro.GetAngle();
	else
		return gyro.GetAngle();
}
//returns error from gyro
float Swerve::GyroError(float AngleSP, float SpeedLimit)
{
	float Pos = gyro.GetAngle();
	float PConBand = 30;
	
	float Error = Pos - AngleSP;
	
	if(Error > 0)
	{
		if(Error > PConBand)
			Error = SpeedLimit;
		else
			Error = (Error * SpeedLimit) / PConBand;
	}
	else
	{
		if(Error < -PConBand)
			Error = -PConBand;
		else
			Error = (Error * SpeedLimit) / PConBand;
	}
	
	return Error;
}
//Zeros Gyro, returns true when complete
bool Swerve::ZeroGyro(float InitTime)
{
	bool Done = false;
	
	if(GyroZeroFlag == false)
	{
		gyroChannel.InitAccumulator();
		GyroZeroFlag = true;
		GyroZeroTime = GetClock();
	}
	else
	{
		if(GetClock() > GyroZeroTime + InitTime)
		{
			INT64 value;
			UINT32 count;
			gyroChannel.GetAccumulatorOutput(&value, &count);
			
			INT32 center = (INT32)((float)value / (float)count + 0.5);
			
			gyroChannel.SetAccumulatorCenter(center);
			gyroChannel.ResetAccumulator();

			GyroZeroFlag = false;
			Done = true;
		}
	}
	
	return Done;

}

/*********************************Debug***********************************/

//prints debug variables
//if minimize is true fewer variables will be displayed
void Swerve::PrintVar(bool minimize)
{
	if(minimize == false)
	{		
//		SmartDashboard::Log(StRoll[0], "FLRoll");
//		SmartDashboard::Log(StRoll[1], "FRRoll");
//		SmartDashboard::Log(StRoll[2], "RLRoll");
//		SmartDashboard::Log(StRoll[3], "RRRoll");
		
		//Wheel Offsets
//		SmartDashboard::Log(FLOffset, "FLOffset");
//		SmartDashboard::Log(FROffset, "FROffset");
//		SmartDashboard::Log(RLOffset, "RLOffset");
//		SmartDashboard::Log(RROffset, "RROffset");
				
		//Wheel Position
//		SmartDashboard::Log(WhPosCon, "IsWheelPosEnabled");
	}
	
	//Module Positions
//	SmartDashboard::Log(FLPos.GetVoltage(), "FLPos");
//	SmartDashboard::Log(FRPos.GetVoltage(), "FRPos");
//	SmartDashboard::Log(RLPos.GetVoltage(), "RLPos");
//	SmartDashboard::Log(RRPos.GetVoltage(), "RRPos");

	//Wheel Position
//	SmartDashboard::Log(FWPos.GetDistance(), "FWPosition");
	
	//Gyro
//	SmartDashboard::Log(gyro.GetAngle(), "GyroAngle");
}
