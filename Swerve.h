//Swerve.h : contains declaration of Swerve class

#ifndef SWERVE_H
#define SWERVE_H

#include "WPILib.h"
#include "HOT_PID.h"
#include "RAWCConstants.h"
#include "Steer.h"

class Swerve
{	
public:
	
	Swerve();
	
	void EnableSaftey();	//commetted out could be used in place the Watchdog
	void Init();	//initializes various PID loops etc...	
	
	/**********************Driving******************/
	
	/*	Indicates various driving modes:
	 *	S = Swerve, the standard mode of driving, pivot point in the center of the robot
	 *	C = Crab, move laterally
	 *	A = Axis, turns wheels perpendicular to the center of the robot
	 *	FP = Front Pivot, sets pivot point between the front wheels
	 *	L = Lock, sets wheels perpendicular to the center of the robot and doesn't allow the robot to drive
	 *	BL = Bridge Lock, sets rear wheels perpendicular to center of the robot and the front wheels parallel to the center
	 */
	
	typedef enum {S = 1, C, A, FP, L, BL}SteerMode;
	
	void Drive(float JoyY, float JoyX, double SteerDeg);	//Steers and drives, must be called every scan
	
	bool SaveSteerPotOffset();	//Saves offsets of steering modules, returns true when complete
	
	void SetSteerMode(SteerMode Mode);	//Sets the current steering mode
	SteerMode GetSteerMode();	//Returns the current steering mode
	
	void SetF2B(bool IsF2B);	//Sets state of F2B
	void SwitchF2B();	//Toggles state of F2B
	bool GetF2B();	//returns state of F2B
	
	bool AreModsOnTarget();	//returns true if all wheel modules are on target
	
	/***************Position Control****************/
		
	void UseWhPosCon(bool yes);	//Enables PID control of wheel positions
	bool IsWhPosConOn();	//returns boolean indicating if we are currently in Wheel Position Control
	void ZeroWhPos();	//Zeros encoder reading wheel postion
	float GetFWPos();	//returns current distance from Quad Encoder on the Front Wheel
	float GetPosError(float throttle);	//Returns error from set point to current position
	
	/*********************Gyro*********************/
	
	float GyroAngle();	//returns current angle reading from gyro
	float GyroError(float AngleSP, float SpeedLimit);
	bool ZeroGyro(float InitTime); //Zeros Gyro, returns true when complete
	
	/*******************Debug**********************/
	
	void PrintVar(bool minimize);	//Prints Debug Variables
	
private:
		
	/*Custom Members*/
	
		/*Classes*/
		Steer DriveCalc;
			
		/*Functions*/
		void PotRollAdjust(double ModPos[]);
		
		bool IsModPosGood(double ModPos[], double ModSP[]);
		
		/*Members*/	
		static const double pi = 3.14159;	//value of pi
		
		//Drive Modes
		SteerMode StMode;
		
		bool Crab;
		bool F2B;
		bool Axis;
		bool FrontPivot;
		bool Lock;
		
		//Used to save wheel module offsets
		RAWCConstants* File;
		static const int StOffAvLoops = 256;
		
		float FLOffset;
		float FROffset;
		float RLOffset;
		float RROffset;
		
		float* pFLOffset;
		float* pFROffset;
		float* pRLOffset;
		float* pRROffset;
		
		//Flags
		bool GyroZeroFlag;
		
		bool ModsOnTarget;
		
		int StRoll[4];
				
		//Initialization Times
		float GyroZeroTime;
		
		//Gyro Drift Test
		float GDTime;
		float GDGyroAng;
		bool IsGyroDrifting;
		
		//Safe Rollover
		double LastPos[4];
		
		//Wheel Position Control Flag
		bool WhPosCon;
		
	/*Robot Classes*/
	
		/*Motor Classes*/
	
		//Drive Classes
		CANJaguar FLDrive;
		CANJaguar FRDrive;
		CANJaguar RLDrive;
		CANJaguar RRDrive;
		
		//Steering Classes
		CANJaguar FLSteer;
		CANJaguar FRSteer;
		CANJaguar RLSteer;
		CANJaguar RRSteer;
	
		/*Control Classes*/
	
		//PID Control
		HOT_PID FLPID;
		HOT_PID FRPID;
		HOT_PID RLPID;
		HOT_PID RRPID;
		
		//Wheel Module Position
		AnalogChannel FLPos;
		AnalogChannel FRPos;
		AnalogChannel RLPos;
		AnalogChannel RRPos;
		
		//Gyro
		AnalogChannel gyroChannel;
		Gyro gyro;
		
		//Wheel Position Control
		Encoder FWPos;
		
		HOT_PID WhPID;
		HOT_PID WhAPID;
};



#endif //SWERVE_H

