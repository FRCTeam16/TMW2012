//Steer.h : contains declaration of Steer class

#ifndef STEER_H
#define STEER_H

class Steer
{
public:

//	Steer();
	Steer(double w, double x, double y, double a);

	//Calculates angles and ratios necessary for
	//turning a 4 wheel, swerve drive base	
	void Calc4WheelTurn(double radian);
	
	void SetA(const double A);	//Sets a new value of A
	
	const double GetFRRatio();	//Ratio of speed that should be given to the front right wheel
	const double GetFLRatio();	//Ratio of speed that should be given to the front left wheel
	const double GetRRRatio();	//Ratio of speed that should be given to the rear right wheel
	const double GetRLRatio();	//Ratio of speed that should be given to the rear left wheel
	
	const double GetThetaFL();	//Angle at which the front left wheel should point
	const double GetThetaFR();	//Angle at which the front right wheel should point
	const double GetThetaRC();
	const double GetThetaRL();	//Angle at which the rear left wheel should point
	const double GetThetaRR();	//Angle at which the rear right wheel should point
	
private:

	void LeftTurn4Wheels();	//Calculates a left hand turn
	void RightTurn4Wheels();	//Calculates a right hand turn
	
	//Varibles used in class

	//Varibles needed to be defined in constructor
	double pi;	//3.14159
	double A;	//A is the ratio of X to turn harder 
	double W;	//W is the distance form the mid-point of one back wheel to the mid-point of the other back wheel
	double X;	//X is the distance form the mid-point of the back wheels to the mid-point of the front wheels 
	double Y;	//Y is the distance from the mid-point of one front wheel to the mid-point of the other font wheel
	
	//Varibles dynamically defined
	double FL;	//FL, distance from Front Left Wheel to the center of rotation
	double FR;	//FR, distance from Front Right Wheel to the center of rotation
	double RL;	//RL, distance from Rear Left Wheel to the center of rotation
	double RR;	//RR, distance from Rear Right Wheel to the center of rotation
	
	double Z;	//Z, distance form A * X to center of rotation
	
	//Varibles accessable by user
	double thetaRC;	//Angle used to calculate all other angles
	
	double thetaFL;	//thetaFL, angle from Front Left Wheel to the center of rotation
	double thetaFR;	//thetaFR, angle from Front Right Wheel to the center of rotation
	double thetaRL;	//thetaRL, angle from Rear Left Wheel to the center of rotation
	double thetaRR;	//thetaRR, angle from Rear Right Wheel to the center of rotation

	double FRRatio;	//Ratio of Speed of Front Right wheel
	double FLRatio;	//Ratio of Speed of Front Left wheel
	double RRRatio;	//Ratio of Speed of Rear Right wheel
	double RLRatio;	//Ratio of Speed of Rear Left wheel

};

#endif	//STEER_H
