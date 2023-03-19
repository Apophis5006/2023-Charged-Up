/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PS4Controller.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>
#include <frc/Preferences.h>
#include <frc2/command/PIDCommand.h>

#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalOutput.h>
#include "AHRS.h"

// #define CAMERA

#ifdef CAMERA
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cameraserver/CameraServer.h>
#endif

#define PVALUE 1.50
#define IVALUE 0.000
#define DVALUE 500000.00

#define START 10
#define STOP  11
#define MOVE  12
#define ARM_AUTO 13
#define BALANCE 14
#define WAIT_AUTO 15
#define MOVE_TIMED 16

#define NUMAUTOLINES 30
int (*AutoArray)[8];

#define IDXX 7 /* x offset from 0,0 for index wheel FL=7, FR=17, RL=7, RR=17 */
#define IDXY -17 /*y offset from 0,0 for index wheel FL=-7, FR=-7, RL=-17, RR=-17 */ 

//Flag so we don't reinit Gyros etc when switching from Auto to Telop
int RobotInitialized=0;
int AutoArraySet=0;
//Arm Positions
#define TRAVEL_POS 0
#define HP_PICKUP 1
#define HP_PICK_DROP 2
#define FLOOR_PICKUP 3
#define MID_SCORE 4
#define TOP_SCORE 5
#define CONE_VERTICAL 6
#define SHUTE_PICKUP 7
//Auto
#define AUTO_FLOOR 8 
#define AUTO_TOP 9
int ArmPosition = TRAVEL_POS;
int ArmManual = 0;
#define CONE 0
#define CUBE 2
int ObjectType = CONE;

#define INTAKE_IDLE 0
#define INTAKE_IN 1
#define INTAKE_EJECT 2
#define INTAKE_HOLD 3
int AutoIntake = INTAKE_IDLE;

// autonomous barrel race  RED 1
#define BRPW 70

const std::string BALANCE_SELECTION_STRING = "Balance";
int BALANCE_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,      0,       0,        0,            0}, //Start at midfield location
			{ARM_AUTO,   3000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,HP_PICKUP,	 0,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,HP_PICKUP,	 0,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,TOP_SCORE,	 0,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,TOP_SCORE,	 0,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,TOP_SCORE,	 0,		   0,  INTAKE_EJECT},
			{WAIT_AUTO,  500,		0,		0,TOP_SCORE,	 0,		   0,  INTAKE_EJECT},
			{ARM_AUTO,   2000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_EJECT},
			{WAIT_AUTO,  500,		0,		0,TOP_SCORE,	 0,		   0,  INTAKE_EJECT},
			// {MOVE_TIMED, 	0,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_EJECT},
			// {MOVE,	 	 250,		5,		30,		0,		 30,		   0,  INTAKE_IDLE},
			{BALANCE,	 500,		0,		20,		0,		 100,		   0,  INTAKE_EJECT}, //was 25 pwr
			{STOP,       0,         0,      0,      0,       0,        0,            0},	//STOP
};


const std::string VL_SELECTION_STRING = "Veer Left";
int VL_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,      0,       0,        0,  INTAKE_IN}, //Start at midfield location
			{ARM_AUTO,   3000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,HP_PICKUP,	 0,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,HP_PICKUP,	 0,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,AUTO_TOP,	 CONE,		   0,  INTAKE_IN},
			{WAIT_AUTO,	 1000,		0,		0,AUTO_TOP,	 CONE,		   0,  INTAKE_IN},
			{ARM_AUTO,   2000,		0,		0,AUTO_TOP,	 CONE,		   0,  INTAKE_EJECT},
			{WAIT_AUTO,  500,		0,		0,AUTO_TOP,	 CONE,		   0,  INTAKE_EJECT},
			{ARM_AUTO,   2000,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_EJECT},
			{WAIT_AUTO,  500,		0,		0,TRAVEL_POS,	 0,		   0,  INTAKE_EJECT},
			{MOVE,	 	 250,		0,		30,		0,		 150,	   0,  INTAKE_IN},
			{ARM_AUTO,   3000,		0,		0,AUTO_FLOOR,	 CUBE,		   0,  INTAKE_IN},
			{MOVE,	 	 10,	    5,		15,		-16,	 210,	   0,  INTAKE_IN},
			{ARM_AUTO,   3000,		0,		0,TRAVEL_POS,	 CUBE,		   0,  INTAKE_IN},
			{MOVE,	 	 250,		5,		30,	  -22,		 140,		  180,  INTAKE_IN},
			{ARM_AUTO,   3000,		0,		0,AUTO_FLOOR,	 CUBE,		   0,  INTAKE_IN},
			{MOVE,	 	 250,		5,		30,	  -22,		 -22,		  180,  INTAKE_IN},
			{STOP,       0,         0,      0,      0,       0,        0,       INTAKE_EJECT},	//STOP
}; 

const std::string MOVE_TEST_SELECTION_STRING = "Move Test";
int MOVE_TEST_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,      0,       0,        0,            0}, //Start at midfield location
			{MOVE,	   500,		    5,	   10,		0,	     50,	   180,  INTAKE_IDLE},
		//    {BALANCE, 	 500,		5,	    30,		0,	 50,		   0,  INTAKE_IDLE},
			// {MOVE,	 	 250,		5,		20,		-30,		40,		   0,  INTAKE_IDLE},
			// {MOVE,	 	 250,		5,		20,		-30,		0,		   0,  INTAKE_IDLE},
			// {MOVE,	 	 250,		5,		20,		0,		 0,		   0,  INTAKE_IDLE},
			{STOP,       0,         0,      0,      0,       0,        0,            0},	//STOP
};

const std::string NO_MOVE_SELECTIONSTRING = "No Move";
int NO_MOVE_AutoArray[NUMAUTOLINES][8]={
	{START,      0,         0,      0,      0,       0,        0,            0},
	{STOP,       0,         0,      0,      0,       0,        0,            0},	//STOP
};

//Motor controller defines
#define TIMEOUT 10
#define NOTIMEOUT 0
//Joystick defines
#define DIR 0
#define ROT 1
#define ARM 2

//PS4 Controller defines
#define OP_CONTROLLER 2

//-------------------CAN BUS DEFINES---------------
//Swerve CAN bus defines for steering motors
#define RRS 6
#define FRS 7
#define FLS 10
#define RLS 11
//Swerve CAN bus defines for drive motors
#define FLD 14
#define FRD 13
#define RLD 12
#define RRD 15

#define WRIST 8
#define SHOULDER 1
#define INTAKE 9


//For swerve code, define motor data locations in the arrays
#define RR 0
#define FR 1
#define FL 2
#define RL 3
#define ALL -1 /* all drive channels */
#define IDX  (FR) /* use FL Left Wheel as index */
#define PI 3.141592654
#define SW_L 21.75   //Distance between wheels from front to back
#define SW_W 21.75   //Distance between wheels from side to side

int TeleStarted=1;
double Travel;
int FirstPass=1,UseYTravel;
//Realtime values of robot status
int FieldCentric=0;   //1= feild centric driving
float RobotAngle=0.0; //angle from Gyro
float RobotPitch=0.0; //pitch from Gyro, used to balance robot
float PitchOffset; //offset of pitch to set zero on init
frc2::PIDController AutoRotatePID{1.1,0,0.0}; //PID controller for wall snapping
int IntakeState=0;
int UpdateCount=0;    //counter to slow screen update so it does not take too many CPU cycles
int SelectedPosition = -1;
int RampHit=0;

//Swerve Control variables  RR, FR, FL, RL
double FLZero,FRZero,RRZero,RLZero = 0;
float SpeedPolarity[4]={1.0,1.0,1.0,1.0}; //which way is forward on wheels (switches to speed up swerve rotations)
float ActDir[4]={0.0,0.0,0.0,0.0};        //storage for actual angles of wheel from motor controller encoder feedback
float TargetDir[4]={0.0,0.0,0.0,0.0};     //target degrees of each wheel
float ModDir[4]={0.0,0.0,0.0,0.0};        //rotational modification degrees of each wheel to adjust target
float ModSpd[4]={0.0,0.0,0.0,0.0};        //speed modification degrees of each wheel to adjust target
double SWRVY;							  //Swerve Control Variable--Represents Y coordinate of drive joystick
double SWRVX;                             //Swerve Control Variable--Represents X coordinate of drive joystick
double SWRVZ;                             //Swerve Control Variable--Represents X coordinate of rotate joystick
float delta =0.0;                         //local variable for determining best way(least delta degrees) to meet rotatioanl target


//Auto Pilot variables
    float TargetAngle;
	float CurrentAngle;
	//float RotPwr;
	int AutoLine=0;
	double RobotX[4]={0.0,0.0,0.0,0.0},RobotY[4]={0.0,0.0,0.0,0.0};
	double AutoDriveX, AutoDriveY, AutoDriveZ;



class Robot : public frc::TimedRobot {
  public:
    void AutonomousPeriodic() override {
	    int i;
	    if(RobotInitialized==0) { //Initialize once only
		    RobotInit();
			
	    }
	    if(AutoArraySet==0) { //First time through
            AutoArraySet=1;
		    AutoLine=0;
			for(i=0;i<4;i++){
				OldPosition[i]=GetLocation(i);
			}
			
			std::string selectedAuto = AutoChooser.GetSelected();
			if(selectedAuto == BALANCE_SELECTION_STRING) AutoArray=BALANCE_AutoArray;	
			else if(selectedAuto == VL_SELECTION_STRING) AutoArray=VL_AutoArray;
			else if(selectedAuto == MOVE_TEST_SELECTION_STRING) AutoArray=MOVE_TEST_AutoArray;
			else AutoArray=NO_MOVE_AutoArray;
			
	   }
	   AutoTime.Start();
	
		RunShoulder();
		RunWrist();
		RunIntake();

		// UpdateDriverScreen();
        
		ReadGyro();              //get the angle of the robot in to variable "RobotAngle"
		TrackRobot();			//Track the Robot's FL wheel X,Y position around the field
		AutoStateMachine();      //Get updated directional commands from autopilot
		if(!FirstPass) {		// don't try to control anything when state changes
			SwerveControl();         //convert auto pilot commands to joystick motions
			SwerveDrive();           //Update the speed and direction of the swerve drive motors
		}

  }
    void TeleopPeriodic() override {
	  int i;
		if(RobotInitialized==0) {
			RobotInit();
			for(i=0;i<4;i++){
				OldPosition[i]=GetLocation(i);
			}
		}
		RobotInitialized=100;
	    ReadGyro();
		TrackRobot();			//Track the Robot's FL wheel X,Y position around the field
	    SwerveControl(); 
		SwerveDrive();

		UpdateDriverScreen();

		// UpdateScreenAuto(0,0,0,0,0,0);

		TrackRobot();

		OperatorControl();


		RunShoulder();
		RunWrist();
		RunIntake();

  }

	void TestPeriodic() override{
        if (!RobotInitialized)
        {
            RobotInit(); 
            RobotInitialized=1;
        }

        ReadGyro();

        UpdateDriverScreen();

		ArmManual = 1;

        if(rot_stick.GetTrigger()){ //Cal Swerve Zeros write
            frc::Preferences::SetDouble("FLZero",FLZero);
            frc::Preferences::SetDouble("FRZero",FRZero);
            frc::Preferences::SetDouble("RLZero",RLZero);
            frc::Preferences::SetDouble("RRZero",RRZero);            
        }

        FLSteer.Set(ControlMode::Current,0.0);
        FRSteer.Set(ControlMode::Current,0.0);
        RLSteer.Set(ControlMode::Current,0.0);
        RRSteer.Set(ControlMode::Current,0.0);

        FLZero = fmod(FLSteer.GetSensorCollection().GetPulseWidthPosition(),4096);
        FRZero = fmod(FRSteer.GetSensorCollection().GetPulseWidthPosition(),4096);
        RLZero = fmod(RLSteer.GetSensorCollection().GetPulseWidthPosition(),4096);
        RRZero = fmod(RRSteer.GetSensorCollection().GetPulseWidthPosition(),4096);

		//Read encoder counts and then divide to get degrees of rotation
    	ActDir[FL]=fmod((FLSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0); //GetPulseWidthPosition()
		ActDir[FR]=fmod(((FRSteer.GetSensorCollection().GetPulseWidthPosition()-FRZero)/11.38),360.0);
    	ActDir[RL]=fmod((RLSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0);
    	ActDir[RR]=fmod((RRSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0);

		WristTarget=0;
		ShoulderTarget=0;

		#ifdef XBOX_CONTROLLER
		//homing routine
		if(OpController.GetBackButton()){
			//hold to home wrist
			Wrist.GetSensorCollection().SetQuadraturePosition(0,TIMEOUT);
		}else if(OpController.GetStartButton()){
			//hold to home Shoulder
			Shoulder.GetSensorCollection().SetIntegratedSensorPosition(0);
		}
		

		Wrist.Set(ControlMode::PercentOutput, OpController.GetLeftY()*0.5);
		Shoulder.Set(ControlMode::PercentOutput, -OpController.GetRightY()*0.5);

		#else
		if(OpController.GetShareButton()){
			//hold to home wrist
			Wrist.GetSensorCollection().SetQuadraturePosition(0,TIMEOUT);
		}else if(OpController.GetPSButton()){
			//hold to home Shoulder
			Shoulder.GetSensorCollection().SetIntegratedSensorPosition(0);
		}
		

		Wrist.Set(ControlMode::PercentOutput, OpController.GetLeftY());
		Shoulder.Set(ControlMode::PercentOutput, -OpController.GetRightY());

		#endif
    }

	#define SHOULDER_POSE 0
	#define WRIST_POSE 1

	#define MAX_SHOULDER 500000
	#define MIN_SHOULDER 5000
	#define MAX_WRIST 6000
	#define MIN_WRIST 200 
	

	
	long ArmPoses[12][4] = {
	//	cone Sholder Position	cone Wrist Position, cube shoulder, cube wrist
		{20000,MIN_WRIST,20000,MIN_WRIST}, //TRAVEL_POS		
		// {130400,5084,129792,5046},// HP_PICKUP Sideways
		{152783, 5766, 129792,5046},// HP_PICKUP
		{127531, 5084,124500,5046},// HP_PICK_DROP 
		{30700, 3586,50600, 4120},// FLOOR_PICKUP
		{124620,4931,114150,4019},// MID_SCORE
		{271858, 2223,112180,3367},// TOP_SCORE		
		{68500, 4955,68500, 4955},//CONE_VERTICAL
		{54265,2129,54265,2129},//SHUTE_PICKUP
		//Auto
		{16954,3235,38127,4214}, //FLOOR_PICKUP
		{250669,2214,112180,3367}, //TOP_SCORE
	};

	long ShoulderTarget = 0;
	long WristTarget =0;
	int HomeShoulder=0;
	int HomeWrist=0;

	#ifdef  XBOX_CONTROLLER
	void OperatorControl(){
		//Button Mappings
		
		//cone cube
		if(OpController.GetLeftTriggerAxis()>0.5){
			ObjectType = CUBE;
			LedIn1.Set(0);
			LedIn2.Set(1);
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.25);
		}
		else if(OpController.GetRightTriggerAxis()>0.5) {
			ObjectType = CONE;
			LedIn1.Set(1);
			LedIn2.Set(0);
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
		}else{
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
		}

		 		
		
		//Arm Positions
		if(OpController.GetXButton()) SelectedPosition = HP_PICKUP;
		else if(OpController.GetAButton()) SelectedPosition = FLOOR_PICKUP;
		else if(OpController.GetBButton()) SelectedPosition = MID_SCORE;
		else if(OpController.GetYButton()) SelectedPosition=TOP_SCORE;
		else if(OpController.GetRightBumper()) SelectedPosition=TRAVEL_POS;
		
		
		//homing rouetine
		if(OpController.GetBackButtonPressed()){
			//hold to home wrist
			HomeWrist=1;
		}else if(OpController.GetBackButtonReleased()){
            HomeWrist=2;
		} 

		
		if(OpController.GetStartButtonPressed() && !HomeShoulder){
			//hold to home Shoulder
			HomeShoulder = 1;
		}else if(OpController.GetStartButtonReleased()) HomeShoulder =0;

		//Manual Mode
		double StickL = OpController.GetLeftY();
		double StickR = -OpController.GetRightY(); 
		if(fabs(StickL)>0.15){
				WristTarget =Wrist.GetSensorCollection().GetQuadraturePosition()+ StickL*500;
				SelectedPosition=-1;
	    }
		if(fabs(StickR)>0.15){
				ShoulderTarget =Shoulder.GetSensorCollection().GetIntegratedSensorPosition() + StickR*16000;
				SelectedPosition=-1;
		}	
		
         
	}
	#else
		void OperatorControl(){
		//Button Mappings
		
		//cone cube
		if(OpController.GetRawAxis(2)>0.5){
			ObjectType = CUBE;
			LedIn1.Set(0);
			LedIn2.Set(1);
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.25);
		}
		else if(OpController.GetRawAxis(3)>0.5) {
			ObjectType = CONE;
			LedIn1.Set(1);
			LedIn2.Set(0);
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
		}else{
			OpController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,0.0);
		}

		 		
		
		//Arm Positions
		if(OpController.GetRawButton(1)) SelectedPosition = FLOOR_PICKUP;
		else if(OpController.GetRawButton(2)) SelectedPosition = MID_SCORE;//FLOOR_PICKUP;
		else if(OpController.GetRawButton(3)) SelectedPosition = HP_PICKUP;//MID_SCORE;
		// else if(OpController.GetRawButtonReleased(3)) SelectedPosition = HP_PICK_DROP;
		else if(OpController.GetRawButton(4)) SelectedPosition=TOP_SCORE;
		else if(OpController.GetPOV()==90) SelectedPosition=CONE_VERTICAL;
		else if(OpController.GetPOV()==270) SelectedPosition=SHUTE_PICKUP;
		else if(OpController.GetR1Button()) SelectedPosition=TRAVEL_POS;
		
		
		//homing rouetine
		if(OpController.GetRawButtonPressed(7)){
			//hold to home wrist
			HomeWrist=1;
		}else if(OpController.GetRawButtonReleased(7)){
			
            HomeWrist=2;
		} 

		
		if(OpController.GetRawButtonPressed(8) && !HomeShoulder){
			//hold to home Shoulder
			HomeShoulder = 1;
		}else if(OpController.GetRawButtonReleased(8)) HomeShoulder =0;

		//Manual Mode
		double StickL = OpController.GetLeftY();
		double StickR = -OpController.GetRightY(); 
		if(fabs(StickL)>0.15){
				WristTarget = Wrist.GetSensorCollection().GetQuadraturePosition() + StickL*1000;
				SelectedPosition=-1;
	    }
		if(fabs(StickR)>0.15){
				ShoulderTarget =Shoulder.GetSensorCollection().GetIntegratedSensorPosition() + StickR*16000;
				SelectedPosition=-1;
		}	
		
         
	}	
	#endif
	

	void RunWrist(){

		if(SelectedPosition>=0) WristTarget =  ArmPoses[SelectedPosition][ObjectType+WRIST_POSE];
		//Wrist limits //deemed unneccessary because all values are from array
		// if(WristTarget>MAX_WRIST){
		// 	WristTarget=MAX_WRIST;
		// }
		// if(WristTarget<MIN_WRIST){
		// 	WristTarget=MIN_WRIST;
		// }


		if(ObjectType==CONE &&  //prevents hight limit violation //`fix rist not moving at top_score
		(((SelectedPosition==TOP_SCORE || SelectedPosition==AUTO_TOP) && Shoulder.GetSensorCollection().GetIntegratedSensorPosition() < 225000) ||
		 (SelectedPosition!=TOP_SCORE && SelectedPosition!=AUTO_TOP && SelectedPosition!=-1 && Shoulder.GetSensorCollection().GetIntegratedSensorPosition() > ArmPoses[HP_PICKUP][SHOULDER_POSE])))
		{
			// WristTarget = ArmPoses[TRAVEL_POS][WRIST_POSE];
			if(Wrist.GetSensorCollection().GetQuadraturePosition() > 2500) WristTarget = MAX_WRIST;
			else WristTarget = ArmPoses[TRAVEL_POS][WRIST_POSE];
		}
		
		if(HomeWrist==1){
			Wrist.Set(ControlMode::PercentOutput,-0.2);
		}else if(HomeWrist==2){
	        Wrist.GetSensorCollection().SetQuadraturePosition(0);
			WristTarget=MIN_WRIST;
			Wrist.ConfigPeakCurrentLimit(3,TIMEOUT);
			HomeWrist=0;
		}else{
			Wrist.Set(ControlMode::Position,WristTarget);
			Wrist.ConfigPeakCurrentLimit(60,TIMEOUT);
		} 
	}

	void RunShoulder(){
		if(SelectedPosition>=0) ShoulderTarget = ArmPoses[SelectedPosition][ObjectType+SHOULDER_POSE];
		

		//Shoulder Limits
		// if(ShoulderTarget>MAX_SHOULDER){
		// 	ShoulderTarget=MAX_SHOULDER;
		// }
		// if(ShoulderTarget<MIN_SHOULDER){
		// 	ShoulderTarget=MIN_SHOULDER;
		// }

		//Position Motion Magic Configs
		// if(SelectedPosition==TRAVEL_POS){
		// 	Shoulder.ConfigMotionCruiseVelocity(100000);
		// 	Shoulder.ConfigMotionAcceleration(10000);			
		// }else{
		// 	Shoulder.ConfigMotionCruiseVelocity(500000);
		// 	Shoulder.ConfigMotionAcceleration(40000);
		// }

		if(HomeShoulder==1){
			Shoulder.Set(ControlMode::PercentOutput,-0.2);
			if(Shoulder.GetOutputCurrent()>=9.0){
				Shoulder.GetSensorCollection().SetIntegratedSensorPosition(0);
				ShoulderTarget=MIN_SHOULDER;
				HomeShoulder=2;
			}
		}else Shoulder.Set(ControlMode::MotionMagic,ShoulderTarget);
	}

	int InLast = 0;
	void RunIntake(){
		double intakePercent = 0.0;
		// if(!IsAutonomous()){
			if(OpController.GetPOV()==0 || dir_stick.GetTrigger() || (IsAutonomous() && (AutoIntake == INTAKE_IN))){ //intake
				intakePercent=1.0;
				InLast=1;
			}
			else if(OpController.GetPOV()==180 || dir_stick.GetRawButton(2) || (IsAutonomous() && (AutoIntake == INTAKE_EJECT))){//outake
				intakePercent = -0.75;
				InLast=0;
			}
			else
			{
				if(SelectedPosition == TRAVEL_POS 
				&& ObjectType == CONE
				&& (fabs(WristTarget-Wrist.GetSensorCollection().GetPulseWidthPosition())>100) //Wrist position is close enough
				&& (fabs(ShoulderTarget-Shoulder.GetSensorCollection().GetIntegratedSensorPosition())>2000) //shoulder position
				)intakePercent = 0.75;

				else if(InLast==1 || (IsAutonomous() && AutoIntake==INTAKE_HOLD)) intakePercent = 0.1; //0.2
				else intakePercent=0.0;
				
			}
		// // }else{ //Autonomous
		// 	if(AutoIntake==INTAKE_IN){ //intake
		// 		Intake.Set(ControlMode::PercentOutput, 1.0);
		// 		InLast=1;
		// 	}
		// 	else if(AutoIntake == INTAKE_EJECT){//outake
		// 		Intake.Set(ControlMode::PercentOutput, -0.5);
		// 		InLast=0;
		// 	}
		// 	else if(AutoIntake==INTAKE_HOLD)
		// 	{
		// 		Intake.Set(ControlMode::PercentOutput, 0.1); //0.2	
		// 	}else{
		// 		Intake.Set(ControlMode::PercentOutput,0.0);
		// 	}
		// }

		Intake.Set(ControlMode::PercentOutput, intakePercent);
	}

	void UpdateDriverScreen(){
        if(!UpdateCount) {
			UpdateCount=25; //delay between displays to not bog down system
			
			int i;
	  char str[40];
/*
			sprintf(str, "Rot:%f %f",FLSTEER,rot_stick.GetX());
		    frc::SmartDashboard::PutString("DB/String 0", str);
*/
			//sprintf(str, "FR:%d %d",FRSteer.GetSensorCollection().GetPulseWidthPosition(),FRSteer.GetSensorCollection().GetAnalogInRaw());
		    //frc::SmartDashboard::PutString("DB/String 0", str);
			//sprintf(str, "xFL%d,FR%d,RL%d,RR%d",(int)RobotX[FL],(int)RobotX[FR],(int)RobotX[RL],(int)RobotX[RR]);
			//frc::SmartDashboard::PutString("DB/String 1", str);
			sprintf(str, "tFL%d,FR%d,RL%d,RR%d",(int)TargetDir[FL],(int)TargetDir[FR],(int)TargetDir[RL],(int)TargetDir[RR]);
			frc::SmartDashboard::PutString("DB/String 0", str);
			sprintf(str, "aFL%d,FR%d,RL%d,RR%d",(int)ActDir[FL],(int)ActDir[FR],(int)ActDir[RL],(int)ActDir[RR]);
			frc::SmartDashboard::PutString("DB/String 1", str);
 /*
            sprintf(str, "delta:%4.2f",delta);
		    frc::SmartDashboard::PutString("DB/String 0", str);
 */  
			//sprintf(str, "Mod:%4.2f,%4.2f",ActDir[FR],TargetDir[FR]);
		    sprintf(str,"Shld a%6.0f,t%d",Shoulder.GetSensorCollection().GetIntegratedSensorPosition(),ShoulderTarget);
			frc::SmartDashboard::PutString("DB/String 2", str);
			//sprintf(str, "Tgt:%4.2f,%4.2f",TargetDir[RR],TargetDir[FL]);
		    sprintf(str,"Wrst a%d,t%d",Wrist.GetSensorCollection().GetQuadraturePosition(),WristTarget);
			frc::SmartDashboard::PutString("DB/String 3", str);
			// sprintf(str, "gyro= %f",Gyro);
			sprintf(str, "ObjT %d,Pos %d,Gyro %4.2f",ObjectType,SelectedPosition,RobotAngle);
		    frc::SmartDashboard::PutString("DB/String 4", str);
			// sprintf(str, "StKR%f",OpController.GetRightY());
			sprintf(str,"FRC%4.2f",FRDrive.GetSelectedSensorPosition());
		    frc::SmartDashboard::PutString("DB/String 5", str);
			// sprintf(str, "StKL%f",OpController.GetLeftY());
			sprintf(str,"FRZ%4.2f",FRZero);
		    frc::SmartDashboard::PutString("DB/String 6", str);
			// sprintf(str,"Pitch:%4.2f",RobotPitch);
			// sprintf(str,"%s",frc::SmartDashboard::GetData("Auto Selector"));
			// sprintf(str,"V:%4.2f",FLDrive.GetSelectedSensorVelocity());
			sprintf(str,"Dir%4.2f,Dist%4.2f",tempPrint,tempPrint2);
			frc::SmartDashboard::PutString("DB/String 8",str);
			// sprintf(str,"ShCur:%4.2f",Shoulder.GetOutputCurrent());
			sprintf(str,"PosX%4.2f,Y%4.2f",RobotX[IDX],RobotY[IDX]);
			frc::SmartDashboard::PutString("DB/String 9",str);

			// double Kp = frc::SmartDashboard::GetNumber("DB/Slider 0",1.0);
			// double Ki = frc::SmartDashboard::GetNumber("DB/Slider 1",0.0);
			// double Kd = frc::SmartDashboard::GetNumber("DB/Slider 2",0.0);
			// double Kf = frc::SmartDashboard::GetNumber("DB/Slider 2",0.0);

			// Wrist.Config_kP(0,Kp,TIMEOUT);
			// Wrist.Config_kI(0,Ki,TIMEOUT);
			// Wrist.Config_kD(0,Kd,TIMEOUT);
			// Wrist.Config_kF(0,Kf,TIMEOUT);


		}else UpdateCount--;
	}


	void UpdateScreenAuto(int Command, int AutoLine,double AutoX, double AutoY, double MaxPower, int Orientation){
       char str[40];
	
					if(!UpdateCount) {
								UpdateCount=25; //delay between displays to not bog down system
								sprintf(str, "LINE:%d,CMD:%d",AutoLine,Command);
								frc::SmartDashboard::PutString("DB/String 0", str);
								sprintf(str, "TX%d TY%d",(int)AutoX,(int)AutoY);
								frc::SmartDashboard::PutString("DB/String 1", str);
								sprintf(str, "xFL%d,FR%d,RL%d,RR%d",(int)RobotX[FL],(int)RobotX[FR],(int)RobotX[RL],(int)RobotX[RR]);
								frc::SmartDashboard::PutString("DB/String 2", str);
								sprintf(str, "yFL%d,FR%d,RL%d,RR%d",(int)RobotY[FL],(int)RobotY[FR],(int)RobotY[RL],(int)RobotY[RR]);
								frc::SmartDashboard::PutString("DB/String 3", str);
								// sprintf(str, "Pwr:%4.2f",MaxPower);
								sprintf(str,"ST%ld",ShoulderTarget);
								frc::SmartDashboard::PutString("DB/String 4", str);
								sprintf(str, "Ang T%d,A%3.0f",Orientation,fmod(RobotAngle,360.0));
								frc::SmartDashboard::PutString("DB/String 5", str);
								sprintf(str, "A:%d,%d,%d,%d",(int)ActDir[FL],(int)ActDir[FR],(int)ActDir[RL],(int)ActDir[RR]);
								frc::SmartDashboard::PutString("DB/String 6", str);
								sprintf(str, "T:%d,%d,%d,%d",(int)TargetDir[FL],(int)TargetDir[FR],(int)TargetDir[RL],(int)TargetDir[RR]);
								frc::SmartDashboard::PutString("DB/String 7", str);
								// sprintf(str, "TMR: %f",TeleTime.Get());
								sprintf(str,"SWRVY %4.2f,SWRVX%4.2f",SWRVY,SWRVX);
								frc::SmartDashboard::PutString("DB/String 8", str);
								// sprintf(str, "Ptch:%4.2f,RH:%d",RobotPitch,RampHit);
								// sprintf(str,"IN:%d,A%d",AutoIntake,IsAutonomous());
								// sprintf(str,"DIR%4.2f",tempPrint);
								sprintf(str,"S%ld,W%ld",ArmPoses[SelectedPosition][ObjectType+SHOULDER_POSE],ArmPoses[SelectedPosition][ObjectType+WRIST_POSE]);
								frc::SmartDashboard::PutString("DB/String 9", str);


						}else UpdateCount--;
     }


    //This is were the SWRVX,SWRVY,SWRVZ values are set to direct the swerve drives where to go
	void SwerveControl(void){
		//AutoPilot?
	    if(IsAutonomousEnabled()){
			FieldCentric=1; //; //Auto opertes in field centric mode //0
		    SWRVY=AutoDriveY;
			SWRVX=AutoDriveX; 
		    SWRVZ=AutoDriveZ;
			if(SWRVZ>1.0)SWRVZ=1.0;
			if(SWRVZ<-1.0) SWRVZ=-1.0;
			if(SWRVY>1.0)SWRVY=1.0;
			if(SWRVY<-1.0) SWRVY=-1.0;
			if(SWRVX>1.0)SWRVX=1.0;
			if(SWRVX<-1.0) SWRVX=-1.0;
		}else{ //Teleop Mode
			SWRVY=-dir_stick.GetY();
			SWRVX=dir_stick.GetX();
			SWRVZ=rot_stick.GetX();

			if (fabs(SWRVY)<=0.05) SWRVY=0.0;
			if (fabs(SWRVX)<=0.05) SWRVX=0.0;
			if(rot_stick.GetTrigger()) FieldCentric=0;
			else FieldCentric=1;	  
		}

		if(rot_stick.GetRawButtonPressed(4)){ //auto face pickup location
			if(fmod(fabs(RobotAngle),360.0) > 90.0 && fmod(fabs(RobotAngle),360.0) < 270.0) AutoRotatePID.SetSetpoint(180.0);
			else if(fmod(fabs(RobotAngle),360.0) < 90.0 || fmod(fabs(RobotAngle),360.0) > 270.0) AutoRotatePID.SetSetpoint(0.0); //move to init
		}
		if(rot_stick.GetRawButton(4)) SWRVZ = AutoRotatePID.Calculate(fmod(RobotAngle,360))/180;
		//`button for 180
		

		//Button 6 on left joystick resets the gyro to 0 degrees
		if(dir_stick.GetRawButton(9))
		{
			ResetGyro();
		}
	
  }	// End SwerveControl

  //Do the math on each wheel to find the new angle and speed required
  void  Calc4WheelTurn(){

	  double A,B,C,D,R,temp;
	  int i;
	  R=sqrt(pow(SW_L,2)+ pow(SW_W,2)); //aprox 30

	  A=SWRVX-SWRVZ*(SW_L/R);
	  B=SWRVX+SWRVZ*(SW_L/R);
	  C=SWRVY-SWRVZ*(SW_W/R);
	  D=SWRVY+SWRVZ*(SW_W/R);

	  ModDir[FR] = atan2(B,C)*180/PI;
	  ModSpd[FR]=  sqrt(pow(B,2)+pow(C,2));

	  ModDir[FL] = atan2(B,D)*180/PI;
	  ModSpd[FL]=  sqrt(pow(B,2)+pow(D,2));

	  ModDir[RL] = atan2(A,D)*180/PI;
	  ModSpd[RL]=  sqrt(pow(A,2)+pow(D,2));

	  ModDir[RR] = atan2(A,C)*180/PI;
	  ModSpd[RR]=  sqrt(pow(A,2)+pow(C,2));

	  temp=0;
	  for(i=0;i<4;i++){
		  if(ModSpd[i]>temp) temp=ModSpd[i];
	  }
	  if(temp>1.0){
		  for(i=0;i<4;i++) ModSpd[i]/=temp;
	  }
 	 return;
  }

  //Modify the Target direction given the new the direction from Calc4WheelTurn() using shortest distance calculations
  void SetDirection(int channel) {
		int curdir;
 
		if(SpeedPolarity[channel] < 0.0){
			curdir=fmod((TargetDir[channel]+180.0),360.0);
		} else {
			curdir=fmod(TargetDir[channel],360.0);
		}
		
		delta=ModDir[channel]-curdir; //What is the distance to move?  How best to get there...

		delta=fmod(delta,360.0);
		//Is Fastest way across 0 boundary?
		if(delta<-180.0) {  //Don't turn more than 180 degrees
			delta+=360.0;
		}
		if(delta>180.0) {
			delta-=360.0;
		}
		if(delta<=90.0 && delta>=-90.0) { //direct move without direction change
			TargetDir[channel]+=delta;
		} else {
			if(delta>90.0) delta-=180.0;
			if(delta<-90.0) delta+=180.0;
			TargetDir[channel]-=delta;
			if(SpeedPolarity[channel]>0.0 ) SpeedPolarity[channel]=-1.0; //switch directions so we steer less
			else SpeedPolarity[channel]=1.0;
		}
	}	// End SetDirection

	float CVTSpeed(float speed) {
			if(speed>1.0) speed=1.0;
			if(speed<-1.0) speed=-1.0;
			return(-speed);
    }	// End CVTSpeed

	float Gyro;
	//controlling the direction and speed of the swerve drives
    void SwerveDrive(void) {
    	int i;
    	float temp;
      
    	//Read encoder counts and then divide to get degrees of rotation
    	ActDir[FL]=fmod(((FLSteer.GetSensorCollection().GetPulseWidthPosition()-FLZero)/11.38),360.0); //GetPulseWidthPosition()
		ActDir[FR]=fmod(((FRSteer.GetSensorCollection().GetPulseWidthPosition()-FRZero)/11.38),360.0);
    	ActDir[RL]=fmod(((RLSteer.GetSensorCollection().GetPulseWidthPosition()-RLZero)/11.38),360.0);
    	ActDir[RR]=fmod(((RRSteer.GetSensorCollection().GetPulseWidthPosition()-RRZero)/11.38),360.0);

		//If rotational power is low, just don't rotate
        if(IsAutonomous()){
               //SWRVZ=pow(3*SWRVZ,3);
		}else{

			if (fabs(SWRVZ)<0.05){ //rot_stick deadzone
				SWRVZ=0.0;
			}else{
				// if(rot_stick.GetRawButton(8)) { // button 8 for fast rotation
				/// 	SWRVZ=pow(2.5*SWRVZ,3);  // show off (or big, heavy frame)
				// } else {
				int sign = fabs(SWRVZ)/SWRVZ;
				SWRVZ = pow(SWRVZ,2)*sign;//sin(SWRVZ);
				if(dir_stick.GetRawButton(4)) SWRVZ *= 0.2; 
				
				// }

			}
	    }
		//deermine if we are field or robot centric and make the adjustment here if necessary    
	    if(FieldCentric) Gyro=RobotAngle*PI/180; // || IsAutonomous()
	    else Gyro=0;

		//modify the target robot direction from field centric to robot centric for wheel positions and then set them in calc4wheel
	    temp=SWRVY*cos(Gyro)+SWRVX*sin(Gyro);
	    SWRVX=-SWRVY*sin(Gyro)+SWRVX*cos(Gyro);
	    SWRVY=temp;

		if(!IsAutonomous()){
			SWRVX = SWRVX; //pow(SWRVX,3)
			SWRVY = SWRVY;//pow(SWRVY,3)

			if(dir_stick.GetRawButton(4)) SWRVX *= 0.2; 
			if(dir_stick.GetRawButton(4)) SWRVY *= 0.2; 

			// if (SWRVX > 0.1) {
			// 	SWRVX = (0.1 + pow(SWRVX,3)) /1.1;
			// } else if (SWRVX < -0.1) {
			// 	SWRVX = (-0.1 + pow(SWRVX,3)) /1.1;
			// }
			// if (SWRVY > 0.1) {
			// 	SWRVY = (0.1 + pow(SWRVY,3)) /1.1;
			// } else if (SWRVX < -0.1) {
			// 	SWRVY = (-0.1 + pow(SWRVY,3)) /1.1;
			// }

		}
		Calc4WheelTurn();

		for(i=0;i<4;i++) SetDirection(i);
		

		

		
        if(TeleStarted==0&&fabs(ModSpd[IDX])>.1){
		    TeleTime.Start();
			TeleTime.Reset();
			TeleStarted=1;
		}

        //send the drive motor speeds to the motor controllers
		// FLDrive.Set(ControlMode::PercentOutput,-CVTSpeed(SpeedPolarity[FL]*-ModSpd[FL]));
		FLDrive.Set(ControlMode::Velocity,1023*(-CVTSpeed(SpeedPolarity[FL]*-ModSpd[FL])));
		FRDrive.Set(ControlMode::Velocity,1023*(CVTSpeed(SpeedPolarity[FR]*-ModSpd[FR])));
		RLDrive.Set(ControlMode::Velocity,1023*(-CVTSpeed(SpeedPolarity[RL]*-ModSpd[RL])));
		RRDrive.Set(ControlMode::Velocity,1023*(CVTSpeed(SpeedPolarity[RR]*-ModSpd[RR])));
		FLSteer.Set(ControlMode::Position, -(TargetDir[FL]/360.0)*4096);
	    FRSteer.Set(ControlMode::Position, -(TargetDir[FR]/360.0)*4096);
	    RRSteer.Set(ControlMode::Position, -(TargetDir[RR]/360.0)*4096);
		RLSteer.Set(ControlMode::Position, -(TargetDir[RL]/360.0)*4096);
    }	// End SwerveDrive

	void ReadGyro(void) {
		RobotAngle = -GyroSensor->GetYaw();//-GyroOffset;
        RobotPitch = GyroSensor->GetPitch() - PitchOffset;
	}	// End ReadyGyro
	void ResetGyro(void) {
		GyroSensor->ZeroYaw();
		PitchOffset = GyroSensor->GetPitch();

	}	// End ResetGyro

    double NewPosition[4],OldPosition[4];
	//Track robot FL wheel X,Y position in inches Y is long side of field X is width
    
	double tempPrint = 0;
	double tempPrint2 = 0;
	//Track robot FL wheel X,Y position in inches Y is long side of field X is width
    void TrackRobot(void){
		int i;
        double Dist,Dir;
		for(i=0;i<4;i++){
	     	//Determine distance moved since last time in this routine
			NewPosition[i]=GetLocation(i);
			Dist=OldPosition[i]-NewPosition[i];
			OldPosition[i]=NewPosition[i];
			Dir=ActDir[i]+RobotAngle;
			Dir=fmod(Dir,360.0);
			if(i==IDX){
				tempPrint = Dir;
				tempPrint2 = NewPosition[IDX];
			}
			RobotX[i]-=Dist*sin((Dir*PI)/180.0); //` the result of sin((-100 * PI) / 180) appoximatly = 0.03. Potentially adding a small number to RobotX each time.	
			RobotY[i]-=Dist*cos((Dir*PI)/180.0);
		}
	}	// End TrackRobot

    double GetLocation(int i){
		double result;
		switch (i){ 
			case FL:
				result=FLDrive.GetSelectedSensorPosition();
				break;
			case FR:
				result=FRDrive.GetSelectedSensorPosition();
				break;
			case RL:
				result=RLDrive.GetSelectedSensorPosition();
				break;
			case RR:
				result=RRDrive.GetSelectedSensorPosition();
				break;
			default:
				result=0.0;
		}
		return(result/780);
	}



  	//AUTONOMOUS DRIVING STATE MACHINE
    void AutoStateMachine(void){
		int *Array, RemainingInches;
		int MaxFromDcel,MaxFromAcel;
		double MaxPower,X,Y,Z ,DeltaA,fctr;
	    int Command, AccSec, DecInches, Speed, AutoX, AutoY, Orientation;
        //point data to current autonomous command line
	    Array=AutoArray[AutoLine]; 
		Command=*Array;
		Orientation=*(Array+6);
		Speed=*(Array+3);
		AccSec=*(Array+1);
		DecInches=*(Array+2);
        AutoX=*(Array+4);
		AutoY=*(Array+5);
		AutoIntake=*(Array+7);
        
		//distance to target
		RemainingInches=sqrt(pow((RobotX[IDX]-AutoX),2)+pow((RobotY[IDX]-AutoY),2));
		//Max speed percentage based on deceleration value given 
		if(DecInches==0) MaxFromDcel=100;
		else MaxFromDcel=(pow(RemainingInches,2)*10000)/pow(DecInches,2);
		//Get msec since we started and divide by msec to full power for power limit from acceleration
		MaxFromAcel=(int)(AutoTime.Get()*100000)/AccSec;

		// MaxFromAcel=100;
		// MaxFromDcel=100;

        if(MaxFromDcel<MaxFromAcel){
			MaxPower=(MaxFromDcel*Speed)/100;
        }else{
			MaxPower=(MaxFromAcel*Speed)/100;
		}
		if(MaxPower>Speed)MaxPower=Speed;
		if(MaxPower<0)MaxPower=0;
		//Calculate the X,Y direction to target
		X=((double)AutoX-RobotX[IDX]);
		Y=((double)AutoY-RobotY[IDX]);
		//Calculate the delta in the robot orientation target
		if(Orientation>90 || Orientation<-90){
			if(RobotAngle<-90){
				DeltaA=(fmod(RobotAngle+360.0,360.0)-Orientation);
			}else DeltaA=(RobotAngle-Orientation);
		}else DeltaA=(RobotAngle-Orientation);
		
		Z=(DeltaA*-1.0)/360.0; //90
		if(Z>0.5)Z=0.5;
		if(Z<-0.5) Z=-0.5;

		if(AutoLine>NUMAUTOLINES) Command=STOP; //in case no one put in a stop command

		UpdateScreenAuto(Command,AutoLine,X,Y,MaxPower,Orientation);
		// tempPrint = RemainingInches;


		switch(Command){ //command mode
		    case START: 
			            RobotX[IDX]=(double)AutoX;
						RobotY[IDX]=(double)AutoY;
						ResetGyro();
						
					

						if(AutoTime.Get().value() > 2.5){
							AutoLine++;
						    FirstPass=1;
					        TeleStarted=0; //Trigger Timer to reset and run on power to wheels
						}
		      
 		            break;
			case BALANCE: 
						                        
						fctr=fabs(X)+fabs(Y);
						if(fctr==0.0) fctr=1.0;
						
						AutoDriveX=0.0; //(double)((RobotPitch/15)*20)/100.0;
						if(RobotPitch>12.0) RampHit=1;
						if(RampHit){
                            //AutoDriveY=(double)((RobotPitch/15.0)*10.0)/100.0;
							if(RobotPitch>15.0)RobotPitch=15.0;
							if(RobotPitch<-15.0)RobotPitch=-15.0;
							Y=pow((RobotPitch/15.0),3);
						    AutoDriveY=(double)((Y)*10.0)/100.0; //Pwr was 50
							
						} else{
							AutoDriveY=(double)((Y/(fctr))*MaxPower)/100.0;
						}
		    
					    AutoDriveZ=0.0; 
						
                        if(FirstPass){
							FirstPass=0;
							if(fabs(Y)>fabs(X)){
								Travel=Y;
								UseYTravel=1;
							}else{
                                Travel=X;
								UseYTravel=0;
							} 
							
						}
						

						if((((UseYTravel&&(Y*Travel<0))||((!UseYTravel)&&(X*Travel<0)))&&!RampHit)){
                           RemainingInches=0;
						   AutoLine++;
						   FirstPass=1;
						}
						
						if(RampHit){
							if(RobotPitch <1.0  && RobotPitch > -1.0 && RampHit){
                            if(AutoTime.Get().value() > 2.0){
						 	  AutoLine++;
							  AutoTime.Reset();
						 	  FirstPass=1;
						    }
						 }else{
							AutoTime.Reset();
						 }
						}
						 break;
			case MOVE:  //Determine direction and rotation values from autonomous command
			            fctr=fabs(X)+fabs(Y);
						if(fctr==0.0) fctr=1.0;
						
						//if(RemainingInches>0){
							AutoDriveX=(double)((X/(fctr))*MaxPower)/100.0;
							AutoDriveY=(double)((Y/(fctr))*MaxPower)/100.0;
		               // }else{
						//	AutoDriveX=0.0;
						//    AutoDriveY=0.0;
						//}
						//if(fabs(DeltaA)>0){
						    AutoDriveZ=Z; //(double)(pow(Z,3)/15);
						//}else{
						//	AutoDriveZ=0.0;
						//}
                        if(FirstPass){
							FirstPass=0;
							if(fabs(Y)>fabs(X)){
								Travel=Y;
								UseYTravel=1;
							}else{
                                Travel=X;
								UseYTravel=0;
							} 
							
						}
						

						if((UseYTravel&&(Y*Travel<0))||((!UseYTravel)&&(X*Travel<0))){
                           RemainingInches=0;
						}
						if(RemainingInches<(Speed/5)){//&&!FirstPass){ //} && fabs(DeltaA)<5){
						   AutoLine++;
						   AutoTime.Reset();
						   FirstPass=1;
						}
						break;
			case ARM_AUTO:
					SelectedPosition = AutoX;
					ObjectType = AutoY;
					if(fabs(ShoulderTarget-Shoulder.GetSensorCollection().GetIntegratedSensorPosition())<5000 
						&& fabs(WristTarget-Wrist.GetSensorCollection().GetQuadraturePosition())<250
						|| AutoTime.Get().value()*1000 > AccSec){//arm_pos is almost equal to arm_target
						AutoLine++;
						AutoTime.Reset();
						FirstPass=1;
					}
					break;
			case MOVE_TIMED:
				AutoDriveX=0.0;
				AutoDriveY=0.1;
				AutoDriveZ=0.0;
				AccSec=2000;
			case WAIT_AUTO:
				if(AutoTime.Get().value() * 1000 > AccSec){
						AutoLine++;
					    FirstPass=1;
						AutoTime.Reset();
				        TeleStarted=0; //Trigger Timer to reset and run on power to wheels
				}
				break;
			default:
			case STOP:
						AutoDriveX=0.0;
						AutoDriveY=0.0;
						AutoDriveZ=0.01;
						TeleTime.Stop();
						FirstPass=0;
						//Intake.Set(ControlMode::PercentOutput, 0);
					    //UpdateScreenAuto( Command, AutoLine, AutoX, AutoY, MaxPower, Orientation);	
						break;
			
		}

		
	

    }
	 

	


 private:
  frc::Joystick dir_stick{DIR};  //swerve direction
  frc::Joystick rot_stick{ROT};  //swerve rotation
  frc::Joystick arm_stick{ARM};  //arm functions

  #ifdef XBOX_CONTROLLER
  frc::XboxController OpController{OP_CONTROLLER};
  #else
  frc::PS4Controller OpController{OP_CONTROLLER};
  #endif


  TalonFX FLDrive = {FLD};
  TalonFX FRDrive = {FRD};
  TalonFX RLDrive = {RLD};
  TalonFX RRDrive = {RRD};
  TalonSRX FLSteer = {FLS};
  TalonSRX FRSteer = {FRS};
  TalonSRX RLSteer = {RLS};
  TalonSRX RRSteer = {RRS};

  TalonSRX Wrist = {WRIST};
  TalonSRX Intake = {INTAKE};
  TalonFX  Shoulder = {SHOULDER};

//   TalonSRX Intake = {INTAKE};
  frc::DigitalOutput LedIn1{8};
  frc::DigitalOutput LedIn2{9};
  //frc::ADXRS450_Gyro Gyro; //small gyro
  AHRS *GyroSensor; //expansion port gyro
  frc::Timer AutoTime; //seconds timer for auto states
  frc::Timer TeleTime;
  frc::Timer SwitchTimer;

  frc::SendableChooser<std::string> AutoChooser;
  std::string m_autoSelected;


  void RobotInit() {

		RobotInitialized++;
        GyroSensor = new AHRS(frc::SPI::Port::kMXP);

		//Steer Zeros
    	FLZero = frc::Preferences::GetDouble("FLZero",0);
    	FRZero = frc::Preferences::GetDouble("FRZero",0);
    	RLZero = frc::Preferences::GetDouble("RLZero",0);
    	RRZero = frc::Preferences::GetDouble("RRZero",0);

	    //FLSteer.GetSensorCollection().SetPulseWidthPosition(0,TIMEOUT);
	    FLSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		FLSteer.SetSensorPhase(true);
	    FLSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FLSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    FLSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FLSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FLSteer.SelectProfileSlot(0, 0);
		FLSteer.Config_kP(0, PVALUE, TIMEOUT);
		FLSteer.Config_kI(0, IVALUE, TIMEOUT);
		FLSteer.Config_kD(0, DVALUE, TIMEOUT);
		FLSteer.SetNeutralMode(NeutralMode::Coast);
		FLSteer.Set(ControlMode::Position, 0.0);
		FLSteer.ConfigClosedLoopPeakOutput(0,0.6,TIMEOUT);

		FLSteer.GetSensorCollection().SetPulseWidthPosition(FLSteer.GetSensorCollection().GetPulseWidthPosition()%4096 - frc::Preferences::GetDouble("FLZero",0), TIMEOUT);

        //FRSteer.GetSensorCollection().SetPulseWidthPosition(0,TIMEOUT);
        FRSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		FRSteer.SetSensorPhase(true);
	    FRSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FRSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    FRSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FRSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FRSteer.SelectProfileSlot(0, 0);
		FRSteer.Config_kP(0, PVALUE, TIMEOUT);
		FRSteer.Config_kI(0, IVALUE, TIMEOUT);
		FRSteer.Config_kD(0, DVALUE, TIMEOUT);
		FRSteer.SetNeutralMode(NeutralMode::Coast);
		FRSteer.Set(ControlMode::Position, 0.0);
		FRSteer.ConfigClosedLoopPeakOutput(0,0.6,TIMEOUT);

		FRSteer.GetSensorCollection().SetPulseWidthPosition(FRSteer.GetSensorCollection().GetPulseWidthPosition()%4096 - frc::Preferences::GetDouble("FRZero",0), TIMEOUT);
        
		RLSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		RLSteer.SetSensorPhase(true);
	    RLSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RLSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RLSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RLSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RLSteer.SelectProfileSlot(0, 0);
		RLSteer.Config_kP(0, PVALUE, TIMEOUT);
		RLSteer.Config_kI(0, IVALUE, TIMEOUT);
		RLSteer.Config_kD(0, DVALUE, TIMEOUT);
		RLSteer.SetNeutralMode(NeutralMode::Coast);
		RLSteer.Set(ControlMode::Position, 0.0);
		RLSteer.ConfigClosedLoopPeakOutput(0,0.6,TIMEOUT);

		RLSteer.GetSensorCollection().SetPulseWidthPosition(RLSteer.GetSensorCollection().GetPulseWidthPosition()%4096- frc::Preferences::GetDouble("RLZero",0), TIMEOUT);

 	    RRSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		RRSteer.SetSensorPhase(true);
	    RRSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RRSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RRSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RRSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RRSteer.SelectProfileSlot(0, 0);
		RRSteer.Config_kP(0, PVALUE, TIMEOUT);
		RRSteer.Config_kI(0, IVALUE, TIMEOUT);
		RRSteer.Config_kD(0, DVALUE, TIMEOUT);
		RRSteer.SetNeutralMode(NeutralMode::Coast);
		RRSteer.Set(ControlMode::Position, 0.0);
		RRSteer.ConfigClosedLoopPeakOutput(0,0.6,TIMEOUT);

		RRSteer.GetSensorCollection().SetPulseWidthPosition(RRSteer.GetSensorCollection().GetPulseWidthPosition()%4096 - frc::Preferences::GetDouble("RRZero",0), TIMEOUT);
		  
        RLDrive.GetSensorCollection().SetIntegratedSensorPosition(0);
        RLDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	 // PIDLoop=0 
		RLDrive.SetSensorPhase(false);
	    RLDrive.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RLDrive.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RLDrive.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RLDrive.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RLDrive.SelectProfileSlot(0, 0);
		RLDrive.Config_kP(0, 0.0, TIMEOUT);
		RLDrive.Config_kI(0, 0.0, TIMEOUT);
		RLDrive.Config_kD(0, 0.0, TIMEOUT);
		RLDrive.Config_kF(0, 1.0, TIMEOUT);
		RLDrive.SetNeutralMode(NeutralMode::Coast);
		RLDrive.Set(ControlMode::PercentOutput, 0.0);

		RRDrive.GetSensorCollection().SetIntegratedSensorPosition(0);
        RRDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	 
		RRDrive.SetSensorPhase(false);
	    RRDrive.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RRDrive.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RRDrive.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RRDrive.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RRDrive.SelectProfileSlot(0, 0);
		RRDrive.Config_kP(0, 0.0, TIMEOUT);
		RRDrive.Config_kI(0, 0.0, TIMEOUT);
		RRDrive.Config_kD(0, 0.0, TIMEOUT);
		RRDrive.Config_kF(0, 1.0, TIMEOUT);
		RRDrive.SetNeutralMode(NeutralMode::Coast);
		RRDrive.Set(ControlMode::PercentOutput, 0.0);

        FRDrive.GetSensorCollection().SetIntegratedSensorPosition(0);
        FRDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	 
		FRDrive.SetSensorPhase(false);
	    FRDrive.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FRDrive.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    FRDrive.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FRDrive.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FRDrive.SelectProfileSlot(0, 0);
		FRDrive.Config_kP(0, 0.0, TIMEOUT);
		FRDrive.Config_kI(0, 0.0, TIMEOUT);
		FRDrive.Config_kD(0, 0.0, TIMEOUT);
		FRDrive.Config_kF(0, 1.0, TIMEOUT);
		FRDrive.SetNeutralMode(NeutralMode::Coast);
		FRDrive.Set(ControlMode::PercentOutput, 0.0);

		FLDrive.GetSensorCollection().SetIntegratedSensorPosition(0);
        FLDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	
		FLDrive.SetSensorPhase(false);
	    FLDrive.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FLDrive.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    FLDrive.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FLDrive.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FLDrive.SelectProfileSlot(0, 0);
		FLDrive.Config_kP(0, 0.0, TIMEOUT); //0.5
		FLDrive.Config_kI(0, 0.0, TIMEOUT);
		FLDrive.Config_kD(0, 0.0, TIMEOUT);
		FLDrive.Config_kF(0, 1.0, TIMEOUT);
		FLDrive.SetNeutralMode(NeutralMode::Coast);
		FLDrive.Set(ControlMode::PercentOutput, 0.0);

		Shoulder.GetSensorCollection().SetIntegratedSensorPosition(0);
        Shoulder.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	
		Shoulder.SetSensorPhase(false);
	    Shoulder.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Shoulder.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Shoulder.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Shoulder.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		// Shoulder.ConfigSupplyCurrentLimit(motorcontrol::SupplyCurrentLimitConfiguration{})
		Shoulder.SelectProfileSlot(0, 0);
		Shoulder.Config_kP(0, 0.4, TIMEOUT);
		Shoulder.Config_kI(0, 0.0, TIMEOUT);
		Shoulder.Config_kD(0, 40.0, TIMEOUT); //100
		Shoulder.Config_kF(0, 0.0,TIMEOUT);  //0.3
		Shoulder.ConfigSupplyCurrentLimit(motorcontrol::SupplyCurrentLimitConfiguration{true,6,6,.2},TIMEOUT);
		Shoulder.SetNeutralMode(NeutralMode::Brake);
		Shoulder.Set(ControlMode::PercentOutput, 0.0);
		Shoulder.ConfigClosedloopRamp(0.25,TIMEOUT);

		// Set relevant frame periods to be at least as fast as periodic rate 
		Shoulder.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, TIMEOUT);
		Shoulder.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, TIMEOUT);

		// Set acceleration and vcruise velocity - see documentation 
		Shoulder.ConfigMotionCruiseVelocity(250000, TIMEOUT); //50000
		Shoulder.ConfigMotionAcceleration(20000, TIMEOUT); //40000
		Shoulder.ConfigMotionSCurveStrength(0,TIMEOUT);

		Wrist.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, NOTIMEOUT);	 
		Wrist.SetSensorPhase(false);
	    Wrist.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Wrist.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Wrist.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Wrist.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Wrist.SelectProfileSlot(0, 0);
		Wrist.Config_kP(0, 1.0, TIMEOUT); //1.5//.85
		Wrist.Config_kI(0, 0.0, TIMEOUT);
		Wrist.Config_kD(0, 0.2, TIMEOUT);
		Wrist.SetNeutralMode(NeutralMode::Brake); //`Coast
		// Wrist.ConfigPeakCurrentLimit(1,TIMEOUT);
		Wrist.Set(ControlMode::PercentOutput, 0.0);
		Wrist.EnableCurrentLimit(true);
		Wrist.ConfigPeakCurrentLimit(50,TIMEOUT);
		// Wrist.SetSelectedSensorPosition(fmod(Wrist.GetSelectedSensorPosition(),2048));
		Wrist.GetSensorCollection().SetQuadraturePosition(0,TIMEOUT);
	    
		  

		 Intake.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);
         Intake.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	
		 Intake.SetSensorPhase(false);
	     Intake.ConfigNominalOutputForward(0.0f, TIMEOUT);
		 Intake.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	     Intake.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	 Intake.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		 Intake.SelectProfileSlot(0, 0);
		 Intake.Config_kP(0, 0.5, TIMEOUT);
		 Intake.Config_kI(0, 0.0, TIMEOUT);
		 Intake.Config_kD(0, 0.0, TIMEOUT);
		 Intake.ConfigContinuousCurrentLimit(20,TIMEOUT);
		 Intake.EnableCurrentLimit(true);
		 Intake.Set(ControlMode::PercentOutput, 0.0);

		
		ResetGyro();

		AutoChooser.SetDefaultOption(NO_MOVE_SELECTIONSTRING,NO_MOVE_SELECTIONSTRING);
		AutoChooser.AddOption(BALANCE_SELECTION_STRING,BALANCE_SELECTION_STRING);
  		AutoChooser.AddOption(VL_SELECTION_STRING,VL_SELECTION_STRING);
		AutoChooser.AddOption(MOVE_TEST_SELECTION_STRING,MOVE_TEST_SELECTION_STRING);
  		frc::SmartDashboard::PutData("Auto Selector", &AutoChooser);

		#ifdef CAMERA
		frc::CameraServer::StartAutomaticCapture();
		//std::thread visionThread(VisionThread);
		//visionThread.detach();
		#endif
  }

  #ifdef CAMERA 
  static void VisionThread() {
		//Camera
		
		cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
		// frc::CameraServer::RemoveCamera("USB Camera 0");
		camera.SetResolution(1024,768);
		cs::CvSink cvSink = frc::CameraServer::GetVideo();
		cs::CvSource outputStream = frc::CameraServer::PutVideo("Flipped", 1024,768);
		cv::Mat src;
		cv::Mat dst;
		cv::flip(src, dst, 0);

		
		
		while (true) {
			if (cvSink.GrabFrame(src) == 0) {
				outputStream.NotifyError(cvSink.GetError());
				continue;
			}

		cv::flip(src, dst, 0);

		// rectangle(dst, cv::Point(100,100), cv::Point(400,400), cv::Scalar(255,255,255), 5);
		outputStream.PutFrame(dst);
			
		}


		//frc::CameraServer::StartAutomaticCapture(1);
				
		

	}
	#endif

	// void RobotInit() override {
	// 	std::thread visionThread(VisionThread);
	// 	visionThread.detach();
	// }

};
#ifndef RUNNING_FRC_TESTS
int main() { 
	
	return frc::StartRobot<Robot>(); 

}
#endif
