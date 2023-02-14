/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PS4Controller.h>
#include <frc/TimedRobot.h>
#include <frc/DriverStation.h>
#include <frc/Preferences.h>

#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "AHRS.h"
#define PVALUE 2.00
#define IVALUE 0.000
#define DVALUE 500000.00

#define START 10
#define STOP  11
#define MOVE  12
#define SWITCH 14
#define ADJUST 13

#define NUMAUTOLINES 30
int (*AutoArray)[8];

#define IDXX 7 /* x offset from 0,0 for index wheel FL=7, FR=17, RL=7, RR=17 */
#define IDXY -17 /*y offset from 0,0 for index wheel FL=-7, FR=-7, RL=-17, RR=-17 */ 


// autonomous barrel race  RED 1
#define BRPW 70
int BR_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,      0,       0,        0,            0}, //Start at midfield location
			{MOVE,    750,         5,   BRPW,     -10,      50,        0,            0},	//up to speed
			{MOVE,       5,         5,   BRPW,     5,     101,        0,            0},	//cone 1
			{MOVE,       5,         5,   BRPW,     30,     121,        0,            0},	//
			{MOVE,       5,         5,   BRPW,     30,      75,        0,            0},	//
			{MOVE,       5,         5,   BRPW,     5,      75,        0,            0},	//
			{MOVE,       5,         5,   BRPW,     -15,     185,        0,            0},	//cone 2
			{MOVE,       5,         5,   BRPW,      -45,     185,        0,            0}, 	//
			{MOVE,       5,         5,   BRPW,      -45,     130,        0,            0}, 	//
			{MOVE,       5,         5,   BRPW,     40,     196,        0,            0}, 	//cone 3
		    {MOVE,       5,         5,   BRPW,     40,     240,        0,            0},	//
		    {MOVE,       5,         5,   BRPW,     -5,     240,        0,            0},
			{MOVE,       5,         5,    100,     -25,     120,        0,            0},	//
			{MOVE,       5,        60,    100,     0,       0,        0,            0},	//return to base
			{MOVE,       5,        60,     30,     0,      20,        0,            0},	//descelerate
			{STOP,       0,         0,      0,      0,       0,        0,            0},	//STOP
};

// slalom  RED 2
#define SLPW 65
int	SL_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,     160,      40,        0,            0},	//Start at right location
			{MOVE,    500,         5,   SLPW,     110,     90,        0,            0},	//
			{MOVE,       5,         5,   100,     90,     260,        0,            0},	//pass 1
			{MOVE,     5,         5,     65,     125,     285,        0,            0},	//500
			{MOVE,		 5,			5,	   65,	   105,		320,		0,			  0},	//pass 3
			{MOVE,       5,         5,     65,      75,     290,        0,            0},	//
			{MOVE,       5,         5,     65,     120,     250,        0,            0},	//pass 4
			{MOVE,       5,         5,   100,     120,      80,        0,            0},	//
			{MOVE,       5,         5,     65,      95,      30,        0,            0},	//pass 5
			{STOP,       0,         0,      0,       0,       0,        0,            0},	//STOP
};

// Bounce Path   RED 3 line up to right of zone with left front corner at middle
#define BNPW 75
int	BN_AutoArray[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,    IDXX,   IDXY,        0,            0},	//Start at midfield location
			{MOVE,    500,         5,   BNPW,     -23,     22,        0,            0},	//
			{MOVE,       5,        10,     50,     -50,     25,        0,            0},	//pin 1
			{ADJUST,     0,         0,      0,     -10,      0,        0,            0},	//spin 10"@45 15"@50
			{MOVE,    500,         5,   BNPW,      15,     70,        0,            0},	//
			{MOVE,       5,         5,   BNPW,      40,     94,        0,            0},	//turn 1
			{MOVE,       5,         5,   BNPW,      35,    100,        0,            0},	//
			{MOVE,       5,         5,  55,     -20,    130,        0,            0},	//
			{MOVE,       5,         5,   40,     -50,    125,        0,            0},	//
			{ADJUST,     0,         0,      0,     0,      0,        0,            0},	//spin 10"@45 15"@50
			{MOVE,    500,        5,   BNPW,      20,    120,        0,            0},	//pin 2
			{MOVE,       5,         5,   BNPW,      35,    195,        0,            0},	//
			{MOVE,       5,        5,   BNPW,     -30,    200,        0,            0},	//pin 3
		    {MOVE,       5,        5,   40,     -60,    200,        0,            0},	//pin 3
			//{ADJUST,     0,         0,      0,     -10,      0,        0,            0},	//spin 10"@45 15"@50
			{MOVE,    500,         5,   BNPW,     -20,    190,        0,            0},	//
			{MOVE,       5,         5,   BNPW,     -20,    220,        0,            0},	//finish
			{STOP,       0,         0,      0,       0,      0,        0,            0},	//STOP
};

// Galactic Search Path(s)   BLUE 1
#define GSPW 45
int	GS_AutoArray_Initial[NUMAUTOLINES][8]={
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,TargetX, TargetY, Orientation Deg,IntakeState
			{START,      0,         0,      0,    20,      10,        0,            -1},	//Start at midfield location
			{MOVE,    1500,         5,   GSPW,     20,     40,        45,            -1},	//
			{MOVE,       5,         5,   GSPW,     30,     80,        85,           1},	//
			{SWITCH,     6,         0,      0,     0,      0,        0,            0},	//Detect path A or B and switch AutoArray
			//-----------------------------Ball 1 Red B--------------------------
			{MOVE,    5,         5,   GSPW,     100,     140,        45,            1},
			{MOVE,    5,         5,   GSPW,     35,     220,        -45,            1}, 
			{MOVE,    5,         5,   GSPW,     40,     295,        -90,            1},
			{MOVE,    5,         5,   GSPW,     50,     285,        -90,            0}, 
			{STOP,       0,         0,      0,       0,      0,        0,            0}, //STOP
			//----------------------------Ball 1 Red A--------------------------
			{MOVE,    5,         5,   GSPW,     70,     75,        90,            1}, 
			{SWITCH,     6,         0,      0,     0,      0,        0,            0},
			{MOVE,    500,         5,   GSPW,     125,    175,        45,            1},
			{MOVE,    500,         5,   GSPW,     20,     160,       -30,            1}, 
			{MOVE,    5,         5,   GSPW,     30,     250,        -45,            1},
			{MOVE,    5,         5,   GSPW,     50,     330,        -90,            0}, 
			{STOP,       0,         0,      0,       0,      0,        0,            0},	//STOP
			//--------------------------Blue A + B------------------------------
			{MOVE,    1500,         5,   GSPW,     130,     130,        0,            1},
			{MOVE,    1500,         5,   GSPW,     138,     165,        -30,            1},
			{MOVE,    1500,         5,   GSPW,     110,     175,        -90,            1},
			{MOVE,    5,         5,   GSPW,     49,     170,        -30,            1},
			{MOVE,    5,         5,   GSPW,     45,     225,        0,            1},
			{MOVE,    5,         5,   GSPW,     94,     300,        45,            1},
			{MOVE,    5,         5,   GSPW,     125,     345,        90,            1},
			{STOP,       0,         0,      0,       0,      0,        0,            0},
};
//Motor controller defines
#define TIMEOUT 10
#define NOTIMEOUT 0
//Joystick defines
#define DIR 0
#define ROT 1
#define ARM 2

//PS4 Controller defines
#define OP_PS4 2

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

#define WRIST 9
#define SHOULDER 1
#define INTAKE 8


//For swerve code, define motor data locations in the arrays
#define RR 0
#define FR 1
#define FL 2
#define RL 3
#define ALL -1 /* all drive channels */
#define IDX  3 /* use Rear Left Wheel as index */
#define PI 3.141592654
#define SW_L 21.75   //Distance between wheels from front to back
#define SW_W 21.75   //Distance between wheels from side to side

int TeleStarted=1;
double Travel;
int FirstPass=1,UseYTravel;
//Realtime values of robot status
int FieldCentric=0;   //1= feild centric driving
float RobotAngle=0.0; //angle from Gyro
int IntakeState=0;
int UpdateCount=0;    //counter to slow screen update so it does not take too many CPU cycles


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


//Flag so we don't reinit Gyros etc when switching from Auto to Telop
int RobotInitialized=0;
int AutoArraySet=0;
int StartingLocation=0;
#define RED1  (frc::DriverStation::kRed*10+1)
#define RED2  (frc::DriverStation::kRed*10+2)
#define RED3  (frc::DriverStation::kRed*10+3)
#define BLUE1 (frc::DriverStation::kBlue*10+1)
#define BLUE2 (frc::DriverStation::kBlue*10+2)
#define BLUE3 (frc::DriverStation::kBlue*10+3)
#define UNKNOWN 0

//Arm Positions
#define ARM_DEFAULT 0
#define HP_PICKUP 1
#define FLOOR_PICKUP 2
#define MID_SCORE 3 
#define TOP_SCORE 4
#define TRAVEL_POS 5
int ArmPosition = ARM_DEFAULT;
#define CONE 1
#define CUBE 0
int ObjectType = 0;

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
			StartingLocation = RED1;//`frc::DriverStation::GetInstance().GetAlliance()*10 + frc::DriverStation::GetInstance().GetLocation();
			switch (StartingLocation){ 
				case RED1:
					AutoArray=BR_AutoArray;			//Red 1 = Barrel Race game
					break;
				case RED2:
					AutoArray=SL_AutoArray;			//Red 2 = Slalom game
					break;
				case RED3:
					AutoArray=BN_AutoArray;			//Red 3 = Bounce game
					break;
				case BLUE1:
				default:
					AutoArray=GS_AutoArray_Initial;	//Blue 1 = Galactic Search game--need logic to determine path A or B
					break;
			}
	   }
	   AutoTime.Start();
	
		ReadGyro();              //get the angle of the robot in to variable "RobotAngle"
		TrackRobot();			//Track the Robot's FL wheel X,Y position around the field
		AutoStateMachine();      //Get updated directional commands from autopilot
		if(!FirstPass) {		// don't try to control anything when state changes
			SwerveControl();         //convert auto pilot commands to joystick motions
			SwerveDrive();           //Update the speed and direction of the swerve drive motors
		}

		UpdateDriverScreen();
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
		//TrackRobot();			//Track the Robot's FL wheel X,Y position around the field
	    SwerveControl(); 
		SwerveDrive();

		UpdateDriverScreen();

		JoystickControl(); //` add way to switch between

  }

	void TestPeriodic() override{
        if (!RobotInitialized)
        {
            RobotInit(); 
            RobotInitialized=1;
        }

        ReadGyro();

        UpdateDriverScreen();

		

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

        FLZero = FLSteer.GetSensorCollection().GetPulseWidthPosition();
        FRZero = FRSteer.GetSensorCollection().GetPulseWidthPosition();
        RLZero = RLSteer.GetSensorCollection().GetPulseWidthPosition();
        RRZero = RRSteer.GetSensorCollection().GetPulseWidthPosition();


    }

	void JoystickControl(){
		//Intake.Set(ControlMode::PercentOutput, arm_stick.GetY());
		if(arm_stick.GetRawButton(2)){ //outake
			Intake.Set(ControlMode::PercentOutput, .5);
		}
		else if(arm_stick.GetTrigger()){//intake
			Intake.Set(ControlMode::PercentOutput, -1.0);
		}
		else
		{
			Intake.Set(ControlMode::PercentOutput, 0);
		}

        Wrist.Set(ControlMode::PercentOutput, arm_stick.GetX());
		Shoulder.Set(ControlMode::PercentOutput, -arm_stick.GetY());
	}

	void Ps4Control(){
		//Button Mappings
		
		//Arm Positions
		if(OpController.GetSquareButton()) ArmPosition = HP_PICKUP; 
		if(OpController.GetRightX()) ArmPosition = FLOOR_PICKUP;
		else if(OpController.GetCircleButton()) ArmPosition = MID_SCORE;
		else if(OpController.GetTriangleButton()) ArmPosition = TOP_SCORE;
		else if(OpController.GetR1Button()) ArmPosition = TRAVEL_POS;
		else ArmPosition = ARM_DEFAULT;

		//cone cube
		if(OpController.GetR2Button()) ObjectType = CUBE;
		if(OpController.GetL2Button()) {
			ObjectType = CONE;
		}



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
			sprintf(str, "FR:%d %d",FRSteer.GetSensorCollection().GetPulseWidthPosition(),FRSteer.GetSensorCollection().GetAnalogInRaw());
		    frc::SmartDashboard::PutString("DB/String 0", str);
			sprintf(str, "xFL%d,FR%d,RL%d,RR%d",(int)RobotX[FL],(int)RobotX[FR],(int)RobotX[RL],(int)RobotX[RR]);
			frc::SmartDashboard::PutString("DB/String 1", str);
			sprintf(str, "yFL%d,FR%d,RL%d,RR%d",(int)TargetDir[FL],(int)TargetDir[FR],(int)TargetDir[RL],(int)TargetDir[RR]);
			frc::SmartDashboard::PutString("DB/String 2", str);
 /*
            sprintf(str, "delta:%4.2f",delta);
		    frc::SmartDashboard::PutString("DB/String 0", str);
 */  
			sprintf(str, "Mod:%4.2f,%4.2f",ActDir[FR],TargetDir[FR]);
		    frc::SmartDashboard::PutString("DB/String 3", str);
			sprintf(str, "Tgt:%4.2f,%4.2f",TargetDir[RR],TargetDir[FL]);
		    frc::SmartDashboard::PutString("DB/String 4", str);
			sprintf(str, "gyro= %f",fmod(RobotAngle,360.0));
		    frc::SmartDashboard::PutString("DB/String 5", str);

			//Temp display for encoder values
			sprintf(str, "FL%4.2f,RL%4.2f", fmod(ActDir[FL],360.0), fmod(ActDir[RL],360.0));
		    frc::SmartDashboard::PutString("DB/String 6", str);
			sprintf(str, "FR%4.2f,RR%4.2f", fmod(ActDir[FR],360.0), fmod(ActDir[RR],360.0));
		    frc::SmartDashboard::PutString("DB/String 7", str);
				//sprintf(str, "TMR: %f",TeleTime.Get());
				//frc::SmartDashboard::PutString("DB/String 8", str);
				//sprintf(str, "INTAKE: %.2f", arm_stick.GetY());
				sprintf(str,"SWRVZ%4.2f",SWRVZ);
				frc::SmartDashboard::PutString("DB/String 8", str);


			//Temp End	

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
								sprintf(str, "Pwr:%4.2f",MaxPower);
								frc::SmartDashboard::PutString("DB/String 4", str);
								sprintf(str, "Ang T%d,A%3.0f",Orientation,fmod(RobotAngle,360.0));
								frc::SmartDashboard::PutString("DB/String 5", str);
								sprintf(str, "A:%d,%d,%d,%d",(int)ActDir[FL],(int)ActDir[FR],(int)ActDir[RL],(int)ActDir[RR]);
								frc::SmartDashboard::PutString("DB/String 6", str);
								sprintf(str, "T:%d,%d,%d,%d",(int)TargetDir[FL],(int)TargetDir[FR],(int)TargetDir[RL],(int)TargetDir[RR]);
								frc::SmartDashboard::PutString("DB/String 7", str);
								sprintf(str, "TMR: %f",TeleTime.Get());
								frc::SmartDashboard::PutString("DB/String 8", str);


						}else UpdateCount--;
     }


    //This is were the SWRVX,SWRVY,SWRVZ values are set to direct the swerve drives where to go
	void SwerveControl(void){
		//AutoPilot?
	    if(IsAutonomousEnabled()){
			FieldCentric=1; //Auto opertes in field centric mode
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
			if (fabs(SWRVY)<0.05) SWRVY=0.0;
			SWRVX=dir_stick.GetX();
			if (fabs(SWRVX)<0.05) SWRVX=0.0;
			SWRVZ=rot_stick.GetX();
			if(dir_stick.GetTrigger()) FieldCentric=0;
			else FieldCentric=1;	  
		}
		//Button 6 on left joystick resets the gyro to 0 degrees
		if(rot_stick.GetRawButton(6))
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

	//controlling the direction and speed of the swerve drives
    void SwerveDrive(void) {
    	int i;
    	float Gyro,temp;
      
    	//Read encoder counts and then divide to get degrees of rotation
    	ActDir[FL]=fmod((FLSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0); //GetPulseWidthPosition()
		ActDir[FR]=fmod((FRSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0);
    	ActDir[RL]=fmod((RLSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0);
    	ActDir[RR]=fmod((RRSteer.GetSensorCollection().GetPulseWidthPosition()/11.38),360.0);

		//If rotational power is low, just don't rotate
        if(IsAutonomous()){
               //SWRVZ=pow(3*SWRVZ,3);
		}else{

			if (fabs(SWRVZ)<0.05){ //rot_stick deadzone
				SWRVZ=0.0;
			}else{
				// if(rot_stick.GetRawButton(8)) { // button 8 for fast rotation
				// 	SWRVZ=pow(2.5*SWRVZ,3);  // show off (or big, heavy frame)
				// } else {
				// 	SWRVZ=pow(1*SWRVZ,3); // small frame normal rotation
				// }

			}
	    }
		//deermine if we are field or robot centric and make the adjustment here if necessary    
	    if(FieldCentric||IsAutonomous()) Gyro=RobotAngle*PI/180;
	    else Gyro=0;

		//modify the target robot direction from field centric to robot centric for wheel positions and then set them in calc4wheel
	    temp=SWRVY*cos(Gyro)+SWRVX*sin(Gyro);
	    SWRVX=-SWRVY*sin(Gyro)+SWRVX*cos(Gyro);
	    SWRVY=temp;
		Calc4WheelTurn();

		for(i=0;i<4;i++) SetDirection(i);

        if(rot_stick.GetRawButton(7)){
			TeleTime.Stop();
			TeleTime.Reset();
			TeleStarted=0;
			for(i=0;i<4;i++){
				RobotX[i]=0.0;
				RobotY[i]=0.0;
			}
		}

        if(TeleStarted==0&&fabs(ModSpd[IDX])>.1){
		    TeleTime.Start();
			TeleTime.Reset();
			TeleStarted=1;
		}

        //send the drive motor speeds to the motor controllers
		FLDrive.Set(ControlMode::PercentOutput,-CVTSpeed(SpeedPolarity[FL]*-ModSpd[FL]));
		FRDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[FR]*-ModSpd[FR]));
		RLDrive.Set(ControlMode::PercentOutput,-CVTSpeed(SpeedPolarity[RL]*-ModSpd[RL]));
		RRDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[RR]*-ModSpd[RR]));
		FLSteer.Set(ControlMode::Position, -(TargetDir[FL]/360.0)*4096);
	    FRSteer.Set(ControlMode::Position, -(TargetDir[FR]/360.0)*4096);
	    RRSteer.Set(ControlMode::Position, -(TargetDir[RR]/360.0)*4096);
		RLSteer.Set(ControlMode::Position, -(TargetDir[RL]/360.0)*4096);
    }	// End SwerveDrive

	void ReadGyro(void) {
		RobotAngle = GyroSensor->GetYaw();//-GyroOffset;
        GyroSensor->ZeroYaw();
        //RobotPitch = ahrs->GetPitch();
	}	// End ReadyGyro
	void ResetGyro(void) {
		GyroSensor->ZeroYaw();
	}	// End ResetGyro

    double NewPosition[4],OldPosition[4];
	//Track robot FL wheel X,Y position in inches Y is long side of field X is width
    void TrackRobot(void){
		int i;
        double Dist,Dir;
		for(i=0;i<4;i++){
	     	//Determine distance moved since last time in this routine
			NewPosition[i]=GetLocation(i);
			Dist=NewPosition[i]-OldPosition[i];
			OldPosition[i]=NewPosition[i];
			Dir=ActDir[i]+RobotAngle;
			Dir=fmod(Dir,360.0);
			RobotX[i]+=Dist*sin((Dir*PI)/180.0);
			RobotY[i]+=Dist*cos((Dir*PI)/180.0);
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
		return(result/772.5); //678.44);
	}

	void ArmControl(){
		switch (ArmPosition)
		{
		case HP_PICKUP:
			/* code */
			break;
		case FLOOR_PICKUP:

			break;
		case MID_SCORE:

			break;
		case TOP_SCORE:

			break;
		case TRAVEL_POS:

			break;
		case ARM_DEFAULT:
		default:
			
			break;
		}
	}

  	//AUTONOMOUS DRIVING STATE MACHINE
    void AutoStateMachine(void){
		int *Array, RemainingInches;
		int MaxFromDcel,MaxFromAcel;
		double MaxPower,X,Y,Z ,DeltaA,fctr;
	    int Command, AccSec, DecInches, Speed, AutoX, AutoY, Orientation, IntakeDIR;
        //point data to current autonomous command line
	    Array=AutoArray[AutoLine]; 
		Command=*Array;
		Orientation=*(Array+6);
		Speed=*(Array+3);
		AccSec=*(Array+1);
		DecInches=*(Array+2);
        AutoX=*(Array+4);
		AutoY=*(Array+5);
		IntakeDIR=*(Array+7);
        
		//distance to target
		RemainingInches=sqrt(pow((RobotX[IDX]-AutoX),2)+pow((RobotY[IDX]-AutoY),2));
		//Max speed percentage based on deceleration value given 
		MaxFromDcel=(pow(RemainingInches,2)*10000)/pow(DecInches,2);
		//Get msec since we started and divide by msec to full power for power limit from acceleration
		MaxFromAcel=(int)(AutoTime.Get()*100000)/AccSec;

		//MaxFromAcel=100;
		//MaxFromDcel=100;

        if(MaxFromDcel<MaxFromAcel){
			MaxPower=(MaxFromDcel*Speed)/100;
        }else{
			MaxPower=(MaxFromAcel*Speed)/100;
		}
		if(MaxPower>Speed)MaxPower=Speed;
		if(MaxPower<0)MaxPower=0;
		//Calculate the X,Y direction to target
		X=(AutoX-(int)RobotX[IDX]);
		Y=(AutoY-(int)RobotY[IDX]);
		//Calculate the delta in the robot orientation target
		DeltaA=(RobotAngle-Orientation);
		Z=(DeltaA*-1.0)/90.0;
		if(Z>1.0)Z=1.0;
		if(Z<-1.0) Z=-1.0;

		if(AutoLine>NUMAUTOLINES) Command=STOP; //in case no one put in a stop command
		if(IntakeDIR>0){
			//Intake.Set(ControlMode::PercentOutput, INTAKE_SPEED);
		} else if(IntakeDIR<0){
			//Intake.Set(ControlMode::PercentOutput, -(INTAKE_SPEED));
		} else {
			//Intake.Set(ControlMode::PercentOutput, 0);
		}

		switch(Command){ //command mode
		    case START: 
			            RobotX[IDX]=(double)AutoX;
						RobotY[IDX]=(double)AutoY;
						ResetGyro();
						AutoLine++;
						FirstPass=1;
					    TeleStarted=0; //Trigger Timer to reset and run on power to wheels
		       UpdateScreenAuto( Command, AutoLine, AutoX, AutoY, MaxPower, Orientation);
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
		    case ADJUST: 
			            RobotX[IDX]+=(double)AutoX;
						RobotY[IDX]+=(double)AutoY;
						AutoLine++;
						FirstPass=1;
 		            break;
			case SWITCH:
                        if(FirstPass){
							SwitchTimer.Start();
							SwitchTimer.Reset();
							FirstPass=0;
						}
						AutoDriveX=0.0;
						AutoDriveY=0.0;
						AutoDriveZ=0.0;
						if(SwitchTimer.Get()>0.25_s){//`||BallSensor.Get()){

							//`if(!BallSensor.Get()) {
							//	AutoLine+=AccSec;
							//} else {
							//	AutoLine++;
							//}
						   FirstPass=1;
						}
					break;
			default:
			case STOP:
						AutoDriveX=0.0;
						AutoDriveY=0.0;
						AutoDriveZ=0.0;
						TeleTime.Stop();
						FirstPass=0;
						//Intake.Set(ControlMode::PercentOutput, 0);
					    UpdateScreenAuto( Command, AutoLine, AutoX, AutoY, MaxPower, Orientation);	
						break;
			
		}

		
	

    }

	 
 private:
  frc::Joystick dir_stick{DIR};  //swerve direction
  frc::Joystick rot_stick{ROT};  //swerve rotation
  frc::Joystick arm_stick{ARM};  //arm functions

  frc::PS4Controller OpController{OP_PS4};
  
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
  //frc::DigitalInput BallSensor{0};
  //frc::ADXRS450_Gyro Gyro; //small gyro
  AHRS *GyroSensor; //expansion port gyro
  frc::Timer AutoTime; //seconds timer for auto states
  frc::Timer TeleTime;
  frc::Timer SwitchTimer;
  //cs::UsbCamera Camera1;


  void RobotInit() {

		//Camera1= frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
		RobotInitialized++;

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

		RRSteer.GetSensorCollection().SetPulseWidthPosition(RRSteer.GetSensorCollection().GetPulseWidthPosition()%4096 - frc::Preferences::GetDouble("RRZero",0), TIMEOUT);
		  
        RLDrive.GetSensorCollection().SetIntegratedSensorPosition(0);
        RLDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	 // PIDLoop=0 
		RLDrive.SetSensorPhase(false);
	    RLDrive.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RLDrive.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RLDrive.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RLDrive.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RLDrive.SelectProfileSlot(0, 0);
		RLDrive.Config_kP(0, 0.5, TIMEOUT);
		RLDrive.Config_kI(0, 0.0, TIMEOUT);
		RLDrive.Config_kD(0, 0.0, TIMEOUT);
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
		RRDrive.Config_kP(0, 0.5, TIMEOUT);
		RRDrive.Config_kI(0, 0.0, TIMEOUT);
		RRDrive.Config_kD(0, 0.0, TIMEOUT);
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
		FRDrive.Config_kP(0, 0.5, TIMEOUT);
		FRDrive.Config_kI(0, 0.0, TIMEOUT);
		FRDrive.Config_kD(0, 0.0, TIMEOUT);
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
		FLDrive.Config_kP(0, 0.5, TIMEOUT);
		FLDrive.Config_kI(0, 0.0, TIMEOUT);
		FLDrive.Config_kD(0, 0.0, TIMEOUT);
		FLDrive.SetNeutralMode(NeutralMode::Coast);
		FLDrive.Set(ControlMode::PercentOutput, 0.0);

		Shoulder.GetSensorCollection().SetIntegratedSensorPosition(0);
        Shoulder.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, NOTIMEOUT);	
		Shoulder.SetSensorPhase(false);
	    Shoulder.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Shoulder.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Shoulder.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Shoulder.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Shoulder.SelectProfileSlot(0, 0);
		Shoulder.Config_kP(0, 0.5, TIMEOUT);
		Shoulder.Config_kI(0, 0.0, TIMEOUT);
		Shoulder.Config_kD(0, 0.0, TIMEOUT);
		Shoulder.SetNeutralMode(NeutralMode::Brake);
		Shoulder.Set(ControlMode::PercentOutput, 0.0);

		Wrist.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		Wrist.SetSensorPhase(true);
	    Wrist.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Wrist.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Wrist.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Wrist.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Wrist.SelectProfileSlot(0, 0);
		Wrist.Config_kP(0, PVALUE, TIMEOUT);
		Wrist.Config_kI(0, IVALUE, TIMEOUT);
		Wrist.Config_kD(0, DVALUE, TIMEOUT);
		Wrist.SetNeutralMode(NeutralMode::Coast);
		Wrist.Set(ControlMode::PercentOutput, 0.0);

	//	Wrist.GetSensorCollection().SetPulseWidthPosition(RRSteer.GetSensorCollection().GetPulseWidthPosition()%4096 - frc::Preferences::GetDouble("RRZero",0), TIMEOUT);
		  

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
		 Intake.Set(ControlMode::PercentOutput, 0.0);

		GyroSensor = new AHRS(frc::SPI::Port::kMXP);

		ResetGyro();
	
	}
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
