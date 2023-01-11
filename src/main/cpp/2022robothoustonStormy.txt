/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//#define CALSWERVE  //Move all gears to left side of robot and then read the Zero values for table
#define CAMERA
//#define LIME

#include <stdio.h>
#include <frc/Joystick.h>
#include <frc/DigitalInput.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include "ctre/Phoenix.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <vector>

#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ahrs.h" //https://www.kauailabs.com/public_files/navx-mxp/apidocs/c++/

#include "frc/DriverStation.h"
#ifdef CAMERA
#include <cameraserver/CameraServer.h>
#endif


//LimeLight Defines
#ifdef LIME
	#define TARGET 0
	#define CAMERA 1 5
	#define TARGET_ABOVE_CAMERA	88    //83	// Vertical Distance from camera to center of reflective tape = 6.33 ft
	#define GOAL_ABOVE_SHOOTER	83	// Vertical Distance from shooter to center of upper goal = 6.00 ft
	#define ADD_TO_INNER_GOAL	27	// Horizontal Distance from outer goal to inner goal = 2'5.25"
	#define CAMERA_ANGLE	    35 //40	// Angle (Pitch) of mounted camera
	std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	double targetOffsetAngleHorizontal = table->GetNumber("tx",0.0);
	double targetOffsetAngleVertical = table->GetNumber("ty",0.0);
	double targetArea = table->GetNumber("ta",0.0);
#endif	


#define TIMEOUT 10
#define NOTIMEOUT 0
#define TRUE 1
#define FALSE 0

int Initialized = 0;

//Intake variables
int IntakeIsDown = 0;
int Magazine_On = 0;
int Intake_On=0;
int AutoShootState=0;
double Drive_Speed_Target=0;
double GyroOffset=0.0;
//Shooter Variables
int Shooter_On = 0, Shooter_Trigger_On = 0, ShooterState = 00;
float Shooter_Speed_Target = 0;
int WasTelop=0, WasAuto=0;

float ShooterVelocity=0;

float HoodPower=0;
int TargetPosition=0; //0=close, 1=far


//Joystick defines
#define DIR 0
#define ROT 1
#define ARM 2

//#define PVALUE 1.0
//-------------------CAN BUS DEFINES---------------
//Swerve CAN bus defines for steering motors
#define FLS 4
#define FRS 1
#define RRS 3
#define RLS 2
//Swerve CAN bus defines for drive motors
#define FLD 15
#define FRD 12
#define RLD 13
#define RRD 14
//CAN Bus defines for intake motor
#define INTAKE 8
#define MAGAZINE 7
#define TRIGGER 5
#define HOOD 6
#define SHOOTER 11

//For swerve code, define motor data locations in the arrays
#define RR 0
#define FR 1
#define FL 2
#define RL 3
#define ALL -1 /* all drive channels */
#define IDX  3 /* use Rear Left Wheel as index */
#define PI 3.141592654
#define ENCODER_RES_FL 4096 //4179.0 //4156.0 //4180.0
#define ENCODER_RES_RL 4096 //4152.0 //4185.0 //4180.0
#define ENCODER_RES_FR 4096//4154.0 //4076.0 //4180.0
#define ENCODER_RES_RR 4096 //4162.0 //4175.0 //4180.0
#define SW_L 19.0      //18.75   //Distance between wheels from front to back
#define SW_W 18.875   //18.75  //Distance between wheels from side to side


#define STATE_HOLD 0
#define STATE_FILL_MAGAZINE 1
#define STATE_TRIGGER 2
#define STATE_SPACE 3
#define STATE_END 4



int FieldCentric=0;   //1= feild centric driving
float RobotAngle=0.0; //angle from Gyro
float RobotPitch=0.0;
int UpdateCount=0;    //counter to slow screen update so it does not take too many CPU cycles




//Swerve Control variables  RR, FR, FL, RL
float SpeedPolarity[4]={1.0,1.0,1.0,1.0}; //which way is forward on wheels (switches to speed up swerve rotations)
float ActDir[4]={0.0,0.0,0.0,0.0};        //storage for actual angles of wheel from motor controller encoder feedback
#ifdef CALSWERVE
float ZeroDir[4]={0.0,0.0,0.0,0.0};  //storage for adjustment of actual angles of wheel from motor controller encoder feedback
#else
//float ZeroDir[4]={272.88,69.12,218.16,54.58};
//float ZeroDir[4]={243.36,76.32,210.96,58.18};
//Swerve Control variables  RR, FR, FL, RL
//float ZeroDir[4]={63,76.32,210.96,58.18}; 4/14 6 pm
//float ZeroDir[4]={-120.44,-65.448,300.48,177.84}; //FR -124 //4/18 6:15
//float ZeroDir[4]={120,65,-65,177}; 
float ZeroDir[4]={-60,-115,-105,-3};  //FL was 115, -22, -42, 

#endif
int ClosestZero[4];
float TargetDir[4]={0.0,0.0,0.0,0.0};     //target degrees of each wheel
float ModDir[4]={0.0,0.0,0.0,0.0};        //rotational modification degrees of each wheel to adjust target
float ModSpd[4]={0.0,0.0,0.0,0.0};        //speed modification degrees of each wheel to adjust target
double SWRVY;							  //Swerve Control Variable--Represents Y coordinate of drive joystick
double SWRVX;                             //Swerve Control Variable--Represents X coordinate of drive joystick
double SWRVZ;                             //Swerve Control Variable--Represents X coordinate of rotate joystick
float delta =0.0;                         //local variable for determining best way(least delta degrees) to meet rotatioanl target
AHRS *ahrs;  //gyro

double PercentData = 0;
int TargetIndex=0;
float ShootSpeedError=0.0;

double StartYPosition=0.0,StartXPosition=0.0;
#define SHOOT_SPEED_CLOSE 160 
#define SHOOT_SPEED_FAR   190


#define NUM_DATA_CELLS 9
const double ShooterTable[3][NUM_DATA_CELLS]{
    {94, 111,133,157,178,203,240,264,285,},//Distance
    {30, 31, 32, 40, 40, 42, 61, 55, 68},  //hood angle 
    {166,166,171,196,200,203,221,229,212}  //Velocity
};


#define Climb_Hold  0
#define Climb_Raise 1
#define Climb_Up    2
#define Climb_Arm   3
#define Climb_Drop  4
#define Climb_End   5
#define Climb_Reset 6

int ClimbState = Climb_Hold;
int GearboxLocked = 0;
int LaunchArm = 0;
int IsClimbing = 0;
double ElevatorSpeed = 0;
float ShooterTracker[8];
int TrackerPtr=0;


#define ARM_TIME  .25_s
#define DROP_TIME .1_s
#define EL1     32 
#define EL2     31
#define EL3     30

#define NUMAUTOLINES 30

#define START 10
#define STOP  11
#define MOVE  12
#define FIRE  13

//Auto Pilot variables
    float TargetAngle;
	float CurrentAngle;
	float RotPwr;
	int    AutoLine=0,FirstPass=0,AutoFire=0;
	double RobotX[4]={0.0,0.0,0.0,0.0},RobotY[4]={0.0,0.0,0.0,0.0};
	double AutoDriveX, AutoDriveY, AutoDriveZ;

int *AutoArray;
// 2 Ball Auto
#define BNPW 30
int	TwoBall_AutoArray[NUMAUTOLINES][8]={ //` Test again!!!!
	        //CMD,   Acc mSec,Dec Inches, MaxPwr,       TargetX, TargetY, Orientation Deg,IntakeState
			{START,       0,     0,         0,             0,      0,        0,            0},	//Start at midfield location
			{FIRE,       25,     0,  SHOOT_SPEED_CLOSE,   0,      0,        0,            1},	//pin 1
            {MOVE,      500,     5,        BNPW,          20,    80,        0,            1},	//120 Move back to ball on field			
            {MOVE,      500,     5,        BNPW,          -10,     5,      -18,            1},
            {FIRE,       25,     0,  SHOOT_SPEED_CLOSE,   0,      0,        0,            1},	//pin 1
           // {MOVE,      500,     5,        BNPW,           0,   30,       -23,            1},
			{STOP,        0,     0,      0,                0,      0,        0,           0},	//STOP
};


double outTargetInches,outInchesTraveled;

const double PVALUE = .7; //.75; 
const double IVALUE = 0.0; //0.00125;  
const double DVALUE = 0.00;


class Robot : public frc::TimedRobot
{
public:
    void RobotInit(void);
//AUTONOMOUS DRIVING STATE MACHINE
    void AutoStateMachine(void){
		int *Array;
        double InchesTraveled,TargetInches;
		int MaxFromDcel,MaxFromAcel;
		double MaxPower,X,Y,Z ,DeltaA,fctr;
	    int Command, AccSec, DecInches, Speed, AutoX, AutoY, Orientation, IntakeDIR;
        //point data to current autonomous command line
	    Array=AutoArray+(AutoLine*8); 
		Command=*Array;
		Orientation=*(Array+6);
		Speed=*(Array+3);
		AccSec=*(Array+1);
		DecInches=*(Array+2);
        AutoX=*(Array+4);
		AutoY=*(Array+5);
		IntakeDIR=*(Array+7);
        if(FirstPass){
           StartXPosition=RobotX[IDX];
           StartYPosition=RobotY[IDX];

        }
        
		//distance to target
		InchesTraveled=(int)sqrt(pow((double)(RobotX[IDX]-StartXPosition),2)+pow((double)(RobotY[IDX]-StartYPosition),2));
        TargetInches=(int)sqrt(pow((double)(AutoX-StartXPosition),2)+pow((double)(AutoY-StartYPosition),2));
		//Max speed percentage based on deceleration value given 
		MaxFromDcel=(pow(TargetInches-InchesTraveled,2)*10000)/pow(DecInches,2);
		//Get msec since we started and divide by msec to full power for power limit from acceleration
		//MaxFromAcel=(int)(AutoTimer.Get()*100000)/AccSec;
        MaxFromAcel=100;

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
		Z=(DeltaA*-1.0)/10.0;
		//if(Z>1.0)Z=1.0;
		//if(Z<-1.0) Z=-1.0;

		if(AutoLine>NUMAUTOLINES) Command=STOP; //in case no one put in a stop command
		if(IntakeDIR>0){
			Intake_On=1;
            IntakeIsDown=1;
 		} 

        
        TargetPosition=0; //close shots

		switch(Command){ //command mode
		    case START: 
			            RobotX[IDX]=(double)AutoX;
						RobotY[IDX]=(double)AutoY;
						ResetGyro();
                        AutoTimer.Start();
						if(AutoTimer.Get() > 3_s) AutoLine++; //let everything start and be initialized befor moving--this is probably excessive
 						FirstPass=1;
					    break;
			case MOVE:  //Determine direction and rotation values from autonomous command
			            fctr=fabs(X)+fabs(Y);
						if(fctr==0.0) fctr=1.0;
						
						AutoDriveX=(double)((X/(fctr))*MaxPower)/100.0;
						AutoDriveY=(double)((Y/(fctr))*MaxPower)/100.0;
		               
					    AutoDriveZ=Z; 
					
						if((TargetInches-InchesTraveled)<(Speed/5.0)){
                           AutoLine++;
                           AutoDriveX=0;
                           AutoDriveY=0;
                           AutoDriveZ=0;
						   AutoTimer.Reset();
						   FirstPass=1;
						}else FirstPass=0;


						break;
            case FIRE:  //Determine direction and rotation values from autonomous command
			           if(FirstPass){
                           AutoFire=1;
                       } 
                       FirstPass=0;
                       if(!AutoFire){ //Ball Shot?
                           AutoLine++;
						   AutoTimer.Reset();
						   FirstPass=1;
                       }
                        break;

			default:
			case STOP:
						AutoDriveX=0.0;
						AutoDriveY=0.0;
						AutoDriveZ=0.0;
						FirstPass=0;
                        GyroOffset=157.0;
						break;
			
		}

		outInchesTraveled = InchesTraveled;
        outTargetInches=TargetInches;

    }
	
	
	int ShooterStable(){
	 int error=0;
		
		 ShooterTracker[TrackerPtr]=ShooterVelocity;
		 if((ShooterTracker[TrackerPtr&0x7]-ShooterTracker[(TrackerPtr+1)&0x7])>20) error=1;  //max-min should be <20?
		 TrackerPtr++;
		 TrackerPtr&=0x7; //limit ptr to 0-7 for 8 values
		 //Shooter Velocity is a negative number, Target is posative
		 if(Shooter_Speed_Target>1) ShootSpeedError=100*(fabs(Shooter_Speed_Target-ShooterVelocity)/Shooter_Speed_Target);
		 else ShootSpeedError=100;
		 if(ShootSpeedError>50) error=1;  //be sure shooter is at least running 
		 return(1-error);
		
    }  
	
    void TargetLime(void)
    {
     #ifdef LIME         
            //Get offset values from Limelight
            targetOffsetAngleHorizontal = table->GetNumber("tx",0.0);
            targetOffsetAngleVertical = table->GetNumber("ty",0.0);
            targetArea = table->GetNumber("ta",0.0); //area
                 
            if (targetArea>0){  //Shooter in autotarget and target in view
                DistanceToGoal = (TARGET_ABOVE_CAMERA / tan((CAMERA_ANGLE + targetOffsetAngleVertical) * PI / 180.0)) + ADD_TO_INNER_GOAL;
            } else{
                DistanceToGoal = -100000;
            }
            
      #endif
        
    }


    void Climber()
    {
         
        if(Arm_Stick.GetRawButton(7)) {
            GearboxLocked=1;
            ElevatorSpeed=0;
            IsClimbing=0;
        }
        if(Arm_Stick.GetRawButton(12) && Dir_Stick.GetRawButton(10)) LaunchArm = 1;
        else if(Arm_Stick.GetRawButton(12) && IsClimbing) LaunchArm = 1;
        else LaunchArm=0;

        if(Arm_Stick.GetRawButton(11)){
            IsClimbing=1;
            ElevatorSpeed=2*Arm_Stick.GetY(); 
            GearboxLocked=0;
        }else{
           ElevatorSpeed=0; 
           GearboxLocked=1;
        }

        Gearbox_Lock.Set(!GearboxLocked);
        Telescope_Lock.Set(LaunchArm); 
         
        if(GearboxLocked){
            ElevatorSpeed=0;
            LockTimer.Reset();
        }


        if(LockTimer.Get() > .1_s){
            Elevator_1.Set(ControlMode::PercentOutput,-ElevatorSpeed);
            Elevator_2.Set(ControlMode::PercentOutput,-ElevatorSpeed);
            Elevator_3.Set(ControlMode::PercentOutput,-ElevatorSpeed);
        }else{
            Elevator_1.Set(ControlMode::PercentOutput,0);
            Elevator_2.Set(ControlMode::PercentOutput,0);
            Elevator_3.Set(ControlMode::PercentOutput,0);

        }
    

    }

    void Intake()
    {
        if(IsAutonomousEnabled()){
            Magazine_On=1;
        }else{
            if(!Shooter_On) Intake_On=0; //default to off unless shooter wheel is running
 
            if (Dir_Stick.GetRawButton(3)) //Intake Up
            {
                IntakeIsDown = 0;
            }


            if (Dir_Stick.GetTrigger())//Make Intake Down
            {
                IntakeIsDown = 1;
				Intake_On=1;
                Magazine_On=1;
            } 
			else if (Dir_Stick.GetRawButton(2))//spit out ball
            {
                Intake_On=-1; 
                Magazine_On = -1;
            } 
			else
            {
                if(!Shooter_On) Magazine_On = 0;  //magazine stop unless shooting
            }
          
        }
    
        if(!IntakeIsDown){
			Intake_Cylinder2.Set(0);
            if(IntakeTimer.Get() > 0.5_s){
                    Intake_Cylinder.Set(0);
            }
        }
        else {
            IntakeTimer.Reset();
    
    
            Intake_Cylinder.Set(1);
			Intake_Cylinder2.Set(1);
        }

        if(Intake_On==1&&IntakeIsDown){
           Intake_Motor.Set(ControlMode::PercentOutput, -.8);//forward
        }else if(Intake_On==-1&&IntakeIsDown){
            Intake_Motor.Set(ControlMode::PercentOutput, .8);//reverse
        }else{
            Intake_Motor.Set(ControlMode::PercentOutput, 0);//off
        }


        if(BallDetect.Get()&&!Shooter_Trigger_On&&Magazine_On>0){ //stop magazine, if there is a ball ready to fire, and not currently firing
           Magazine_On=0;
        }
        if (Magazine_On == 1) {
            Magazine_Motor.Set(ControlMode::PercentOutput, 1); //forward
        }else if (Magazine_On == -1) {
            Magazine_Motor.Set(ControlMode::PercentOutput, -1.0);//reverse
        } else{
            Magazine_Motor.Set(ControlMode::PercentOutput, 0); //off
        }

        
    }



    void Shooter()
    {
        if(Arm_Stick.GetRawButton(5)) TargetPosition=0;
        if(Arm_Stick.GetRawButton(6)) TargetPosition=1;
   
        if(Arm_Stick.GetRawButton(3)||IsAutonomousEnabled()){//force spin up of shooter and staging of ball in magazine
            Shooter_On=1;
            Magazine_On=1;
        }else{
            Shooter_On=0;
        }
		
        if(Arm_Stick.GetTrigger()){ //detect operator request to fire
                AutoFire=1;
        }
		
        switch (AutoShootState){
                default:
                        AutoShootState=STATE_HOLD;
                        break;
                case STATE_HOLD:
                        if(AutoFire){//request to fire received
                            AutoShootState=STATE_FILL_MAGAZINE;
                        }
                        break;
                case STATE_FILL_MAGAZINE:
                        Magazine_On=1;
                        Shooter_On=1;
                        Intake_On=1;
                        if((BallDetect.Get() && (ShooterStable())||IsAutonomous())){//} || (IsTeleop()&&Arm_Stick.GetRawButton()){ 
                            AutoShootState=STATE_TRIGGER;
							TriggerTimer.Reset();
                        }
                         break;
                case STATE_TRIGGER:
                        Magazine_On=1;
                        Shooter_On=1;
                        Intake_On=1;
                        Shooter_Trigger_On=1;
                        if(TriggerTimer.Get() > .25_s){ 
                            AutoShootState=STATE_SPACE;
                            TriggerTimer.Reset(); //start timing the backing up of the next ball
                        }
                        break;
                 case STATE_SPACE:
   					   Shooter_Trigger_On=0; //stop trigger wheel
                       Magazine_On=-1; //back up any ball waiting to separate
                       Shooter_On=0;
                       Intake_On=1; //keep any ball being backed up from falling out
                       if(TriggerTimer.Get()>.5_s) { //ball backed up, stop motors
                              AutoShootState=STATE_END;
                       }
                       break;
                 case STATE_END:
                       Magazine_On=0;
                       Shooter_On=0;
                       Intake_On=0;
                       Shooter_Trigger_On=0;
                       AutoShootState=STATE_HOLD;
                       AutoFire=0;
                       break;

            }
         
        
            //control trigger wheel
            if(Shooter_Trigger_On){
                Trigger_Motor.Set(ControlMode::PercentOutput,-1); //trigger running 
            }else{
                Trigger_Motor.Set(ControlMode::PercentOutput, 0); //off
            } 
		
            //control shooter motor and hood
            if(Shooter_On){
                SpinDownTimer.Reset();
                if(TargetPosition==0){
                    Shooter_Speed_Target= SHOOT_SPEED_CLOSE; //+(40.0*Arm_Stick.GetZ());	
                    HoodPower=-.25; //close position
                }else{
                    Shooter_Speed_Target=SHOOT_SPEED_FAR;//+(40.0*Arm_Stick.GetZ());
                    HoodPower=.50;  //far position
                }
            }else{
                if(SpinDownTimer.Get()>4_s){
                    Shooter_Speed_Target=0; //turn off shooter after 4 seconds of inactivity
                    HoodPower=0;
                }
            }
            if(IsClimbing) HoodPower=-.25; //hood down during climb
            Hood_Motor.Set(ControlMode::PercentOutput,HoodPower);
            Shooter_Motor.Set(ControlMode::Velocity,Shooter_Speed_Target); //0 to 1024 //inverted from -
            
            ShooterVelocity=Shooter_Motor.GetSelectedSensorVelocity(1)/64;
        
    }

    int gUpdateCount = 0; ///counter to be sure we only update drive screen every once in a while so it does not bog down the network
    void UpdateDriverScreen(void) //` fix later to display what you actually want
    {
        char str[40];
        int light;

        if (gUpdateCount <= 0)
        {
            gUpdateCount = 25; //delay between displays to not bog down system
   
           //Code for displaying swerve values
           sprintf(str,"FLA%d,T:%4.2f",FLSteer.GetSensorCollection().GetPulseWidthPosition(),TargetDir[FL]*(4096/360.0));
           frc::SmartDashboard::PutString("DB/String 0",str);
           sprintf(str,"FRA%d,T:%4.2f",FRSteer.GetSensorCollection().GetPulseWidthPosition(),TargetDir[FR]*(4096/360.0));
           frc::SmartDashboard::PutString("DB/String 1",str);
           sprintf(str,"RLA%d,T:%4.2f",RLSteer.GetSensorCollection().GetPulseWidthPosition(),TargetDir[RL]*(4096/360.0));
           frc::SmartDashboard::PutString("DB/String 2",str);
           sprintf(str,"RRA%d,T:%4.2f",RRSteer.GetSensorCollection().GetPulseWidthPosition(),TargetDir[RR]*(4096/360.0));
           frc::SmartDashboard::PutString("DB/String 3",str);
           sprintf(str,"BD%d,ST%4.2f,SA%4.2f",BallDetect.Get(),Shooter_Speed_Target,ShooterVelocity);
           frc::SmartDashboard::PutString("DB/String 4",str);
			sprintf(str,"Ang%4.2f,CLMB:%d,P%4.2f",RobotAngle,IsClimbing,RobotPitch);
		   frc::SmartDashboard::PutString("DB/String 5", str);
           sprintf(str, "AL %d, St:%d, T:%3.1f",AutoLine, AutoShootState,AutoTimer.Get());
           frc::SmartDashboard::PutString("DB/String 6",str);
	       sprintf(str, "EAmp%4.1f,%4.1f,%4.1f",Elevator_1.GetStatorCurrent(),Elevator_2.GetStatorCurrent(),Elevator_3.GetStatorCurrent());
           frc::SmartDashboard::PutString("DB/String 7",str);
		   //sprintf(str, "Z:%4.2f SWRVZ%4.2f",Rot_Stick.GetX(),SWRVZ);
           sprintf(str,"SC%4.2f",RLDrive.GetSupplyCurrent());
           frc::SmartDashboard::PutString("DB/String 8",str);
		   
           light = BallDetect.Get();
		   frc::SmartDashboard::PutNumber("DB/LED 0",light);
           frc::SmartDashboard::PutNumber("DB/LED 1",Shooter_On);

        }
        gUpdateCount--;
    } // End UpdateDriverScreen
	
 //This is were the SWRVX,SWRVY,SWRVZ values are set to direct the swerve drives where to go
	void SwerveControl(void){
        double Zsign;
		//AutoPilot?
	    if(IsAutonomousEnabled()){
		    SWRVY=AutoDriveY;
			SWRVX=AutoDriveX; 
	        SWRVZ=AutoDriveZ;
            
			if(SWRVZ>5.0)SWRVZ=5.0;
			if(SWRVZ<-5.0) SWRVZ=-5.0;
			if(SWRVY>1.0)SWRVY=1.0;
			if(SWRVY<-1.0) SWRVY=-1.0;
			if(SWRVX>1.0)SWRVX=1.0;
			if(SWRVX<-1.0) SWRVX=-1.0;
		}else{ //Teleop Mode
			SWRVY=-Dir_Stick.GetY();
			if (fabs(SWRVY)<0.02) SWRVY=0.0;
			SWRVX=Dir_Stick.GetX();
			if (fabs(SWRVX)<0.02) SWRVX=0.0;
            SWRVZ=Rot_Stick.GetX(); 
			if(fabs(SWRVZ)<0.02) SWRVZ=0.0;
           // SWRVZ=pow(2.0*SWRVZ,1);  //`change to 2.0,5 when able to test//was 3.0 and 3//make rotation exponential
            Zsign=4.0;
            if(SWRVZ<0) Zsign=-4.0;
            SWRVZ=pow(fabs(SWRVZ),.5);  //`change to 2.0,5 when able to test//was 3.0 and 3//make rotation exponential
            SWRVZ*=Zsign;
        	if(Rot_Stick.GetTrigger()) FieldCentric=0;
			else FieldCentric=1;	  
		}
		//Button 9 on left joystick resets the gyro to 0 degrees
		if(Dir_Stick.GetRawButton(9))
		{
			ResetGyro();
            GyroOffset=0;
		}
	
  }	// End SwerveControl

  //Do the math on each wheel to find the new angle and speed required
  void  Calc4WheelTurn(){

	  double A,B,C,D,R,temp;
	  int i;
	  R=sqrt(pow(SW_L,2))+(pow(SW_W,2));

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
		
		delta=ModDir[channel]-curdir+ZeroDir[channel]; //What is the distance to move?  How best to get there...

		delta=fmod(delta,360.0); //`
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
    	ActDir[FL]=fmod((FLSteer.GetSensorCollection().GetPulseWidthPosition()/(ENCODER_RES_FL/360.0)) - ZeroDir[FL],360.0); //was divided by 11.38
		ActDir[FR]=fmod((FRSteer.GetSensorCollection().GetPulseWidthPosition()/(ENCODER_RES_FR/360.0)) - ZeroDir[FR],360.0);
    	ActDir[RL]=fmod((RLSteer.GetSensorCollection().GetPulseWidthPosition()/(ENCODER_RES_RL/360.0)) - ZeroDir[RL],360.0);
    	ActDir[RR]=fmod((RRSteer.GetSensorCollection().GetPulseWidthPosition()/(ENCODER_RES_RR/360.0)) - ZeroDir[RR],360.0);
    

		//deermine if we are field or robot centric and make the adjustment here if necessary    
	    if(FieldCentric||IsAutonomousEnabled()) Gyro=RobotAngle*PI/180;
	    else Gyro=0;

		//modify the target robot direction from field centric to robot centric for wheel positions and then set them in calc4wheel
	    temp=SWRVY*cos(Gyro)+SWRVX*sin(Gyro);
	    SWRVX=-SWRVY*sin(Gyro)+SWRVX*cos(Gyro);
	    SWRVY=temp;
		Calc4WheelTurn();

		for(i=0;i<4;i++) SetDirection(i);
      
        //send the drive motor speeds to the motor controllers
		FLDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[FL]*ModSpd[FL]));
		FRDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[FR]*ModSpd[FR]));
		RLDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[RL]*ModSpd[RL]));
		RRDrive.Set(ControlMode::PercentOutput,CVTSpeed(SpeedPolarity[RR]*ModSpd[RR]));
        #ifndef CALSWERVE
        if(fabs(SWRVX)>.01||fabs(SWRVY)>.01 || fabs(SWRVZ)>0.1){
     	#endif
           FLSteer.Set(ControlMode::Position,((TargetDir[FL]/360.0)*ENCODER_RES_FL) + ClosestZero[FL]);
		  FRSteer.Set(ControlMode::Position,((TargetDir[FR]/360.0)*ENCODER_RES_FR) + ClosestZero[FR]);
		  RRSteer.Set(ControlMode::Position,((TargetDir[RR]/360.0)*ENCODER_RES_RR) + ClosestZero[RR]);
		  RLSteer.Set(ControlMode::Position,((TargetDir[RL]/360.0)*ENCODER_RES_RL) + ClosestZero[RL]);
        #ifndef CALSWERVE
        }
        #endif
    }	// End SwerveDrive

double NewPosition[4],OldPosition[4];
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
		return(result/850); //772.5 //678.44);
	}

    void ReadGyro(void) {
		RobotAngle = ahrs->GetYaw()-GyroOffset;
        
        //RobotPitch = ahrs->GetPitch();
	}	// End ReadyGyro
	void ResetGyro(void) {
		ahrs->ZeroYaw();
	}	// End ResetGyro

 
void AutonomousPeriodic() override
    {
        if (!Initialized)
        {
            RobotInit();
            Initialized=1;   
  	  //   PVALUE = frc::SmartDashboard::GetNumber("DB/Slider 0",-1);
     //   IVALUE = frc::SmartDashboard::GetNumber("DB/Slider 1",-1);
     //   DVALUE = frc::SmartDashboard::GetNumber("DB/Slider 2",-1);
          /*switch(StartingLocation){
                //change which arrays are selected in the future
                default:
                case 0:  AutoArray = (int*)&TwoBall_AutoArray; break;
                case 1:  AutoArray = (int*)&TwoBall_AutoArray; break;
                case 2:  AutoArray = (int*)&TwoBall_AutoArray; break;
                case 3:  AutoArray = (int*)&TwoBall_AutoArray; break;
                case 4:  AutoArray = (int*)&TwoBall_AutoArray; break;
                case 5:  AutoArray = (int*)&TwoBall_AutoArray; break;
                break;
            }*/
            
   	
        }
        AutoArray = (int*)&TwoBall_AutoArray; //remove if we ever have multiple autonomouses
             	
		ReadGyro();              //get the angle of the robot in to variable "RobotAngle"
		TrackRobot();			 //Track the Robot's FL wheel X,Y position around the field
        AutoStateMachine();      //Get updated directional commands from autopilot
		SwerveControl();         //convert auto pilot commands to joystick motions
		SwerveDrive();           //Update the speed and direction of the swerve drive motors
        TargetLime();
	    Intake();
        Shooter();
        UpdateDriverScreen();
        
    }

    void TeleopPeriodic() override
    {

        if (!Initialized)
        {
            RobotInit(); 
            Initialized=1;
        }
       #ifdef CALSWERVE
			ZeroDir[RR]=frc::SmartDashboard::GetNumber("DB/Slider 0",0)*72; 
			ZeroDir[FR]=frc::SmartDashboard::GetNumber("DB/Slider 1",0)*72;
			ZeroDir[FL]=frc::SmartDashboard::GetNumber("DB/Slider 2",0)*72;
			ZeroDir[RL]=frc::SmartDashboard::GetNumber("DB/Slider 3",0)*72;
			if(ZeroDir[RR]>180) ZeroDir[RR] = (ZeroDir[RR] - 360.0);
			if(ZeroDir[FR]>180) ZeroDir[FR] = (ZeroDir[FR] - 360.0);
			if(ZeroDir[FL]>180) ZeroDir[FL] = (ZeroDir[FL] - 360.0);
			if(ZeroDir[RL]>180) ZeroDir[RL] = (ZeroDir[RL] - 360.0);
	   #endif

         SwerveControl(); 
	     SwerveDrive();
         ReadGyro();
         TargetLime();
	     Climber();
         Intake();
         Shooter();
         UpdateDriverScreen(); 

    }

private:

    frc::Joystick Rot_Stick{ROT};
    frc::Joystick Dir_Stick{DIR};
    frc::Joystick Arm_Stick{ARM};

    TalonFX RLDrive = {RLD};
    TalonFX FLDrive = {FLD};
    TalonFX FRDrive = {FRD};
    TalonFX RRDrive = {RRD};
    TalonSRX Intake_Motor = {INTAKE};
    TalonSRX Magazine_Motor = {MAGAZINE};
    TalonFX Shooter_Motor = {SHOOTER};
    TalonSRX Hood_Motor = {HOOD};
    TalonSRX Trigger_Motor = {TRIGGER};
    TalonSRX RLSteer = {RLS};
    TalonSRX RRSteer = {RRS};
    TalonSRX FRSteer = {FRS};
    TalonSRX FLSteer = {FLS};
    TalonSRX Elevator_1 = {EL1};
    TalonSRX Elevator_2 = {EL2};
    TalonSRX Elevator_3 = {EL3};

    frc::Solenoid Intake_Cylinder{frc::PneumaticsModuleType::CTREPCM,0}; //O is intake 
    frc::Solenoid Telescope_Lock{frc::PneumaticsModuleType::CTREPCM,1}; //is Telescoping arm lock
    frc::Solenoid Gearbox_Lock{frc::PneumaticsModuleType::CTREPCM,2}; //gearbox Lock / elevator lock
    frc::Solenoid Intake_Cylinder2{frc::PneumaticsModuleType::CTREPCM,3}; //3 is intake small cyclinder


    frc::Timer SpinDownTimer;
    frc::Timer SpinUpTimer;
    frc::Timer ClimbTimer;
    frc::Timer AutoTimer;
    frc::Timer TelopTimer;
    frc::Timer TriggerTimer;
    frc::Timer LockTimer;
    frc::Timer IntakeTimer;

    frc::DigitalInput ElevatorBottom{1};      
    frc::DigitalInput BallDetect{0};

    

};


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif

void Robot::RobotInit()
{
    Initialized = 1;
    ahrs = new AHRS(frc::SPI::Port::kMXP);
    Intake_Cylinder.Set(0);
	Intake_Cylinder2.Set(0);
    Telescope_Lock.Set(0);
    Gearbox_Lock.Set(0);
    IntakeIsDown = 0;
    ClimbTimer.Start();
    ClimbTimer.Reset();

    SpinUpTimer.Start();
    SpinUpTimer.Reset();
    SpinDownTimer.Start();
    SpinDownTimer.Reset();
    IntakeTimer.Start();
    IntakeTimer.Reset();
    //AutoTimer.Start();
    AutoTimer.Reset();
    TriggerTimer.Start();
    LockTimer.Start();
    
    Hood_Motor.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	
    Hood_Motor.SetSensorPhase(true);
    Hood_Motor.ConfigNominalOutputForward(0.0f, TIMEOUT);
    Hood_Motor.ConfigNominalOutputReverse(0.0f, TIMEOUT);
    Hood_Motor.ConfigPeakOutputForward(+12.0f, TIMEOUT);
    Hood_Motor.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
    Hood_Motor.SelectProfileSlot(0, 0);
    Hood_Motor.Config_kP(0, 1.0, TIMEOUT);
    Hood_Motor.Config_kI(0, 0.004, TIMEOUT);
    Hood_Motor.Config_kD(0, 0.0, TIMEOUT);
    Hood_Motor.Config_kF(0, 0.0, TIMEOUT);
  
    
    Shooter_Motor.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition);
    Shooter_Motor.SetSensorPhase(true);
    Shooter_Motor.ConfigNominalOutputForward(0.0f, TIMEOUT);
    Shooter_Motor.ConfigNominalOutputReverse(0.0f, TIMEOUT);
    Shooter_Motor.ConfigPeakOutputForward(+12.0f, TIMEOUT);
    Shooter_Motor.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
    Shooter_Motor.SelectProfileSlot(0, 0);
    Shooter_Motor.Config_kP(0, 0.0, TIMEOUT);
    Shooter_Motor.Config_kI(0, 0.0, TIMEOUT);
    Shooter_Motor.Config_kD(0, 0.0, TIMEOUT);
    Shooter_Motor.Config_kF(0, 4.0, TIMEOUT);
    Shooter_Motor.Set(ControlMode::PercentOutput, 0.0);

    Magazine_Motor.ConfigNominalOutputForward(0.0f, TIMEOUT);
    Magazine_Motor.ConfigNominalOutputReverse(0.0f, TIMEOUT);
    Magazine_Motor.ConfigPeakOutputForward(+12.0f, TIMEOUT);
    Magazine_Motor.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
    Magazine_Motor.SelectProfileSlot(0, 0);
    Magazine_Motor.Config_kP(0, 0.5, TIMEOUT);
    Magazine_Motor.Config_kI(0, 0.0, TIMEOUT);
    Magazine_Motor.Config_kD(0, 0.0, TIMEOUT);
    Magazine_Motor.Set(ControlMode::PercentOutput, 0.0);


        ClosestZero[FL] = FLSteer.GetSensorCollection().GetPulseWidthPosition() / 4096;
        ClosestZero[FR] = FRSteer.GetSensorCollection().GetPulseWidthPosition() / 4096;
        ClosestZero[RL] = RLSteer.GetSensorCollection().GetPulseWidthPosition() / 4096;
        ClosestZero[RR] = RRSteer.GetSensorCollection().GetPulseWidthPosition() / 4096; 

	    
	    FLSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);
		FLSteer.SetSensorPhase(false);
		FLSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FLSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	   	FLSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FLSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FLSteer.SelectProfileSlot(0, 0);
		FLSteer.Config_kP(0, PVALUE, TIMEOUT);
		FLSteer.Config_kI(0, IVALUE, TIMEOUT);
		FLSteer.Config_kD(0, DVALUE, TIMEOUT);
        FLSteer.Config_kF(0, 0.0, TIMEOUT);
        FLSteer.ConfigFeedbackNotContinuous(0,TIMEOUT);
        FLSteer.ConfigClosedloopRamp(.3,TIMEOUT);
        FLSteer.Set(ControlMode::Position, ClosestZero[FL]);
       // FLSteer.SetSelectedSensorPosition(0,0,TIMEOUT);

        
        FRSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		FRSteer.SetSensorPhase(false);
	    FRSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		FRSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    FRSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	FRSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		FRSteer.SelectProfileSlot(0, 0);
		FRSteer.Config_kP(0, PVALUE, TIMEOUT);
		FRSteer.Config_kI(0, IVALUE, TIMEOUT);
		FRSteer.Config_kD(0, DVALUE, TIMEOUT);
        FRSteer.Config_kF(0, 0.0, TIMEOUT);
        FRSteer.ConfigFeedbackNotContinuous(0,TIMEOUT);
        FRSteer.Set(ControlMode::Position, ClosestZero[FR]);
        FRSteer.ConfigClosedloopRamp(.3,TIMEOUT);
      
        RLSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	
		RLSteer.SetSensorPhase(false);
	    RLSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RLSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RLSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RLSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RLSteer.SelectProfileSlot(0, 0);
		RLSteer.Config_kP(0, PVALUE, TIMEOUT);
		RLSteer.Config_kI(0, IVALUE, TIMEOUT);
		RLSteer.Config_kD(0, DVALUE, TIMEOUT);
		RLSteer.Config_kF(0, 0.0, TIMEOUT);
        RLSteer.ConfigFeedbackNotContinuous(0,TIMEOUT);
        //RLSteer.Set(ControlMode::Position, 0.0);
        RLSteer.Set(ControlMode::Position, ClosestZero[RL]);
        RLSteer.ConfigClosedloopRamp(.3,TIMEOUT);
 	    
        RRSteer.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	 
		RRSteer.SetSensorPhase(false);
	    RRSteer.ConfigNominalOutputForward(0.0f, TIMEOUT);
		RRSteer.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    RRSteer.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	RRSteer.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		RRSteer.SelectProfileSlot(0, 0);
		RRSteer.Config_kP(0, PVALUE, TIMEOUT);
		RRSteer.Config_kI(0, IVALUE, TIMEOUT);
		RRSteer.Config_kD(0, DVALUE, TIMEOUT);
        RRSteer.Config_kF(0, 0.0, TIMEOUT);
        RRSteer.ConfigFeedbackNotContinuous(0,TIMEOUT);
        RRSteer.Set(ControlMode::Position, ClosestZero[RR]);
		RRSteer.ConfigClosedloopRamp(.3,TIMEOUT);
         

        
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
        RLDrive.ConfigClosedloopRamp(.3,TIMEOUT);
        RLDrive.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true,40.0,40.0,.1),TIMEOUT);
        

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
		RRDrive.Set(ControlMode::Position, 0.0);
        RRDrive.ConfigClosedloopRamp(.3,TIMEOUT);
        RRDrive.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true,40.0,40.0,.1),TIMEOUT);

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
		FRDrive.Set(ControlMode::Position, 0.0);
        FRDrive.ConfigClosedloopRamp(.3,TIMEOUT);
        FRDrive.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true,40.0,40.0,.1),TIMEOUT);

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
		FLDrive.Set(ControlMode::Position, 0.0);
        FLDrive.ConfigClosedloopRamp(.3,TIMEOUT);
        FLDrive.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true,40.0,40.0,.1),TIMEOUT);

        //CLimber
        Elevator_1.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);
        Elevator_1.SetSensorPhase(true);
	    Elevator_1.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Elevator_1.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Elevator_1.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Elevator_1.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Elevator_1.SelectProfileSlot(0, 0);
		Elevator_1.Config_kP(0, .75, TIMEOUT);
		Elevator_1.Config_kI(0, 0.0, TIMEOUT);
		Elevator_1.Config_kD(0, 0.0, TIMEOUT);
		Elevator_1.Config_kF(0, 0.0, TIMEOUT);


        Elevator_2.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	
		Elevator_2.SetSensorPhase(true);
	    Elevator_2.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Elevator_2.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Elevator_2.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Elevator_2.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Elevator_2.SelectProfileSlot(0, 0);
		Elevator_2.Config_kP(0, .75, TIMEOUT);
		Elevator_2.Config_kI(0, 0.0, TIMEOUT);
		Elevator_2.Config_kD(0, 0.0, TIMEOUT);
		Elevator_2.Config_kF(0, 0.0, TIMEOUT);
        //Elevator_2.Set(ControlMode::Position, 0.0);

        Elevator_3.ConfigSelectedFeedbackSensor(FeedbackDevice::PulseWidthEncodedPosition, 0, NOTIMEOUT);	
		Elevator_3.SetSensorPhase(true);
	    Elevator_3.ConfigNominalOutputForward(0.0f, TIMEOUT);
		Elevator_3.ConfigNominalOutputReverse(0.0f, TIMEOUT);
	    Elevator_3.ConfigPeakOutputForward(+12.0f, TIMEOUT);
	 	Elevator_3.ConfigPeakOutputReverse(-12.0f, TIMEOUT);
		Elevator_3.SelectProfileSlot(0, 0);
		Elevator_3.Config_kP(0, .75, TIMEOUT);
		Elevator_3.Config_kI(0, 0.0, TIMEOUT);
		Elevator_3.Config_kD(0, 0.0, TIMEOUT);
		Elevator_3.Config_kF(0, 0.0, TIMEOUT);
        //Elevator_3.Set(ControlMode::Position, 0.0);

        //ElevatorZero = Elevator_1.GetSensorCollection().GetPulseWidthPosition();


    //Color Sensor
    //m_colorMatcher.AddColorMatch(kBlueTarget);
    //m_colorMatcher.AddColorMatch(kRedTarget);

    //Camera
	#ifdef CAMERA
    frc::CameraServer::StartAutomaticCapture();
    //frc::CameraServer::StartAutomaticCapture(1);
    
    #endif
    //Getting Alliance and location
    //Alliance = frc::DriverStation::GetAlliance();
    //StartingLocation = frc::DriverStation::GetAlliance()*10 + frc::DriverStation::GetLocation();
    //StartingLocation = 0; //frc::SmartDashboard::GetNumber("DB/Slider 0",-1);
    ResetGyro();
  
}