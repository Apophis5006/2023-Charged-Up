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
#include <frc/Preferences.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/PIDSubsystem.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

#include "ctre/Phoenix.h"

//#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include "ahrs.h" //https://www.kauailabs.com/public_files/navx-mxp/apidocs/c++/

#include "frc/DriverStation.h"
#ifdef CAMERA
#include <cameraserver/CameraServer.h>
#endif


#define TIMEOUT 10
#define NOTIMEOUT 0
#define TRUE 1
#define FALSE 0

int Initialized = 0;

//Joystick defines
#define DIR 0
#define ROT 1
#define ARM 2

#define PI 3.141592654

#define PVALUE .7 //0.15
#define DVALUE 0.0
#define IVALUE 0.0

//Swerve CAN bus defines for drive motors
#define FLD 14
#define FRD 13
#define RLD 12
#define RRD 15
//Swerve CAN bus defines for steering motors
#define RRS 6
#define FRS 7
#define FLS 10
#define RLS 11
//Arm & Intake
#define ARM_ID 8
#define INTAKE_ID 9


int UpdateCount=0;    //counter to slow screen update so it does not take too many CPU cycles
double RobotAngle=0.0;
double RobotPitch=0.0;
double GyroOffset=0.0;
int EncoderAct = 0;        //storage for actual angles of wheel from motor controller encoder feedback

int IntakeOn=0;
int ArmPosition = 0;
double ArmAngel = 0.0;

//ArmStates

frc::SwerveModuleState FLState,FRState,RRState,RLState;
double FLZero,FRZero,RRZero,RLZero = 0;

                    //local variable for determining best way(least delta degrees) to meet rotatioanl target
AHRS *ahrs;  //gyro


//#define ARM_TIME  .25_s



class Robot : public frc::TimedRobot
{
public:
    void RobotInit(void);
    
 
    int gUpdateCount = 0; ///counter to be sure we only update drive screen every once in a while so it does not bog down the network
    void UpdateDriverScreen(void) //` fix later to display what you actually want
    {
        char str[40];
   

        if (gUpdateCount <= 0)
        {
            gUpdateCount = 25; //delay between displays to not bog down system
   
           //Code for displaying swerve values
            sprintf(str,"Theta:%4.2f",theta);
         frc::SmartDashboard::PutString("DB/String 1",str);
            sprintf(str,"Encoder:%4.2f",(double)(FLSteer.GetSensorCollection().GetPulseWidthPosition()*(360/4096)));
        frc::SmartDashboard::PutString("DB/String 2",str);
          sprintf(str,"RotStick:%4.2f",Rot_Stick.GetX());
         frc::SmartDashboard::PutString("DB/String 3",str);
             sprintf(str,"DirStick:%4.2f",Dir_Stick.GetX());
         frc::SmartDashboard::PutString("DB/String 4",str);
		

            sprintf(str,"RL:%4.2f S:%4.2f",(double)-RLState.angle.Degrees(),RLState.speed);
         frc::SmartDashboard::PutString("DB/String 6",str);
         sprintf(str,"RR A:%4.2f S:%4.2f",(double)-RRState.angle.Degrees(),RRState.speed);
         frc::SmartDashboard::PutString("DB/String 7",str);
         sprintf(str,"Fl A:%4.2f S:%4.2f",(double)-FLState.angle.Degrees(),FLState.speed);
         frc::SmartDashboard::PutString("DB/String 8",str);
         sprintf(str,"FR A:%4.2f S:%4.2f",(double)-FRState.angle.Degrees(),FRState.speed);
         frc::SmartDashboard::PutString("DB/String 9",str);

		 //  frc::SmartDashboard::PutNumber("DB/LED 0",light);
          // frc::SmartDashboard::PutNumber("DB/LED 1",Shooter_On);

        //vx = units::meters_per_second_t(frc::SmartDashboard::GetNumber("DB/String 0",0));
        //vy = units::meters_per_second_t(frc::SmartDashboard::GetNumber("DB/String 5",0));

        }
        gUpdateCount--;
    } // End UpdateDriverScreen
	
    void ReadGyro(void) {
		RobotAngle = ahrs->GetYaw();
        
        RobotPitch = ahrs->GetRoll()-GyroOffset;
	}	// End ReadyGyro
	void ResetGyro(void) {
		ahrs->ZeroYaw();
        
        GyroOffset=ahrs->GetRoll();
	}	// End ResetGyro

 
    void AutonomousPeriodic() override
    {
   
        UpdateDriverScreen();
        
    }


    void TeleopPeriodic() override
    {

        if (!Initialized)
        {
            RobotInit(); 
            Initialized=1;
        }
        
        vx = Dir_Stick.GetY() * MAX_LINEAR_VELOCITY;
        vy = Dir_Stick.GetX() * MAX_LINEAR_VELOCITY;
        
        //theta = units::radians_per_second_t(Rot_Stick.GetX() * MAX_ANGULAR_VELOCITY);
        theta = 0_rad_per_s;

        Drive();

        ReadGyro(); 

        UpdateDriverScreen(); 

    }

    void TestPeriodic() override{
        if (!Initialized)
        {
            RobotInit(); 
            Initialized=1;
        }

        ReadGyro();

        UpdateDriverScreen();

        if(Rot_Stick.GetTrigger()){ //Cal Swerve Zeros write
            frc::Preferences::SetDouble("FLZero",FLZero);
            frc::Preferences::SetDouble("FRZero",FRZero);
            frc::Preferences::SetDouble("RLZero",RLZero);
            frc::Preferences::SetDouble("RRZero",RRZero);            
        }

        FLZero = FLSteer.GetSensorCollection().GetPulseWidthPosition();
        FRZero = FRSteer.GetSensorCollection().GetPulseWidthPosition();
        RLZero = RLSteer.GetSensorCollection().GetPulseWidthPosition();
        RRZero = RRSteer.GetSensorCollection().GetPulseWidthPosition(); 

    }

    const units::radians_per_second_t MAX_ANGULAR_VELOCITY = 2_rad_per_s*PI;
    const units::meters_per_second_t MAX_LINEAR_VELOCITY = 20.09_fps;
    units::meters_per_second_t vx,vy = 0_mps;
    units::radians_per_second_t theta = 0_rad_per_s;
    //wpi::array<frc::SwerveModuleState, 4U> SwerveStates; //should have default value, breaks code if not

    void Drive(){
        auto SwerveStates = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds{vx,vy,theta});

        FLState = SwerveStates[0];
        FRState = SwerveStates[1];
        RRState = SwerveStates[2];
        RLState = SwerveStates[3];

        m_kinematics.DesaturateWheelSpeeds(&SwerveStates, MAX_LINEAR_VELOCITY);
        
        //Optimizes based on the Encoder Position
        FLState = RLState.Optimize(FLState,units::degree_t((double)(FLSteer.GetSensorCollection().GetPulseWidthPosition()*(360/4096)))); //optimizes motion
        FRState = RLState.Optimize(FRState,units::degree_t((double)(FRSteer.GetSensorCollection().GetPulseWidthPosition()*(360/4096)))); //optimizes motion
        RRState = RRState.Optimize(RRState,units::degree_t((double)(RRSteer.GetSensorCollection().GetPulseWidthPosition()*(360/4096)))); //optimizes motion
        RLState = RLState.Optimize(RLState,units::degree_t((double)(RLSteer.GetSensorCollection().GetPulseWidthPosition()*(360/4096)))); //optimizes motion

        //Steer Motor Position Output
        FLSteer.Set(ControlMode::Position,(double)((-FLState.angle.Degrees()/360)*4096.0) + FLZero);
        FRSteer.Set(ControlMode::Position,(double)((-FRState.angle.Degrees()/360)*4096.0) + FRZero);
        RLSteer.Set(ControlMode::Position,(double)((-RLState.angle.Degrees()/360)*4096.0) + RLZero);
        RRSteer.Set(ControlMode::Position,(double)((-RRState.angle.Degrees()/360)*4096.0) + RRZero);
        //Drive Motor Power Output
        FLDrive.Set(ControlMode::PercentOutput,(double)(FLState.speed/MAX_LINEAR_VELOCITY));
        FRDrive.Set(ControlMode::PercentOutput,(double)(FRState.speed/MAX_LINEAR_VELOCITY));
        RLDrive.Set(ControlMode::PercentOutput,(double)(RLState.speed/MAX_LINEAR_VELOCITY));
        RRDrive.Set(ControlMode::PercentOutput,(double)(RRState.speed/MAX_LINEAR_VELOCITY));
        
        
    }

    void IntakeAndArm(){
        //Handels Intake Control
        if(IsAutonomousEnabled()){
            //`write code for autonomous
        }
        else{
            if(Dir_Stick.GetTrigger()) IntakeOn = 1;
            else if(Dir_Stick.GetRawButton(2)) IntakeOn = -1;
            else IntakeOn=0;
        }
        
        //`add control of Arm Position

        IntakeMotor.Set(ControlMode::PercentOutput,IntakeOn);
        ArmMotor.Set(ControlMode::Position,ArmPosition);
    }

    TalonFX RLDrive = {RLD};
    TalonFX FLDrive = {FLD};
    TalonFX FRDrive = {FRD};
    TalonFX RRDrive = {RRD};

    TalonSRX RLSteer = {RLS};
    TalonSRX RRSteer = {RRS};
    TalonSRX FRSteer = {FRS};
    TalonSRX FLSteer = {FLS};

    TalonSRX ArmMotor = {ARM_ID};
    TalonSRX IntakeMotor = {INTAKE_ID};

private:

    frc::Joystick Rot_Stick{ROT};
    frc::Joystick Dir_Stick{DIR};
    frc::Joystick Arm_Stick{ARM};

    //Locations for the swerve drive modules relative to the robot center.
    //21.75015472 in (0.55245393 m) wheel to wheel
    const frc::Translation2d m_frontLeftLocation{-10.875_in, 10.875_in};
    const frc::Translation2d m_frontRightLocation{10.875_in, 10.875_in};
    const frc::Translation2d m_backLeftLocation{-10.875_in, -10.875_in};
    const frc::Translation2d m_backRightLocation{10.875_in, -10.875_in};

    frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,m_backRightLocation};
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
    

    //Steer Zeros
    FLZero = frc::Preferences::GetDouble("FLZero",0);
    FRZero = frc::Preferences::GetDouble("FRZero",0);
    RLZero = frc::Preferences::GetDouble("RLZero",0);
    RRZero = frc::Preferences::GetDouble("RRZero",0);

    {//FLSteer
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
        //FLSteer.Set(ControlMode::Position, ClosestZero[FL]);
       // FLSteer.SetSelectedSensorPosition(0,0,TIMEOUT);
    }
    {//FRSteer    
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
        //FRSteer.Set(ControlMode::Position, ClosestZero[FR]);
        FRSteer.ConfigClosedloopRamp(.3,TIMEOUT);
    }
    {//RLSteer
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
        //RLSteer.Set(ControlMode::Position, ClosestZero[RL]);
        RLSteer.ConfigClosedloopRamp(.3,TIMEOUT);
        //RLSteer.SetNeutralMode(motorcontrol::NeutralMode::Coast); //`check if this is what we want 
    }
    {//RRSteer    
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
        //RRSteer.Set(ControlMode::Position, ClosestZero[RR]);
		RRSteer.ConfigClosedloopRamp(.3,TIMEOUT);
    }     

    {//RLDrive    
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
    }    
    {//RRDrive
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
    }
    {//FRDrive
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
    }
    {//FLDrive
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
    }
    
    ResetGyro();
    
    //Camera
	#ifdef CAMERA
    frc::CameraServer::StartAutomaticCapture();
    //frc::CameraServer::StartAutomaticCapture(1);
    
    #endif
    //Getting Alliance and location
    //Alliance = frc::DriverStation::GetAlliance();
    //StartingLocation = frc::DriverStation::GetAlliance()*10 + frc::DriverStation::GetLocation();
    //StartingLocation = 0; //frc::SmartDashboard::GetNumber("DB/Slider 0",-1);
    
  
}