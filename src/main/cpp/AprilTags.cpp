/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/apriltag/AprilTag.h>


#include <frc/TimedRobot.h>
#include <frc/Timer.h>

#include <frc/smartdashboard/SmartDashboard.h>
//#include "ahrs.h" //https://www.kauailabs.com/public_files/navx-mxp/apidocs/c++/


/*#include "frc/DriverStation.h"
#ifdef CAMERA
#include <cameraserver/CameraServer.h>
#endif*/


#define TIMEOUT 10
#define NOTIMEOUT 0
#define TRUE 1
#define FALSE 0

int Initialized = 0;

//phematic motor
int RotaryVal = 0;
double TimeVal = 0.25;
int TankFull = 0;

//Joystick defines
#define DIR 0
#define ROT 1
#define ARM 2



#define PI 3.141592654
#define ENCODER_RES_FL 4096 //4179.0 //4156.0 //4180.0
#define ENCODER_RES_RL 4096 //4152.0 //4185.0 //4180.0
#define ENCODER_RES_FR 4096//4154.0 //4076.0 //4180.0
#define ENCODER_RES_RR 4096 //4162.0 //4175.0 //4180.0

#define PVALUE 0.15
#define DVALUE 0.0
#define IVALUE 0.0



int UpdateCount=0;    //counter to slow screen update so it does not take too many CPU cycles
double RobotAngle=0.0;
double RobotPitch=0.0;
double GyroOffset=0.0;
int EncoderAct = 0;        //storage for actual angles of wheel from motor controller encoder feedback

                    //local variable for determining best way(least delta degrees) to meet rotatioanl target
//AHRS *ahrs;  //gyro

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
   
           //Code for displaying swerve values//frc::SmartDashboard::PutString("DB/String 0",str);
          //sprintf(str,"PID:%4.2f",PID.Calculate((ahrs->GetRoll()-GyroOffset)/90, 0));
         frc::SmartDashboard::PutString("DB/String 1",str);
          sprintf(str,"GyroPitch:%4.2f",RobotPitch);
         frc::SmartDashboard::PutString("DB/String 2",str);
		   
          
		 //  frc::SmartDashboard::PutNumber("DB/LED 0",light);
          // frc::SmartDashboard::PutNumber("DB/LED 1",Shooter_On);

        }
        gUpdateCount--;
    } // End UpdateDriverScreen
	
     

    void ReadGyro(void) {
		//RobotAngle = ahrs->GetYaw();
        
        //RobotPitch = ahrs->GetRoll()-GyroOffset;
	}	// End ReadyGyro
	void ResetGyro(void) {
		//ahrs->ZeroYaw();
        
        //GyroOffset=ahrs->GetRoll();
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

      
        ReadGyro(); 

        UpdateDriverScreen(); 

    }

private:

   // frc::DigitalInput ElevatorBottom{1};      
    

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
    
    

    
  
}