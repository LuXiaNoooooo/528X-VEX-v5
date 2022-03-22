#include "pid.h"
#include "vex.h"

using namespace vex;

#include <string>      
#include <stdio.h>
#include <math.h>
#include "vex_timer.h"
#include <iostream>
using namespace vex;

vex::competition    Competition;



// left hand side of the robot has two motors
motor_group   leftDrive( M_left_1, M_right_1 );

// right hand side of the robot has two motors
motor_group   rightDrive( M_left_2, M_right_2 );

//smartdrive    robotDrive( leftDrive, rightDrive, Gyro, 12.56, 16, 16, distanceUnits::in );
drivetrain    robotDrive( leftDrive, rightDrive, 12.56, 6.3, 6.3, distanceUnits::in );



/*int Liftadjust() 
{
  while (true) 
  {
    Controller1.ButtonX.pressed(adjustput);
  }
}
*/

void autonomous( void ) 
{ 
  //Test
  //Welcome();
  //run_for_take(50);
  //run_back(50);
 // robotDrive.turnFor(90, degrees); //90度
  //moveY(100,300);//延时前进，回正用


//moveY(75, 500);
//run_for_take(20);

uptask(200, 500);
wait(500, timeUnits ::msec);
uptask(-190,500);
wait(500, timeUnits ::msec);
Forput(180,1000);
//延展


  set_Motor(M_intake_1,190);
  set_Motor(M_intake_2,-190);
  run_for_take(100);
  //run(85);
  //第二组

  run_back(48);
  robotDrive.turnFor(122,degrees);//
  ResetRotation();
 //向得分区进发
  
  set_Motor(M_intake_1,200);
  set_Motor(M_intake_2,-200);
  wait(400, timeUnits ::msec);
  set_Motor(M_intake_1,30);
  set_Motor(M_intake_2,-30);
 
  //run_back(25);
  //wait(500, timeUnits ::msec);
  run(20);
  //run_for_take(20);
  //moveY(160, 750);
 // wait(600, timeUnits ::msec);


  //run_back(1.5);//放
  wait(200, timeUnits ::msec);
  set_Motor(M_intake_1,-135);
  set_Motor(M_intake_2,135);//140
  wait(622, timeUnits ::msec);
  //呕吐
    set_Motor(M_intake_1,40);
    set_Motor(M_intake_2,-40);
    wait(500, timeUnits ::msec);
    set_Motor(M_intake_1,0);
    set_Motor(M_intake_2,0);
  //小吸一口

  Forput(-110, 800);
  wait(300, timeUnits ::msec);//立
  
  set_Motor(M_up, -40);
  set_Motor(M_intake_1, 50);
  set_Motor(M_intake_2,-50);
  //吸稳
  Forput(-155,245);
  wait(230, timeUnits ::msec);

  
  //Forput(-150, 300);
  set_Motor(M_up, 0);
  set_Motor(M_intake_1,  -80);
  set_Motor(M_intake_2,   80);
  //run_back(10);
  wait(320, timeUnits ::msec);
 // wait(1000, timeUnits ::msec);
 // ResetRotation();
 // run_back(50);
  moveY(-200, 800);
  //呕吐退出
  

}

void joystick1()
{ 
  //Welcome(); 
  up();
  put(); 
  RT_move();
  
  intake();
  quicklyput();
  
}

void usercontrol( void ) 
{
  // User control code here, inside the loop
    while (1) 
    {
  
      joystick1();
   
       
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}




int displayTask() {
    while(1) {
      // display some useful info
      Brain.Screen.setCursor(2,1);
      Brain.Screen.print( "  MotorLb    speed: %4.0f   position: %6.2f", M_left_1.velocity( percent ), M_left_1.position( rev ) );
      Brain.Screen.newLine();
      Brain.Screen.print( "  MotorLf    speed: %4.0f   position: %6.2f", M_left_2.velocity( percent ), M_left_2.position( rev ));
      Brain.Screen.newLine();
      Brain.Screen.print( "  MotorRb    speed: %4.0f   position: %6.2f", M_right_1.velocity( percent ), M_right_1.position( rev ));
      Brain.Screen.newLine();
      Brain.Screen.print( "  MotorRf    speed: %4.0f   position: %6.2f", M_right_2.velocity( percent ), M_right_2.position( rev ));
      Brain.Screen.newLine();
      Brain.Screen.newLine();

      // motor group velocity and position is returned for the first motor in the group
      Brain.Screen.print( "  leftDrive  speed: %4.0f   position: %6.2f", leftDrive.velocity( percent ), leftDrive.position( rev ));
      Brain.Screen.newLine();
      Brain.Screen.print( "  rightDrive speed: %4.0f   position: %6.2f", rightDrive.velocity( percent ), rightDrive.position( rev ));
      Brain.Screen.newLine();
      Brain.Screen.newLine();

      // drivetrain velocity is the average of the motor velocities for left and right
      Brain.Screen.print( "  robotDrive speed: %4.0f", robotDrive.velocity( percent ) );
      Brain.Screen.newLine();

      // no need to run this loop too quickly
      wait( 20, timeUnits::msec );
    }

    return 0;
}


int main() {
    
    

 // Initializing Robot Configuration. DO NOT REMOVE!
    vexcodeInit();

    // set 10 second timeout for robot drive movements
    robotDrive.setTimeout(10, seconds);

    // set the speed used for drive turns
    robotDrive.setTurnVelocity(50, percent);

    // start the display task
    task displayTaskInstance( displayTask );
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
                 
    while(1) 
    {
        vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       




}