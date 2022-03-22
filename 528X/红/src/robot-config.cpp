#include "vex.h"
using namespace vex;
using signature = vision::signature;
using code = vision::code;


vex::brain Brain;

vex::controller Controller1 = vex::controller();
vex::limit Limit1 = vex::limit(Brain.ThreeWirePort.A);
vex::motor M_left_1 = vex::motor(vex::PORT9,true);
vex::motor M_left_2 = vex::motor(vex::PORT19,true);
vex::motor M_right_1 = vex::motor(vex::PORT2,false);
vex::motor M_right_2 = vex::motor(vex::PORT16,false);
//For auto
vex::motor LeftMotor_f = vex::motor(vex::PORT2,true);
vex::motor LeftMotor_b = vex::motor(vex::PORT16,false);
vex::motor RightMotor_f = vex::motor(vex::PORT9,true);
vex::motor RightMotor_b = vex::motor(vex::PORT19,false);

motor_group leftdrive (M_left_1,M_left_2);
motor_group rightdrive(M_right_1,M_right_2);
motor_group ALL(M_left_1,M_left_2,M_right_1,M_right_2);



vex::motor M_intake_1= vex::motor(vex::PORT4);
vex::motor M_intake_2= vex::motor(vex::PORT18);


vex::motor M_put = vex::motor(vex::PORT1) ;
vex::motor M_up = vex::motor(vex::PORT8);

vex::gyro Gyro = vex::gyro(Brain.ThreeWirePort. D);
vex::gyro GyroA = vex::gyro(Brain.ThreeWirePort. A);

vex::bumper Bumper1 = vex::bumper(Brain.ThreeWirePort.B);


void set_Motor(vex::motor motor, int v)
{
    motor.setVelocity(v,vex::velocityUnits::rpm);
    motor.spin(vex::directionType::fwd);
}

void moveY(int v1,int sec)
{
  set_Motor(M_left_1,  v1);
  set_Motor(M_left_2,  -v1);
  set_Motor(M_right_1, -v1);
  set_Motor(M_right_2, v1);
       vex::task::sleep(sec);
  M_left_1.stop(vex::brakeType::coast);
  M_left_2.stop(vex::brakeType::coast);
  M_right_1.stop(vex::brakeType::coast);
  M_right_2.stop(vex::brakeType::coast);
}

/*int Liftadjust() 
{
  while (true) 
  {
    Controller1.ButtonX.pressed(adjustput);
  }
}*/
///////////////////////////////////////Handle Part////////////////////////////////////
          float vl;
          float vr;
          int vy;
          int turn2;
          

          int vy_max_speed =80;
          int turn2_max_speed = 80;//change the motor pwr HERE
          
          int sgn(double d)
          {
            if(d<0) return -1;
            else if(d==0) return 0;
            else return 1;
          }

void RT_move()
      {
        while(1) 
        {
          vy = Controller1.Axis3.value();
          turn2 = Controller1.Axis4.value();

        M_left_1.spin(vex::directionType::fwd, (Controller1.Axis3.value() - Controller1.Axis4.value()), vex::velocityUnits::pct);
        M_left_2.spin(vex::directionType::fwd, -(Controller1.Axis3.value() - Controller1.Axis4.value()), vex::velocityUnits::pct);
        M_right_1.spin(vex::directionType::fwd,-(Controller1.Axis3.value() + Controller1.Axis4.value()), vex::velocityUnits::pct);
        M_right_2.spin(vex::directionType::fwd,(Controller1.Axis3.value() + Controller1.Axis4.value()), vex::velocityUnits::pct);
        
        //monitor();
        intake();
        quicklyput();
        up();
        put();
        //task t1(Liftadjust);


        if(Controller1.ButtonDown.pressing()) break;
        
        }

        if(Controller1.ButtonDown.pressing())
        {
         while(1)
         {
         
         
          vy = Controller1.Axis3.value();
          turn2 = Controller1.Axis4.value();
          
          vy = vy * vy_max_speed/100;
          turn2 = turn2 * turn2_max_speed/100; 

          vl = -(vy - turn2);
          vr = (vy + turn2);

         
           if( fabs(vl)>=250 )
           vl= sgn(vl)*250;
           if( fabs(vr)>=250 )
           vr= sgn(vr)*250;//protect the vale explosion

          set_Motor(M_left_1, vl);
          set_Motor(M_left_2, vl);
          set_Motor(M_right_1,vr);
          set_Motor(M_right_2,vr);

            intake();
            quicklyput();
            up();
            put();
            //monitor();
            //task t1(Liftadjust);
            
        if(Controller1.ButtonUp.pressing()) break;
        
          }  
          
        }


      }
        //之
    void intake()
    {
        if(Controller1.ButtonR1.pressing())
        {
              set_Motor(M_intake_1,600);
              set_Motor(M_intake_2,-600);
        }
           
        else if(Controller1.ButtonR2.pressing())
        {
           
           set_Motor(M_intake_1,-600);
              set_Motor(M_intake_2,600);
        }
        
        else
        {
           set_Motor(M_intake_1,0);
              set_Motor(M_intake_2,0);
        }
    }
    
   
void put()
    {
      
    M_put.spin(vex::directionType::fwd,-(Controller1.Axis2.value()/3), vex::velocityUnits::pct);
    
    }


/*void adjustput() 
{

  M_put.spin(fwd, 5, volt);
  wait(200, msec);
  M_put.stop(hold);
}*/


void quicklyput()
{
  if(Controller1.ButtonY.pressing())
  {
   set_Motor(M_put,1000);
   vex::task::sleep(1000);
   M_put.stop(vex::brakeType::coast); 
  }
}
void U(int v1,int sec)
{
     set_Motor(M_up,v1);
   vex::task::sleep(sec);
        M_up.stop(vex::brakeType::coast);     
}
void P (int v1,int sec)
{
     set_Motor(M_put,-v1);
   vex::task::sleep(sec);
       M_put.stop(vex::brakeType::coast);     
     
}
void pp (int v1,int sec)
{
     set_Motor(M_put,-v1);
   vex::task::sleep(sec);
       M_put.stop(vex::brakeType::hold);     
     
}

  void up()
    {
        if(Controller1.ButtonA.pressing())
        {
         
         pp(100,600);
        
        }


        if(Controller1.ButtonX.pressing())
        {
         
         set_Motor(M_up,600);
            
        
        }
           
        else if(Controller1.ButtonB.pressing())
        {
           
           set_Motor(M_up,-600);
        }
        
        else
        {
           M_up .stop(vex::brakeType::hold);   
        }
    }
 
 void uptask(int v1,int sec)
{
     set_Motor(M_up,v1);
     vex::task::sleep(sec);
      M_put .stop(vex::brakeType::coast);   
}


void I(int v1,int sec)
{
   set_Motor(M_intake_1,-v1);
   set_Motor(M_intake_2,v1);
     vex::task::sleep(sec);
    M_intake_1 .stop(vex::brakeType::coast);
    M_intake_2.stop(vex::brakeType::coast);
}
void Forput(int v1,int sec)
{
     set_Motor(M_put,v1);
     vex::task::sleep(sec);
      M_put .stop(vex::brakeType::coast);
    
    
}
                                                                                             

void vexcodeInit( void ) {
  // nothing to initialize
}


/////////////Designed by LuX///////////////////////////////////////////

void Welcome()
{
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("黑爹 张国浩");
  //Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(2, 1);
  Controller1.Screen.print("Senior Operator");
  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("Welcome Mr.Zhang");
}



void MOVING(int aim,int PWR)
{
  //int PWRH=PWR/2;


  leftdrive.resetRotation();
  rightdrive.resetRotation();

  //M_left_1.startRotateTo(-aim/2, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  //M_left_2.startRotateTo(-aim/2, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  //M_right_1.startRotateTo(aim/2, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  //M_right_2.startRotateTo(aim/2, rotationUnits ::deg, PWR, velocityUnits ::rpm); 
  if(M_left_2.position(rotationUnits ::deg)>aim/2){
    PWR=PWR/2;
  }
  M_left_1.startSpinTo(-aim, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  M_left_2.startSpinTo(-aim, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  M_right_1.startSpinTo(aim, rotationUnits ::deg, PWR, velocityUnits ::rpm);
  M_right_2.startSpinTo(aim, rotationUnits ::deg, PWR, velocityUnits ::rpm);

  //Brain.Screen.print("valu %f",M_left_2.position(rotationUnits ::deg));

  //Brain.Screen.print("valu %d",M_right_2.position(rotationUnits ::deg));
}

//drivetrain robotDrive (leftdrive,rightdrive,12,16,16,distanceUnits::cm);

float tepm=0;
void monitor()
{
  //if(Controller1.ButtonRight.pressing())
 // {
     tepm=M_left_2.temperature(percentUnits::pct);
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(1, 1);
     Controller1.Screen.print("黑爹 张国浩");
     Controller1.Screen.setCursor(2, 1);
     Controller1.Screen.print("F:%d  L:%d",vy,turn2);
     Controller1.Screen.setCursor(3, 1);
     //Controller1.Screen.print("LM:%d",M_left_2.current(amp)/1000);
     Controller1.Screen.print("温度:%f",tepm);
  //}
 }

