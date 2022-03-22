#include "vex.h"
#include "robot-config.h"
#include <iostream>
using namespace std;
void ResetRotation()
{
LeftMotor_f.spin(fwd,0,rpm);
LeftMotor_b.spin(fwd,0,rpm);
RightMotor_f.spin(reverse,0,rpm);
RightMotor_b.spin(reverse,0,rpm);
wait(20,msec);
LeftMotor_f.resetRotation();
RightMotor_f.resetRotation();
LeftMotor_b.resetRotation();
RightMotor_b.resetRotation();
LeftMotor_f.setRotation(0,deg);
RightMotor_f.setRotation(0,deg);
RightMotor_b.setRotation(0,deg);
LeftMotor_f.setRotation(0,deg);

}
struct PID {
  double currentP;
  double aimP;
  double integral;
  double pgain;
  double igain;
  double dgain;
  double deadband;
  double last_error;
  double last_vel;
};

void pid_setpoint(struct PID *PIDController, double process_point,
                  double set_point) {
  PIDController->aimP = set_point;
  PIDController->currentP = process_point;
}

void pid_set(struct PID *PIDController, double p_gain, double i_gain,
             double d_gain, double integral_vel, double deadband) {
  PIDController->pgain = p_gain;
  PIDController->igain = i_gain;
  PIDController->dgain = d_gain;
  PIDController->integral = integral_vel;
  PIDController->deadband = deadband;
  PIDController->last_error = 0;
  PIDController->last_vel = 0;
}

double pid_calc(struct PID *PIDController) {
  double error;
  double pitem, ditem;
  double aimVelocity = 0;
  double pre_vel = PIDController->last_vel;
  double acc = 0;
  error = PIDController->aimP - PIDController->currentP;

  if (fabs(error) > PIDController->deadband) {

    pitem = PIDController->pgain * error;

    if (pitem > 100 || pitem < -100) {
      PIDController->integral = 0;
    } else {
      PIDController->integral += PIDController->igain * error;
    }
    ditem = PIDController->dgain * (error - PIDController->last_error);

    aimVelocity = pitem + ditem + PIDController->integral;
    acc = aimVelocity - pre_vel;

    if ((aimVelocity - pre_vel) > 25) {
      aimVelocity = pre_vel + 24.5;
    } else if ((aimVelocity - pre_vel) < -25) {
      aimVelocity = pre_vel - 24.5;
    }
  } else {
    aimVelocity = PIDController->integral;
  }
  PIDController->last_vel = aimVelocity;
  PIDController->last_error = error;
  Brain.Screen.printAt(10, 140, "acc %8.2f", acc);
  Brain.Screen.printAt(10, 160, "error %8.2f", error);
  Brain.Screen.printAt(10, 180, "aimVelocity %8.2f", aimVelocity);
  return aimVelocity;
}

void Move_Stop(void) {
  RightMotor_b.stop(vex::brakeType::brake);
  RightMotor_f.stop(vex::brakeType::brake);
  LeftMotor_b.stop(vex::brakeType::brake);
  LeftMotor_f.stop(vex::brakeType::brake);
}

void Turning(double Turn_Angle) {
  double wheelDiameterCM = 8.225;
  double aim_p;
  double k;
  double circumference = wheelDiameterCM * 3.141592653;
  if (Turn_Angle>0)
    aim_p= ((Turn_Angle) * circumference) / 90;
    k=1.0;
  if(Turn_Angle<10)
    aim_p= ((Turn_Angle-12.5) * circumference) / 90;
    k=-1.0;
  double degreesToRotate = (360 * aim_p) / circumference;
  double rightEncoder1Offset;
  double rightEncoder2Offset;
  double leftEncoder1Offset;
  double leftEncoder2Offset;
  double rsetVel_1;
  double lsetVel_1;
  double rsetVel_2;
  double lsetVel_2;

  struct PID right1VelController;
  struct PID right2VelController;
  struct PID left1VelController;
  struct PID left2VelController;

  pid_set(&right1VelController, 0.19, 0, 0.6, 0, 1);
  pid_set(&left1VelController, 0.19, 0, 0.6, 0, 1);
  pid_set(&left2VelController, 0.19, 0, 0.6, 0, 1);
  pid_set(&right2VelController, 0.19, 0, 0.6, 0, 1);
  rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg) * k;
  rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg) * k;
  leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg) * k;
  leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg) * k;
  double LeftdegreesToRotate = leftEncoder1Offset - degreesToRotate;
  double RightdegreesToRotate = rightEncoder1Offset + degreesToRotate;
  while (fabs(LeftdegreesToRotate -leftEncoder1Offset) > 3.5) {
    pid_setpoint(&right1VelController, rightEncoder1Offset,
                 RightdegreesToRotate);
    pid_setpoint(&right2VelController, rightEncoder2Offset,
                 RightdegreesToRotate);
    pid_setpoint(&left1VelController, leftEncoder1Offset, LeftdegreesToRotate);
    pid_setpoint(&left2VelController, leftEncoder2Offset, LeftdegreesToRotate);

    rsetVel_1 = pid_calc(&right1VelController);
    rsetVel_2 = pid_calc(&right2VelController);
    lsetVel_1 = pid_calc(&left1VelController);
    lsetVel_2 = pid_calc(&left2VelController);

    RightMotor_b.spin(vex::directionType::fwd, rsetVel_1*4,
                      rpm);
    RightMotor_f.spin(vex::directionType::rev, rsetVel_2*4,
                      rpm);
    LeftMotor_b.spin(vex::directionType::rev, lsetVel_1*4,
                     rpm);
    LeftMotor_f.spin(vex::directionType::fwd, lsetVel_2*4,
                     rpm);

    rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg);
    rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg);
    leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg);
    leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg);


    //Brain.Screen.printAt(100, 200, "Encode value: %f", EncoderG.rotation(deg));
    Brain.Screen.printAt(100, 240, "While: %.2f",
                         fabs(LeftdegreesToRotate - leftEncoder2Offset));
  }
  ResetRotation();
  Move_Stop();
}










class PID_position {
private:
  float kp;       
  float ki;       
  float kd;       
  float target;   
  float actual;   
  float e;       
  float e_pre;    
  float integral; 
public:
  PID_position();
  ~PID_position(){};
  PID_position(float p, float i, float d);
  float pid_control(float tar, float act); 
  void pid_show();                         
};


PID_position::PID_position()
    : kp(0), ki(0), kd(0), target(0), actual(0), integral(0) {
  e = target - actual;
  e_pre = e;
}
PID_position::PID_position(float p, float i, float d)
    : kp(p), ki(i), kd(d), target(0), actual(0), integral(0) {
  e = target - actual;
  e_pre = e;
}
float PID_position::pid_control(float tar, float act) {
  float u;
  target = tar;
  actual = act;
  e = target - actual;
  integral += e;
  u = kp * e + ki * integral + kd * (e - e_pre);
  e_pre = e;
  return u;
}
void PID_position::pid_show() {
  using std::cout;
  using std::endl;
  cout << "The infomation of this position PID controller is as following:"
       << endl;
  cout << "       Kp=" << kp << endl;
  cout << "       Ki=" << ki << endl;
  cout << "       Kd=" << kd << endl;
  cout << " integral=" << integral << endl;
  cout << "   target=" << target << endl;
  cout << "   actual=" << actual << endl;
  cout << "        e=" << e << endl;
  cout << "    e_pre=" << e_pre << endl;
}

class PID_incremental {
private:
  float kp;
  float ki;
  float kd;
  float target;
  float actual;
  float e;
  float e_pre_1;
  float e_pre_2;
  float A;
  float B;
  float C;

public:
  PID_incremental();
  PID_incremental(float p, float i, float d);
  float pid_control(float tar, float act);
  void pid_show();
};

PID_incremental::PID_incremental()
    : kp(0), ki(0), kd(0), e_pre_1(0), e_pre_2(0), target(0), actual(0) {
  A = kp + ki + kd;
  B = -2 * kd - kp;
  C = kd;
  e = target - actual;
}
PID_incremental::PID_incremental(float p, float i, float d)
    : kp(p), ki(i), kd(d), e_pre_1(0), e_pre_2(0), target(0), actual(0) {
  A = kp + ki + kd;
  B = -2 * kd - kp;
  C = kd;
  e = target - actual;
}
float PID_incremental::pid_control(float tar, float act) {
  float u_increment;
  target = tar;
  actual = act;
  e = target - actual;
  u_increment = A * e + B * e_pre_1 + C * e_pre_2;
  e_pre_2 = e_pre_1;
  e_pre_1 = e;
  return u_increment;
}

void PID_incremental::pid_show() {
  using std::cout;
  using std::endl;
  cout << "The infomation of this incremental PID controller is as following:"
       << endl;
  cout << "     Kp=" << kp << endl;
  cout << "     Ki=" << ki << endl;
  cout << "     Kd=" << kd << endl;
  cout << " target=" << target << endl;
  cout << " actual=" << actual << endl;
  cout << "      e=" << e << endl;
  cout << "e_pre_1=" << e_pre_1 << endl;
  cout << "e_pre_2=" << e_pre_2 << endl;
}

void Brake() {
  RightMotor_b.stop(vex::brakeType::brake);
  RightMotor_f.stop(vex::brakeType::brake);
  LeftMotor_b.stop(vex::brakeType::brake);
  LeftMotor_f.stop(vex::brakeType::brake);
  wait(50,msec);
  RightMotor_b.stop(vex::brakeType::coast);
  RightMotor_f.stop(vex::brakeType::coast);
  LeftMotor_b.stop(vex::brakeType::coast);
  LeftMotor_f.stop(vex::brakeType::coast);
}


double run(double Run_Pos) {
double wheelDiameterCM = 8.255;
  double circumference = wheelDiameterCM * 3.1415926;
  double aim_p = Run_Pos;
  double degreesToRotate = (360 * aim_p) / circumference;
  double rightEncoder1Offset;
  double rightEncoder2Offset;
  double leftEncoder1Offset;
  double leftEncoder2Offset;
  rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg);
  rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg);
  leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg);
  leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg);
  double RightdegreesToRotate =  rightEncoder2Offset + degreesToRotate;
  double leftdegreesToRotate  =  leftEncoder2Offset + degreesToRotate;
  double CurrentSpeed = 0;
  double dis =  rightEncoder2Offset +leftEncoder2Offset;
  double leftvel, rightvel;

  double Left = 0, Right = 0;
  double SpeedRatio = 0;
  PID_position pid(0.59, 0.35, 0.001);
  PID_position Pidl(0.59, 0.35, 0.001);
  PID_position Pidr(0.59, 0.35, 0.001);

  double aimL, aimR;
  while (SpeedRatio < 1) // too fast set PID
  {

    dis =  rightEncoder2Offset +leftEncoder2Offset;
    SpeedRatio = dis / (leftdegreesToRotate + RightdegreesToRotate);
    Left =   leftEncoder2Offset;
    Right =  rightEncoder2Offset;
    if (SpeedRatio < 0.05)
      CurrentSpeed = 6;

    else if (SpeedRatio < 0.2) {

      CurrentSpeed = 4;
    } else if (SpeedRatio < 0.25)
      CurrentSpeed = 4;
    else if (SpeedRatio < 0.3)
      CurrentSpeed = 5;
    else if (SpeedRatio < 0.8)
      CurrentSpeed = 6;
    else if (SpeedRatio < 0.85)
      CurrentSpeed = 7;
    else if (SpeedRatio < 0.89)
      CurrentSpeed = 6;
    else if (SpeedRatio < 0.921)

      CurrentSpeed = 4;
    else if (SpeedRatio < 0.93)
      CurrentSpeed =4;
    else if (SpeedRatio < 0.95)
      CurrentSpeed = 3.5;

    else if (SpeedRatio < 0.99)
      CurrentSpeed = 3;

    leftvel = rightvel = CurrentSpeed;
    aimL =CurrentSpeed;
  
    aimR = CurrentSpeed+0.01;
    if (Left > Right) {
      leftvel = Pidl.pid_control(aimL, leftvel);
      rightvel = CurrentSpeed;
    } else if (Right < Left) {
      rightvel = Pidr.pid_control(aimR, rightvel);
      leftvel = CurrentSpeed;
    }
   
    RightMotor_b.spin(reverse, rightvel , volt);
    RightMotor_f.spin(vex::directionType::fwd, rightvel, volt);
    LeftMotor_f.spin(vex::directionType::rev, leftvel, volt);
    LeftMotor_b.spin(vex::directionType::fwd, leftvel, volt);
    
    rightEncoder1Offset = -RightMotor_b.rotation(vex::rotationUnits::deg);
    rightEncoder2Offset = RightMotor_f.rotation(vex::rotationUnits::deg);
    leftEncoder1Offset = LeftMotor_b.rotation(vex::rotationUnits::deg);
    leftEncoder2Offset = -LeftMotor_f.rotation(vex::rotationUnits::deg);
  }

  Brake();
  ResetRotation();
  return leftdegreesToRotate-leftEncoder2Offset;
}


double run_for_take(double Run_Pos) {
double wheelDiameterCM = 8.255;
  double circumference = wheelDiameterCM * 3.1415926;
  double aim_p = Run_Pos;
  double degreesToRotate = (360 * aim_p) / circumference;
  double rightEncoder1Offset;
  double rightEncoder2Offset;
  double leftEncoder1Offset;
  double leftEncoder2Offset;
  rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg);
  rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg);
  leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg);
  leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg);
  double RightdegreesToRotate =  rightEncoder2Offset + degreesToRotate;
  double leftdegreesToRotate  =  leftEncoder2Offset + degreesToRotate;
  double CurrentSpeed = 0;
  double dis =  rightEncoder2Offset +leftEncoder2Offset;
  double leftvel, rightvel;

  double Left = 0, Right = 0;
  double SpeedRatio = 0;
  PID_position pid(0.55, 0.35, 0.001);
  PID_position Pidl(0.55, 0.35, 0.001);
  PID_position Pidr(0.55, 0.35, 0.001);

  double aimL, aimR;
  while (SpeedRatio < 1) // too fast set PID
  {

    dis =  rightEncoder2Offset +leftEncoder2Offset;
    SpeedRatio = dis / (leftdegreesToRotate + RightdegreesToRotate);
    Left =   leftEncoder2Offset;
    Right =  rightEncoder2Offset;
    if (SpeedRatio < 0.05)
      CurrentSpeed = 12;

    else if (SpeedRatio < 0.15) {

      CurrentSpeed = 12;
    } else if (SpeedRatio < 0.20)
      CurrentSpeed = 6;
    else if (SpeedRatio < 0.3)
      CurrentSpeed = 5;
    else if (SpeedRatio < 0.8)
      CurrentSpeed = 5;
    else if (SpeedRatio < 0.85)
      CurrentSpeed = 4;
    else if (SpeedRatio < 0.89)
      CurrentSpeed = 4;
    else if (SpeedRatio < 0.921)

      CurrentSpeed = 4;
    else if (SpeedRatio < 0.93)
      CurrentSpeed = 4;
    else if (SpeedRatio < 0.95)
      CurrentSpeed = 4.5;

    else if (SpeedRatio < 0.99)
      CurrentSpeed = 3;

    leftvel = rightvel = CurrentSpeed;
    aimL =CurrentSpeed;
  
    aimR = CurrentSpeed+0.01;
    if (Left > Right) {
      leftvel = Pidl.pid_control(aimL, leftvel);
      rightvel = CurrentSpeed;
    } else if (Right < Left) {
      rightvel = Pidr.pid_control(aimR, rightvel);
      leftvel = CurrentSpeed;
    }
   
    RightMotor_b.spin(reverse, rightvel , volt);
    RightMotor_f.spin(vex::directionType::fwd, rightvel, volt);
    LeftMotor_f.spin(vex::directionType::rev, leftvel, volt);
    LeftMotor_b.spin(vex::directionType::fwd, leftvel, volt);
    
    rightEncoder1Offset = -RightMotor_b.rotation(vex::rotationUnits::deg);
    rightEncoder2Offset = RightMotor_f.rotation(vex::rotationUnits::deg);
    leftEncoder1Offset = LeftMotor_b.rotation(vex::rotationUnits::deg);
    leftEncoder2Offset = -LeftMotor_f.rotation(vex::rotationUnits::deg);
  }

  Brake();
  ResetRotation();
  return leftdegreesToRotate-leftEncoder2Offset;
}







double run_back(double Run_Pos) {
  double wheelDiameterCM = 8.255;
  double circumference = wheelDiameterCM * 3.1415926;
  double aim_p = Run_Pos;
  double degreesToRotate = (360 * aim_p) / circumference;
  double rightEncoder1Offset;
  double rightEncoder2Offset;
  double leftEncoder1Offset;
  double leftEncoder2Offset;
  rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg);
  rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg);
  leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg);
  leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg);
  double RightdegreesToRotate =  rightEncoder2Offset + degreesToRotate;
  double leftdegreesToRotate  =  leftEncoder2Offset + degreesToRotate;
  double CurrentSpeed = 0;
  double dis =  rightEncoder2Offset +leftEncoder2Offset;
  double leftvel, rightvel;

  double Left = 0, Right = 0;
  double SpeedRatio = 0;
  PID_position pid(0.59, 0.35, 0.001);
  PID_position Pidl(0.59, 0.35, 0.001);
  PID_position Pidr(0.59, 0.35, 0.001);

  double aimL, aimR;
  while (SpeedRatio < 1) // too fast set PID
  {

    dis =  rightEncoder2Offset +leftEncoder2Offset;
    SpeedRatio = dis / (leftdegreesToRotate + RightdegreesToRotate);
    Left =   leftEncoder2Offset;
    Right =  rightEncoder2Offset;
    if (SpeedRatio < 0.05)
      CurrentSpeed = 6;

    else if (SpeedRatio < 0.2) {

      CurrentSpeed = 7;
    } else if (SpeedRatio < 0.25)
      CurrentSpeed = 10;
    else if (SpeedRatio < 0.3)
      CurrentSpeed = 11;
    else if (SpeedRatio < 0.8)
      CurrentSpeed = 20;
    else if (SpeedRatio < 0.85)
      CurrentSpeed = 15;
    else if (SpeedRatio < 0.89)
      CurrentSpeed = 10;
    else if (SpeedRatio < 0.921)

      CurrentSpeed = 6;
    else if (SpeedRatio < 0.93)
      CurrentSpeed =5;
    else if (SpeedRatio < 0.95)
      CurrentSpeed = 4.5;

    else if (SpeedRatio < 0.99)
      CurrentSpeed = 3;

    leftvel = rightvel = CurrentSpeed;
    aimL =CurrentSpeed+0.00001;
    
    aimR = CurrentSpeed+0.00001;
    if (Left > Right) {
      leftvel = Pidl.pid_control(aimL, leftvel);
      rightvel = CurrentSpeed;
    } else if (Right < Left) {
      rightvel = Pidr.pid_control(aimR, rightvel);
      leftvel = CurrentSpeed;
    }

    RightMotor_b.spin(fwd, rightvel , volt);
    RightMotor_f.spin(vex::directionType::rev, rightvel, volt);
    LeftMotor_f.spin(vex::directionType::fwd, leftvel, volt);
    LeftMotor_b.spin(vex::directionType::rev, leftvel, volt);
    //Brain.Screen.printAt(120, 20, "%lf", LeftMotor_b.velocity(rpm));
    /////Brain.Screen.printAt(120, 40, "%lf", RightMotor_b.velocity(rpm));
    ////////////Brain.Screen.printAt(120, 60, "%lf", SpeedRatio);
    rightEncoder1Offset = RightMotor_b.rotation(vex::rotationUnits::deg);
    rightEncoder2Offset = -RightMotor_f.rotation(vex::rotationUnits::deg);
    leftEncoder1Offset = -LeftMotor_b.rotation(vex::rotationUnits::deg);
    leftEncoder2Offset = LeftMotor_f.rotation(vex::rotationUnits::deg);
  }

  Brake();
  ResetRotation();
   return leftdegreesToRotate-leftEncoder2Offset;
}