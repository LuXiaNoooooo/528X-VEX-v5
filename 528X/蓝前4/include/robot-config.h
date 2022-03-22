

using namespace vex;

extern brain Brain;
extern controller Controller1;
extern limit Limit1;
extern motor M_left_1;
extern motor M_left_2;
extern motor M_right_1;
extern motor M_right_2;

extern motor M_intake_1;
extern motor M_intake_2;
extern motor M_put;
extern motor M_up;
extern gyro Gyro;
extern bumper Bumper1;
//For auto
extern motor LeftMotor_f;
extern motor LeftMotor_b;
extern motor RightMotor_f;
extern motor RightMotor_b;

extern void run(vex::motor motor, int per);
extern void set_Motor(vex::motor motor, int v);
extern void gogogo( int v1,int sec);
extern void turn_r(int out);
extern void turn (int v1, int sec,int brake);
extern void youzhuan (int aim, int output,int brake);
extern void go (int v1);
extern void reset_rot ();
extern void distance (double travelTargetCM, int V);
extern void distance_zhi (double travelTargetCM, int V);
extern void distance_zhi_2 (double travelTargetCM, int V);
extern void distance2(double travelTargetCM, int V);
extern void distance_pid (double travelTargetCM,int max,int min);
extern void distance_pid_2 (double travelTargetCM,int max,int min);
extern void RT_move();
extern void intake();
extern void put();
extern void U(int v1,int sec);
extern void P (int v1,int sec);
extern void pp (int v1,int sec);
extern void up();
void uptask(int v1,int sec);
extern void I(int v1,int sec);
extern void turn_pid_abs(int aim ,int max, int min,int brake,float kp,int ba);
extern  void GYRO_TEST();
extern void Forput(int v1,int sec);
extern void quicklyput();
void moveY(int v1,int sec);

//extern void adjustput();



extern void Welcome();
extern void monitor();

extern void MOVING(int aim,int PWR);

// VEXcode devices

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );