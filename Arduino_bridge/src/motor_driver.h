/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 100
  #define LEFT_MOTOR_BACKWARD  5
  #define RIGHT_MOTOR_FORWARD  100
  #define LEFT_MOTOR_FORWARD   4
  #define RIGHT_MOTOR_ENABLE 100
  #define LEFT_MOTOR_ENABLE 9
#endif

#ifndef USE_BASE
#define USE_BASE
#endif

#ifndef L298_MOTOR_DRIVER
#define L298_MOTOR_DRIVER
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
