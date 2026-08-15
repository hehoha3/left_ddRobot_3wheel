/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef L298_MOTOR_DRIVER
  #define RIGHT_MOTOR_BACKWARD 13
  #define LEFT_MOTOR_BACKWARD  12
  #define RIGHT_MOTOR_FORWARD  4
  #define LEFT_MOTOR_FORWARD   7
  #define RIGHT_MOTOR_ENABLE   9
  #define LEFT_MOTOR_ENABLE    11
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
