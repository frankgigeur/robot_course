#include <Arduino.h>
#include <math.h>
#include <LibRobus.h>

#define TOUR_CM 23.9389360203542244771
#define PULSE_PER_TURN 3200
#define HALF_CIRCLE_IN_DEG 180
#define WHEELS_RADIUS 9.25
#define PATH_OFFSET 22.5
#define M_GAUCHE 0
#define M_DROITE 1
#define FRONT_BUMPER 26
#define LEFT_BUMPER 27
#define BACK_BUMPER 28
#define RIGHT_BUMPER 29

#define MAX_INDEX 10 // NEED TO BE SETTED

bool run = false;
float distance[10] = {24.0 + 22 , 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
float angle[9] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0};
long leftEncoder = 0;
long rightEncoder = 0;
unsigned int index = 0;
const float kp = 0.0085;
const float ki = 1;
const float kd = 1;

typedef enum t_mode
{
  A,
  D
} e_mode;

e_mode mode = D;

void moveAlgo();
void motors(char left_motor, char right_motor);
float motorsPid();
void motorsOff();
void encodersReset();
void setEncodersVars();

void setup()
{
  Serial.begin(9600);

  // ENLEVE UN OFFSET À LA DISTANCE POUR CENTRER LE ROBOT AU PARCOURS
  for(int i = 0 ; i < MAX_INDEX ; i++)
  {
    float tmp = 0;
    tmp = tmp - PATH_OFFSET;
  }

  // TRANSFORME LES ANGLES EN PULSE D'ENCODEUR
  for(int i = 0 ; i < (MAX_INDEX - 1) ; i++)
  {
    float tmp = 0;
    tmp = (angle[i] * PI * WHEELS_RADIUS * PULSE_PER_TURN)/(HALF_CIRCLE_IN_DEG * TOUR_CM);
    angle[i] = tmp;
  }
  
  BoardInit();
}

void loop()
{

  if (digitalRead(RIGHT_BUMPER))
  {
    run = false;
  }
  if (digitalRead(BACK_BUMPER))
  {
    encodersReset();
    run = true;
  }
  else if (run)
  {
    moveAlgo();
  }
  else
  {
    motorsOff();
  }

}

void moveAlgo()
{
  setEncodersVars();

  switch (mode)
  {

  case A:

    if(angle[index] > leftEncoder)
    {

    }

    break;

  case D:
    
    if(distance[index] > (leftEncoder == 0 ? 0 : ((leftEncoder/3200) * TOUR_CM)))
    {
      motors(1.0,1.0);
    }
    else
    {      
      motorsOff();
      encodersReset();
      delay(50);
      mode = A;      
    }
    break;

  default:
    Serial.println("ERROR NO MODE!");
    break;
  }
}

void motors(char left_motor, char right_motor)
{

  float speed = motorsPid();

  // char str[50];

  // sprintf(str, "\nG : %ld\nD : %ld",ENCODER_Read(M_GAUCHE),ENCODER_Read(M_DROITE));
  // Serial.print(str);

  MOTOR_SetSpeed(M_GAUCHE, 0. * left_motor);
  MOTOR_SetSpeed(M_DROITE, 0.2 * right_motor + (speed * right_motor));
}

float motorsPid()
{

  long error = abs(leftEncoder) - abs(rightEncoder);
  return (error * kp);

}

void motorsOff()
{
  MOTOR_SetSpeed(M_GAUCHE, 0.0);
  MOTOR_SetSpeed(M_DROITE, 0.0);
}

void encodersReset()
{
  ENCODER_Reset(M_GAUCHE);
  ENCODER_Reset(M_DROITE);
}

void setEncodersVars()
{
  leftEncoder = ENCODER_Read(M_GAUCHE);
  rightEncoder = ENCODER_Read(M_DROITE);
}