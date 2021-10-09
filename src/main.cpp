/**
 * @file main.cpp
 * @author Ro13bots with attitUdeS
 * @brief Code for a robot moving from point A to point B by making angles
 * @version 0.8
 * @date 2021-10-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Arduino.h>
#include <math.h>
#include <LibRobus.h>

#define TOUR_CM 23.939
#define PULSE_PER_TURN 3200
#define HALF_CIRCLE_IN_DEG 180
#define WHEELS_RADIUS 9.3
#define PATH_OFFSET 45
#define OFFSET_START_STOP 10
#define M_GAUCHE 0
#define M_DROITE 1
#define FRONT_BUMPER 26
#define LEFT_BUMPER 27
#define BACK_BUMPER 28
#define RIGHT_BUMPER 29
#define MAX_DELTA_STEPS 2000
#define SLOPE 0.000375
#define SPEED_OFFSET 0.15
#define ANGLE_SPEED 0.4

#define MAX_INDEX 14 // NEED TO BE SETTED

bool run = false;
float distance[15] = {100.5, 49.5, 47.5, 49.5, 32.25, 52, 54.5, 54.5, 52, 32.25, 49.5, 47.5, 49.5, 100.5};
float angle[14] = {90.0, -90.0, -90.0, 90.0, 45.0, -45.0, 185.0, 45.0, -45.0, -90.0, 90, 90, -90};
long leftEncoder = 0;
long rightEncoder = 0;
unsigned int index = 0;
const float kp = 0.0025;
const float ki = 1;
const float kd = 1;
char reverse = 1;

typedef enum t_mode
{
  A,
  D
} e_mode;

e_mode mode = D;

/**
 * @brief Movement algorithm
 * 
 */
void moveAlgo();

/**
 * @brief 
 * 
 * @param left_motor Reverse(-1) , Stop(0) , Forward(1) for the left motor.
 * @param right_motor Reverse(-1) , Stop(0) , Forward(1) for the right motor.
 * @param speed Set the initial speed for both motors
 */
void motors(char left_motor, char right_motor, float speed);

/**
 * @brief 
 * 
 * @return float The correction factor
 */
float motorsPid();

/**
 * @brief Turn off all motors
 * 
 */
void motorsOff();

/**
 * @brief Reset all encoders
 * 
 */
void encodersReset();

/**
 * @brief Set the Encoders Variables
 * 
 */
void setEncodersVars();

/**
 * @brief Speed algorithm
 * 
 * @return float speed
 */
float setSpeed();

void setup()
{
  Serial.begin(9600);

  // ENLEVE UN OFFSET À LA DISTANCE POUR CENTRER LE ROBOT AU PARCOURS
  for (int i = 0; i < MAX_INDEX; i++)
  {
    float tmp = 0;
    tmp = (distance[i] * 3200) / TOUR_CM;
    distance[i] = tmp;
  }

  // TRANSFORME LES ANGLES EN PULSE D'ENCODEUR
  for (int i = 0; i < (MAX_INDEX - 1); i++)
  {
    float tmp = 0;
    tmp = (angle[i] * PI * WHEELS_RADIUS * PULSE_PER_TURN) / (HALF_CIRCLE_IN_DEG * TOUR_CM);
    angle[i] = tmp;
  }

  BoardInit();
}

void loop()
{

  if (digitalRead(RIGHT_BUMPER))
  {
    // POUR LES TEST
    run = false;
    mode = D;
  }
  if (digitalRead(BACK_BUMPER))
  {
    encodersReset(); // POUR LES TEST
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

unsigned int indexOfIndex = 0;

void moveAlgo()
{

  setEncodersVars();

  // PRINT DISTANCE ET ANGLE
  if (index == indexOfIndex)
  {
    Serial.println(distance[index]);
    Serial.println(angle[index]);
    indexOfIndex++;
  }

  switch (mode)
  {
  case A:

    if (abs(angle[index]) > abs(leftEncoder) && abs(angle[index]) > abs(rightEncoder))
    {

      if (angle[index] < 0)
      {
        if (abs(angle[index]) <= abs(leftEncoder)) 
        {
          motors(0, -1, ANGLE_SPEED);
        }
        else if (abs(angle[index]) <= abs(rightEncoder))
        {
          motors(1, 0, ANGLE_SPEED);
        }
        else
        {
          motors(1, -1, ANGLE_SPEED);
        }
      }
      else if (angle[index] > 0)
      {
        if (abs(angle[index]) <= abs(leftEncoder))
        {
          motors(0, 1, ANGLE_SPEED);
        }
        else if (abs(angle[index]) <= abs(rightEncoder))
        {
          motors(-1, 0, ANGLE_SPEED);
        }
        else
        {
          motors(-1, 1, ANGLE_SPEED);
        }
      }
    }
    else
    {
      motorsOff();
      index++;
      delay(100);
      encodersReset();
      mode = D;
    }
    break;

  case D:

    if (distance[index] > leftEncoder)
    {
      motors(1.0, 1.0, 1);
    }
    else
    {
      if (index >= MAX_INDEX)
      {
        run = false;
      }

      motorsOff();
      delay(100);
      encodersReset();
      mode = A;
    }
    break;

  default:
    Serial.println("ERROR NO MODE!");
    break;
  }
}

void motors(char left_motor, char right_motor, float speed)
{

  float pid = 0;

  if (mode == D)
  {
    pid = motorsPid();
    speed = setSpeed();
  }

  // char str[50];

  // sprintf(str, "\nG : %ld\nD : %ld",ENCODER_Read(M_GAUCHE),ENCODER_Read(M_DROITE));
  // Serial.print(str);

  MOTOR_SetSpeed(M_GAUCHE, speed * left_motor);
  MOTOR_SetSpeed(M_DROITE, speed * right_motor + (pid * right_motor));
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
  setEncodersVars();
}

void setEncodersVars()
{
  leftEncoder = ENCODER_Read(M_GAUCHE);
  rightEncoder = ENCODER_Read(M_DROITE);
}


float setSpeed()
{
  float speed = 0;

  if(distance[index] > (2 * MAX_DELTA_STEPS))
  {
    if(leftEncoder < MAX_DELTA_STEPS)
    {
      speed = SLOPE * leftEncoder + SPEED_OFFSET;
    }
    else if(distance[index] - 5000 < leftEncoder)
    {
      speed = SLOPE * (distance[index] - leftEncoder) + SPEED_OFFSET;
    }
    else
    {
      speed = 0.9;
    }
  }
  else
  {
    if(distance[index] * 0.5 > leftEncoder)
    {
      speed = SLOPE * leftEncoder + SPEED_OFFSET;
    }
    else
    {
      speed = SLOPE * (distance[index] - leftEncoder) + SPEED_OFFSET;
    }
  }

  return speed;
}