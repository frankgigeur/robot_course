/**
 * @file main.cpp
 * @author Ro13bots with attitUdeS
 * @brief Code for a robot moving from point A to point B by making angles
 * @version 0.82
 * @date 2021-10-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <Arduino.h>
#include <math.h>
#include <LibRobus.h>
#include <QTRSensors.h>

#define TOUR_CM 23.939
#define PULSE_PER_TURN 3200
#define HALF_CIRCLE_IN_DEG 180
// #define WHEELS_RADIUS_45 9.23
// #define WHEELS_RADIUS_90 9.325 // 13B
#define WHEELS_RADIUS_45 9.6
#define WHEELS_RADIUS_90 9.6 // 13A
#define M_GAUCHE 0
#define M_DROIT 1
#define FRONT_BUMPER 26
#define LEFT_BUMPER 27
#define BACK_BUMPER 28
#define RIGHT_BUMPER 29
#define MAX_DELTA_STEPS 2000
#define SLOPE 0.000375
#define SPEED_OFFSET 0.15
#define DELAY_MBT 100
#define MAX_INDEX 19 // NEED TO BE SETTED

// Suiveur de ligne
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
#define KP 0.0025
#define KD 0.0
double ErreurPrecedente = 0;
#define Centre 3500
#define VITESSE_MAX 0.5


bool run = false;
float distance[19] =  {223, 50.5, 45, 50.5, 31, 46.5, 61, 49, 85, 85, 49, 61, 46.5, 31, 50.5, 45, 50.5, 223};
float angle[19] =     {90, -90  , -90, 90, -45, 90, -45, -20, 180, 20, 45, -90, 45, -90, 90, 90, -90};
// float distance[15] = {100.5, 49.5, 47.5, 49.5, 32.25, 52, 54.5, 54.5, 52, 32.25, 49.5, 47.5, 49.5, 100.5};  // Parcours maison
// float angle[14] = {90.0, -90.0, -90.0, 90.0, 45.0, -45.0, 185.0, 45.0, -45.0, -90.0, 90, 90, -90};
long leftEncoder = 0;
long rightEncoder = 0;
unsigned int index = 0;
const float kp = 0.0025;


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
  BoardInit();

  // TRANSFORME LES DISTANCES EN PULSE D'ENCODEUR
  for (int i = 0; i < MAX_INDEX; i++)
  {
    float tmp = 0;
    tmp = (distance[i] * PULSE_PER_TURN) / TOUR_CM;
    distance[i] = tmp;
  }

  // TRANSFORME LES ANGLES EN PULSE D'ENCODEUR
  for (int i = 0; i < (MAX_INDEX - 1); i++)
  {
    float tmp = 0;
    float wheelsRadius = 0;
    if(abs(angle[i]) < 90)
    {
      wheelsRadius = WHEELS_RADIUS_45;
    }
    else
    {
      wheelsRadius = WHEELS_RADIUS_90;
    }
    tmp = (angle[i] * PI * wheelsRadius * PULSE_PER_TURN) / (HALF_CIRCLE_IN_DEG * TOUR_CM);
    angle[i] = floor(tmp);
  }
}

bool testMode = false;
bool oneSet = true;
long accAngle = 0;
long accLAngle = 0;
long accRAngle = 0;
long errLAngle = 0;
long errRAngle = 0;



void testFunc()
{
  setEncodersVars();
  float vitesse = 0.2;
  if((angle[0] - 500) < leftEncoder)
  {
    vitesse = 0.1;
  }

  if(angle[0] > abs(rightEncoder))
  {
    MOTOR_SetSpeed(M_DROIT,vitesse * -1);
  }
  else
  {
    MOTOR_SetSpeed(M_DROIT,-0.01);
  }

  if(angle[0] > abs(leftEncoder))
  {
    MOTOR_SetSpeed(M_GAUCHE,vitesse * 1);
  }
  else
  {
    MOTOR_SetSpeed(M_GAUCHE,0.01);
  }

  if(angle[0] <= abs(rightEncoder) && angle[0] <= abs(leftEncoder))
  {
    
    delay(100);
    setEncodersVars();
    Serial.print("Angle : ");
    Serial.println(angle[0]);
    Serial.print("EncodeurL : ");
    Serial.println(leftEncoder);
    Serial.print("EncodeurR : ");
    Serial.println(rightEncoder);
    encodersReset();
  }


  // if((angle[0] - 500) < leftEncoder || (angle[0] - 500) < abs(rightEncoder))
  // {
  //   vitesse = 0.12;
  // }

  // if(angle[0] > leftEncoder || angle[0] > abs(rightEncoder))
  // {
  //   if(angle[0] > leftEncoder)
  //   {
  //   MOTOR_SetSpeed(M_GAUCHE,vitesse);
  //   }
  //   else
  //   {
  //     MOTOR_SetSpeed(M_GAUCHE,-0.08);
  //   }
  //   if(angle[0] > rightEncoder * -1)
  //   {
  //   MOTOR_SetSpeed(M_DROIT,vitesse * -1);
  //   }
  //   else
  //   {
  //     MOTOR_SetSpeed(M_DROIT,0.08);
  //   }
  // }
  // else{
  //   MOTOR_SetSpeed(M_DROIT,-0.08);
  //   MOTOR_SetSpeed(M_GAUCHE,-0.08);
  //   motorsOff();
  //   delay(2000);
  //   Serial.print("Angle : ");
  //   Serial.println(angle[0]);
  //   Serial.print("EncodeurL : ");
  //   Serial.println(leftEncoder);
  //   Serial.print("EncodeurR : ");
  //   Serial.println(rightEncoder);
  //   encodersReset();
  // }
  
}

void loop()
{
  if (digitalRead(RIGHT_BUMPER))
  {
    // POUR LES TEST
    run = false;
    mode = D;
  }

  if(digitalRead(LEFT_BUMPER))
  {
    testMode = true;
  }
  if(testMode)
  {
    testFunc();
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
    // motorsOff();
  }
}

// unsigned int indexOfIndex = 0;

void moveAlgo()
{
  
  setEncodersVars();
  float angleSpeed = 0.3;

  // PRINT DISTANCE ET ANGLE
  // if (index == indexOfIndex)
  // {
  //   Serial.println(distance[index]);
  //   Serial.println(angle[index]);
  //   indexOfIndex++;
  // }

  switch (mode)
  {
  case A:

    if ((abs(angle[index]) -500) < abs(leftEncoder) || (abs(angle[index]) -500) < abs(rightEncoder))
    {
      angleSpeed = 0.1;
    }

    if (abs(angle[index]) > abs(leftEncoder) || abs(angle[index]) > abs(rightEncoder))
    {

      if (angle[index] < 0)
      {
        if (abs(angle[index]) > abs(leftEncoder) && abs(angle[index]) > abs(rightEncoder))
        {
          motors(1, -1, angleSpeed);
        }
        else
        {
          if (abs(angle[index]) <= abs(rightEncoder))
          {
            motors(1, 0, angleSpeed);
          }
          if (abs(angle[index]) <= abs(leftEncoder))
          {
            motors(0, -1, angleSpeed);
          }
        }
      }
      else if (angle[index] > 0)
      {
        if (abs(angle[index]) > abs(leftEncoder) && abs(angle[index]) > abs(rightEncoder))
        {
          motors(-1, 1, angleSpeed);
        }
        else
        {
          if (abs(angle[index]) <= abs(rightEncoder))
          {
            motors(-1, 0, angleSpeed);
          }
          if (abs(angle[index]) <= abs(leftEncoder))
          {
            motors(0, 1, angleSpeed);
          }
        }
      }
    }
    else
    {
      motorsOff();

      Serial.print("Angle : ");
      Serial.println(angle[index]);
      Serial.print("LeftEncoder : ");
      Serial.println(leftEncoder);
      Serial.print("RightEncoder : ");
      Serial.println(rightEncoder);
      index++;
      delay(DELAY_MBT);
      
      encodersReset();
      mode = D;
    }
    break;
  case D:

    if (distance[index] > leftEncoder)
    {
      motors(1, 1, 1);
    }
    else
    {
      if (index >= MAX_INDEX)
      {
        run = false;
      }

      motorsOff();
      delay(DELAY_MBT);
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

  MOTOR_SetSpeed(M_GAUCHE, speed * left_motor);
  MOTOR_SetSpeed(M_DROIT, speed * right_motor + (pid * right_motor));
}

float motorsPid()
{

  long error = abs(leftEncoder) - abs(rightEncoder);
  return (error * kp);
}

void motorsOff()
{
  MOTOR_SetSpeed(M_GAUCHE, 0.0);
  MOTOR_SetSpeed(M_DROIT, 0.0);
}

void encodersReset()
{
  ENCODER_Reset(M_GAUCHE);
  ENCODER_Reset(M_DROIT);
  setEncodersVars();
}

void setEncodersVars()
{
  leftEncoder = ENCODER_Read(M_GAUCHE);
  rightEncoder = ENCODER_Read(M_DROIT);
}

float setSpeed()
{
  float speed = 0;

  if (distance[index] > (2 * MAX_DELTA_STEPS))
  {
    if (leftEncoder < MAX_DELTA_STEPS)
    {
      speed = SLOPE * leftEncoder + SPEED_OFFSET;
    }
    else if (distance[index] - MAX_DELTA_STEPS < leftEncoder)
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
    if (distance[index] * 0.5 > leftEncoder)
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

void CalibrationSuiveurLigne()
{
    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    qtr.setEmitterPin(2);

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

    // analogRead() takes about 0.1 ms on an AVR.
    // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
    // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
    // Call calibrate() 400 times to make calibration take about 10 seconds.
    for (uint16_t i = 0; i < 3000; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}

void SuiveurLigneSetup()
{
    motorsOff();

  // Initialisation Suiveur de ligne
    CalibrationSuiveurLigne();
}

void SuiveurLigneLoop()
{
    // Trouver la position de la ligne
    uint16_t position = qtr.readLineBlack(sensorValues);

    int erreur = Centre - position;
    int PID = KP * erreur + KD * (erreur - ErreurPrecedente);
    ErreurPrecedente = erreur;

    MOTOR_SetSpeed(M_GAUCHE, constrain(VITESSE_MAX - PID, 0, VITESSE_MAX));
    MOTOR_SetSpeed(M_DROIT, constrain(VITESSE_MAX + PID, 0, VITESSE_MAX));
}