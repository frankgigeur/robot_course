#include <Arduino.h>
#include <LibRobus.h>

#define TOUR_CM 23.9389360203542244771
#define M_GAUCHE 0
#define M_DROITE 1
#define BACK_BUMPER 13

bool run = false;
const float distance[10] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0};
const float angle[9] = {1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0};


void moveAlgo();
void motors(char left_motor, char right_motor);
float motorsPid(unsigned long long value_encoder);

void setup()
{
  Serial.begin(9600);
  BoardInit();
}

void loop()
{
  if (digitalRead(BACK_BUMPER))
  {
    run = true;
  }
  else if (run)
  {
    moveAlgo();
  }
  else
  {
    motors(0, 0);
  }
}

void moveAlgo()
{
}

void motors(char left_motor, char right_motor)
{

float speed = motorsPid(ENCODER_Read(M_GAUCHE));



MOTOR_SetSpeed(M_GAUCHE,speed * left_motor);
MOTOR_SetSpeed(M_GAUCHE,1.0 * right_motor);

}

float motorsPid(unsigned long long value_encoder)
{
  float vitesse = 0;


  return vitesse;
}