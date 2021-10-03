#include <Arduino.h>
#include <LibRobus.h>

#define M_GAUCHE 0
#define M_DROITE 1

bool test = true;

void setup()
{
  Serial.begin(9600);
  BoardInit();
}

void loop()
{

  if (test)
  {
    MOTOR_SetSpeed(M_GAUCHE,0.9);
    MOTOR_SetSpeed(M_DROITE,0.9061);
    test =false;
  }

  char str[50];

  delay(1000);

  sprintf(str, "Encodeur 1 : %ld\nEncodeur 2 : %ld\n %ld", ENCODER_Read(M_GAUCHE),ENCODER_Read(M_DROITE));


}