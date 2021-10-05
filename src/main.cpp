#include <Arduino.h>
#include <LibRobus.h>

#define M_GAUCHE 0
#define M_DROITE 1
#define KP 0.0008

bool test = true;

void CorrigerErreurMoteur(float vitesse);

void setup()
{
  Serial.begin(9600);
  BoardInit();
}

void loop()
{

  if (test)
  {
    MOTOR_SetSpeed(M_GAUCHE,0.5);
    MOTOR_SetSpeed(M_DROITE,0.2);

    CorrigerErreurMoteur(0.5);

    test =false;
  }

  char str[50];

  delay(1000);

  sprintf(str, "Encodeur 1 : %ld\nEncodeur 2 : %ld\n", ENCODER_Read(M_GAUCHE),ENCODER_Read(M_DROITE));
  Serial.print(str);

}

// Fonction corrigeant l'erreur du moteur droit en le considérant l'esclave du moteur gauche
void CorrigerErreurMoteur(float vitesse)
{
  float errVitesse = (ENCODER_Read(M_GAUCHE)-ENCODER_Read(M_DROITE));
  float corrErr = vitesse + (errVitesse*KP);

  MOTOR_SetSpeed(M_DROITE,corrErr);
}