#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "menu_menu.h"

#define LED_STATUS 32
#define BUZZER 27
#define ONE_WIRE 4

#define M1_S1 14
#define M1_S2 12
#define M2_S1 13
#define M2_S2 5

#define M1_A_LO 18
#define M1_A_HI 15
#define M1_B_LO 21
#define M1_B_HI 19

#define M2_A_LO 23
#define M2_A_HI 22
#define M2_B_LO 26
#define M2_B_HI 25

Adafruit_NeoPixel pixels(1, LED_STATUS, NEO_GRB);

const int M1_A_CHANNEL = 0;

void setup()
{

  Serial.begin(115200);

  setupMenu();  
  pinMode(ONE_WIRE, INPUT);

  pinMode(M1_S1, INPUT);
  pinMode(M1_S2, INPUT);

  pinMode(M2_S1, INPUT);
  pinMode(M2_S2, INPUT);

  pinMode(BUZZER, OUTPUT);

  pinMode(M1_A_LO, OUTPUT);
  pinMode(M1_A_HI, OUTPUT);
  pinMode(M1_B_LO, OUTPUT);
  pinMode(M1_B_HI, OUTPUT);
  pinMode(M2_A_LO, OUTPUT);
  pinMode(M2_A_HI, OUTPUT);
  pinMode(M2_B_LO, OUTPUT);
  pinMode(M2_B_HI, OUTPUT);

  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 10, 0));
  pixels.show();
  Serial.println("Start done");

  const int freq = 2000;

  const int resolution = 8;

  ledcSetup(M1_A_CHANNEL, freq, resolution);
  ledcAttachPin(M1_B_HI, M1_A_CHANNEL);


}

void loop()
{
  taskManager.runLoop();

  int pwmin = pulseIn(M1_S1, HIGH, 200000);
  int pwm_clean = constrain(pwmin, 1000, 2000);
  int out_duty_cycle = map(pwm_clean,1000,2000,20,5000);
  Serial.println(out_duty_cycle);
  //ledcWrite(M1_A_CHANNEL, out_duty_cycle);
  //digitalWrite(M1_A_LO, HIGH);
  delay(100);
}