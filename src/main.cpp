#include <Arduino.h>
#include "credentials.h"
const char ssid[] = WIFI_SSID;
const char pass[] = WIFI_PASSWD;

#define BUILTIN_LED 2

ADC_MODE(ADC_VCC);

void setup() {
  Serial.begin(115200);
  pinMode(BUILTIN_LED, OUTPUT);
}

void loop() {
  Serial.println(ESP.getVcc());

  digitalWrite(BUILTIN_LED, LOW);
  delay(200);
  digitalWrite(BUILTIN_LED, HIGH); 
  delay(200); 
}