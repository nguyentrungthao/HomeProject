#include "triacDimming.h"
#include <AnhLABV01HardWare.h>

#define triacPin TRIAC4_PIN
#define acdetPin ACDET_PIN

triac t1((gpio_num_t)triacPin, TIMER_GROUP_0, TIMER_0);
// triac t2((gpio_num_t)TRIAC2_PIN, TIMER_GROUP_0, TIMER_1);
// triac t3((gpio_num_t)TRIAC3_PIN, TIMER_GROUP_1, TIMER_0);
// triac t4((gpio_num_t)TRIAC4_PIN, TIMER_GROUP_1, TIMER_1);

void setup() {
  Serial.begin(115200);

  // put your setup code here, to run once:
  triac::configACDETPIN((gpio_num_t)acdetPin);

  t1.init();
  // t2.init();
  // t3.init();
  // t4.init();

  t1.TurnOnTriac();
  // t2.TurnOnTriac();
  // t3.TurnOnTriac();
  // t4.TurnOnTriac();

  // t2.SetTimeOverFlow(150);
  // t3.SetTimeOverFlow(150);
  // t4.SetTimeOverFlow(150);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    String str = Serial.readString();
    t1.SetTimeOverFlow(str.toInt());
  }
  delay(1);
}
