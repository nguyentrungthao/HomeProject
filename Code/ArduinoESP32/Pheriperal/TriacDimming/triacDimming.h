#ifndef _TRIAC_H_
#define _TRIAC_H_

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/timer.h"

#define triacHOLD_TIME 100
#define TRIAC_LOW_LIMIT 700
#define TRIAC_HIGH_LIMIT 9600

class triac {
public:
  triac(gpio_num_t pin, timer_group_t grp = TIMER_GROUP_0, timer_idx_t idx = TIMER_0);

  static void configACDETPIN(gpio_num_t acdet_pin);
  uint8_t u8GetFrequence() {
    return triac::u16TriacHighLimit > 9000 ? 50 : 60;
  }

  void init(void);
  void SetTimeOverFlow(uint16_t timeOverFlow);
  void TurnOnTriac();
  void TurnOffTriac();

  void acdet_intr_handler_in_task();
private:
  // ACDET
  static gpio_num_t acdet;
//   static uint8_t u8Frequence;
  static uint16_t u16TriacHighLimit;  // biểu thị thay cho tần số 
  static void acdet_frequence_measure_callback(void* ptr);
  static void vACDetFrequenceMeasure(gpio_num_t acdet);

  // hardware
  gpio_num_t pin;
  timer_group_t grp;
  timer_idx_t idx;

  // control
  bool dis_timer = false;
  uint16_t timeOverFlow;
  bool RunStatus = false;

  // task
  TaskHandle_t TimerTimoutTaskHandle;

  static void TimerTimoutTask(void* ptr);
  static void IRAM_ATTR acdet_intr_handler_in_isr(void* arg);
  static bool IRAM_ATTR timer_triac_intr_handler(void* arg);
};

#endif